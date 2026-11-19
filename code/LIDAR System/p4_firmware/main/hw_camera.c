#include "hw_camera.h"

#include <stdbool.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp_ch34x.h"
#include "wro_config.h"

static const char *TAG = "camera";

#define RING_BYTES      8192
#define USB_RX_BUF      512
#define USB_TASK_STACK  4096
#define TX_TIMEOUT_MS   100

static RingbufHandle_t s_ring;
static cdc_acm_dev_hdl_t s_dev;          /* NULL si no hay S3 conectado */
static volatile uint32_t s_bytes, s_lost;

/* Llega dentro de la tarea del driver USB: solo copia al buffer, sin esperar. */
static bool on_data(const uint8_t *data, size_t len, void *user_arg)
{
    (void)user_arg;
    s_bytes += (uint32_t)len;
    if (xRingbufferSend(s_ring, data, len, 0) != pdTRUE) s_lost += (uint32_t)len;
    return true;
}

static void on_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "S3 desconectado");
        cdc_acm_host_close(event->data.cdc_hdl);
        s_dev = NULL;
    } else if (event->type == CDC_ACM_HOST_ERROR) {
        ESP_LOGE(TAG, "error USB: %d", (int)event->data.error);
    }
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

/* Abre el S3 cuando aparece (puente CH340 de la placa; si es USB nativo, CDC-ACM) y fija el baudio. */
static void open_task(void *arg)
{
    (void)arg;
    esp_err_t last_err = ESP_OK;
    int64_t last_report_us = 0;
    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 64,
        .in_buffer_size = USB_RX_BUF,
        .event_cb = on_event,
        .data_cb = on_data,
        .user_arg = NULL,
    };
    const cdc_acm_line_coding_t coding = {
        .dwDTERate = CONFIG_WRO_CAM_BAUD,
        .bCharFormat = 0,      /* 1 bit de parada */
        .bParityType = 0,
        .bDataBits = 8,
    };
    for (;;) {
        if (!s_dev) {
            cdc_acm_dev_hdl_t dev = NULL;
            const char *kind = "CH340";
            esp_err_t err = ch34x_vcp_open(CH34X_PID_AUTO, 0, &dev_cfg, &dev);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "driver CH34x no abrio el S3: %s", esp_err_to_name(err));
                kind = "CDC generico";
                err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0, &dev_cfg, &dev);
            }
            if (err == ESP_OK) cdc_acm_host_desc_print(dev);   /* diagnostico: VID/PID y descriptores */
            const int64_t now_us = esp_timer_get_time();
            if (err != ESP_OK && (err != last_err || now_us - last_report_us > 3000000)) {
                /* Diagnostico: el driver rechaza el dispositivo o aun no hay ninguno. */
                usb_host_lib_info_t info = {0};
                usb_host_lib_info(&info);
                ESP_LOGW(TAG, "abrir S3: %s (dispositivos USB enumerados: %d)", esp_err_to_name(err), (int)info.num_devices);
                last_err = err;
                last_report_us = now_us;
            }
            if (err == ESP_OK) {
                /* No se tocan DTR/RTS: en la placa del S3 mueven EN y BOOT (reinicio / modo descarga). */
                if (cdc_acm_host_line_coding_set(dev, &coding) != ESP_OK)
                    ESP_LOGW(TAG, "no se pudo fijar %d baudios", CONFIG_WRO_CAM_BAUD);
                s_dev = dev;
                ESP_LOGI(TAG, "S3 conectado por USB (%s), %d 8N1", kind, CONFIG_WRO_CAM_BAUD);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#define CAM_UART UART_NUM_2

static volatile bool s_uart_on;

/* Modo UART: el S3 manda la tabla a 921600 por su UART0; aqui solo se vuelca al buffer. */
static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;) {
        const int n = uart_read_bytes(CAM_UART, buf, sizeof buf, pdMS_TO_TICKS(5));
        if (n > 0) on_data(buf, (size_t)n, NULL);
    }
}

void hw_camera_init(void)
{
    s_ring = xRingbufferCreate(RING_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (!CONFIG_WRO_CAM_USB) {
        const uart_config_t cfg = {
            .baud_rate = CONFIG_WRO_CAM_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_driver_install(CAM_UART, 4096, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(CAM_UART, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(CAM_UART, CONFIG_WRO_CAM_UART_TX_GPIO, CONFIG_WRO_CAM_UART_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        s_uart_on = true;
        xTaskCreate(uart_rx_task, "cam_uart", 3072, NULL, 3, NULL);
        ESP_LOGI(TAG, "S3 por UART: RX GPIO%d, TX GPIO%d, %d 8N1", CONFIG_WRO_CAM_UART_RX_GPIO,
                 CONFIG_WRO_CAM_UART_TX_GPIO, CONFIG_WRO_CAM_BAUD);
        return;
    }
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    /* Diagnostico: que detecta el puerto host (conexion, reset, enumeracion). */
    static const char *const dbg_tags[] = {"HCD DWC", "HUB", "ENUM", "USBH", "USB HOST", "usb_phy", "EXT_PORT"};
    for (size_t i = 0; i < sizeof dbg_tags / sizeof dbg_tags[0]; ++i)
        esp_log_level_set(dbg_tags[i], ESP_LOG_DEBUG);
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_lib_task, "usb_lib", USB_TASK_STACK, NULL, 2, NULL);

    const cdc_acm_host_driver_config_t drv_cfg = {
        .driver_task_stack_size = USB_TASK_STACK,
        .driver_task_priority = 3,
        .xCoreID = 0,
        .new_dev_cb = NULL,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&drv_cfg));
    xTaskCreate(open_task, "cam_open", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "host USB listo; esperando al S3");
}

size_t hw_camera_read(void *ctx, uint8_t *buffer, size_t capacity)
{
    (void)ctx;
    if (!s_ring) return 0;
    size_t item_size = 0;
    uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(s_ring, &item_size, 0, capacity);
    if (!item) return 0;
    memcpy(buffer, item, item_size);
    vRingbufferReturnItem(s_ring, item);
    return item_size;
}

bool hw_camera_write(const uint8_t *data, size_t len)
{
    if (s_uart_on) return len && uart_write_bytes(CAM_UART, data, len) == (int)len;
    cdc_acm_dev_hdl_t dev = s_dev;
    if (!dev || !len) return false;
    return cdc_acm_host_data_tx_blocking(dev, data, len, TX_TIMEOUT_MS) == ESP_OK;
}

void hw_camera_stats(uint32_t *bytes, uint32_t *lost, bool *connected)
{
    *bytes = s_bytes;
    *lost = s_lost;
    *connected = s_uart_on ? s_bytes > 0 : s_dev != NULL;
}

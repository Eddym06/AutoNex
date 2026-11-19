#include "vision_uart_link.h"

#include "sensor_table_protocol.h"
#include "vision_table.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

#if CONFIG_WRO_LINK_ON_CONSOLE_UART
#define VISION_LINK_UART UART_NUM_0
#define VISION_LINK_BAUD CONFIG_WRO_LINK_BAUD
#else
#define VISION_LINK_UART UART_NUM_1
#define VISION_LINK_BAUD 230400
#endif
#define VISION_LINK_TX_GPIO CONFIG_WRO_LINK_TX_GPIO
#if CONFIG_WRO_LINK_RX_GPIO >= 0
#define VISION_LINK_RX_GPIO CONFIG_WRO_LINK_RX_GPIO
#else
#define VISION_LINK_RX_GPIO UART_PIN_NO_CHANGE
#endif

esp_err_t vision_uart_link_init(void)
{
#if CONFIG_WRO_LINK_ON_CONSOLE_UART
    /* UART0 ya es la consola (vision_config_rx instala su driver de recepcion). Se sube el baudio y se
     * silencian los logs: un log en mitad de una trama la corrompe. */
    esp_err_t error = uart_driver_install(VISION_LINK_UART, 2048, 0, 0, NULL, 0);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;
    error = uart_set_baudrate(VISION_LINK_UART, VISION_LINK_BAUD);
    if (error != ESP_OK) return error;
    esp_log_level_set("*", ESP_LOG_ERROR);
    return ESP_OK;
#else
    const uart_config_t config = {
        .baud_rate = VISION_LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    /* A receive buffer exists only when the P4 may send configuration. */
    const int rx_buffer = CONFIG_WRO_LINK_RX_GPIO >= 0 ? 512 : 0;
    esp_err_t error = uart_driver_install(VISION_LINK_UART, rx_buffer > 0 ? rx_buffer : 2048,
                                          0, 0, NULL, 0);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;
    error = uart_param_config(VISION_LINK_UART, &config);
    if (error != ESP_OK) return error;
    return uart_set_pin(VISION_LINK_UART, VISION_LINK_TX_GPIO, VISION_LINK_RX_GPIO,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif
}

esp_err_t vision_uart_link_send(const vision_result_t *result, int width,
                                uint16_t sequence, uint32_t timestamp_ms)
{
    uint8_t packet[WRO_TABLE_MAX_SIZE];
    wro_sensor_table_t table;
    vision_table_build(result, width, sequence, timestamp_ms, &table);
    const size_t length = wro_sensor_table_encode(&table, packet, sizeof(packet));
    if (!length) return ESP_ERR_INVALID_ARG;
    const int written = uart_write_bytes(VISION_LINK_UART, packet, length);
    return written == (int)length ? ESP_OK : ESP_FAIL;
}

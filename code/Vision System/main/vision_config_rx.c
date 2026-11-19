#include "vision_config_rx.h"

#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "vision_config_cmd.h"
#include "vision_settings.h"

#define CONSOLE_UART UART_NUM_0
#define LINK_UART UART_NUM_1

static const char *TAG = "wro_cfg_rx";

static void handle(vision_cmd_parser_t *parser, vision_cmd_event_t ev, vision_config_t *cfg)
{
    switch (ev) {
    case VISION_CMD_CHANGED:
        vision_settings_submit(cfg, false);
        ESP_LOGI(TAG, "configuration changed (not saved: type 'save')");
        break;
    case VISION_CMD_SAVE:
        vision_settings_save();
        break;
    case VISION_CMD_DEFAULTS:
        vision_settings_reset();
        vision_settings_current(cfg);
        ESP_LOGI(TAG, "built-in defaults restored");
        break;
    case VISION_CMD_DUMP: {
        static char text[3072];
        vision_settings_current(cfg);
        vision_config_format(cfg, text, sizeof(text));
        printf("%s", text);
        break;
    }
    case VISION_CMD_REPLY: ESP_LOGI(TAG, "%s", parser->reply); break;
    case VISION_CMD_ERROR: ESP_LOGW(TAG, "%s", parser->reply); break;
    default: break;
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    static vision_cmd_parser_t console, link;
    static vision_config_t working;
    vision_cmd_reset(&console);
    vision_cmd_reset(&link);
    vision_settings_current(&working);
    uint8_t chunk[64];
    for (;;) {
        int n = uart_read_bytes(CONSOLE_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; ++i) {
            vision_cmd_event_t ev = vision_cmd_feed(&console, chunk[i], &working);
            if (ev != VISION_CMD_NONE) handle(&console, ev, &working);
        }
#if CONFIG_WRO_LINK_RX_GPIO >= 0
        n = uart_read_bytes(LINK_UART, chunk, sizeof(chunk), 0);
        for (int i = 0; i < n; ++i) {
            vision_cmd_event_t ev = vision_cmd_feed(&link, chunk[i], &working);
            if (ev != VISION_CMD_NONE) handle(&link, ev, &working);
        }
#endif
    }
}

esp_err_t vision_config_rx_start(void)
{
    /* The log path keeps writing through the console driver; installing the
     * receive side only adds a buffer for incoming bytes. */
    esp_err_t err = uart_driver_install(CONSOLE_UART, 512, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    (void)LINK_UART;
    if (xTaskCreate(rx_task, "wro_cfg_rx", 4096, NULL, 2, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

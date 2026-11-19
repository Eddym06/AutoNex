#include "hw_pinscan.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "wro_config.h"

static const char *TAG = "pinscan";

#define SCAN_FIRST_GPIO 0
#define SCAN_LAST_GPIO  54
#define SCAN_MS         500
#define MIN_TRANSITIONS 50

void hw_pinscan_run(void)
{
    static uint32_t count[SCAN_LAST_GPIO + 1];
    static uint8_t last[SCAN_LAST_GPIO + 1];
    static bool used[SCAN_LAST_GPIO + 1];
    for (int g = SCAN_FIRST_GPIO; g <= SCAN_LAST_GPIO; ++g) {
        /* 24/25 son el USB-JTAG (consola y flasheo): no se tocan. */
        if (g == 24 || g == 25 || !GPIO_IS_VALID_GPIO(g)) continue;
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << g,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        if (gpio_config(&cfg) != ESP_OK) continue;
        used[g] = true;
        last[g] = (uint8_t)gpio_get_level((gpio_num_t)g);
    }
    const int64_t end = esp_timer_get_time() + SCAN_MS * 1000;
    while (esp_timer_get_time() < end) {
        for (int g = SCAN_FIRST_GPIO; g <= SCAN_LAST_GPIO; ++g) {
            if (!used[g]) continue;
            const uint8_t v = (uint8_t)gpio_get_level((gpio_num_t)g);
            if (v != last[g]) { last[g] = v; ++count[g]; }
        }
    }
    int found = 0;
    for (int g = SCAN_FIRST_GPIO; g <= SCAN_LAST_GPIO; ++g) {
        if (used[g] && count[g] >= MIN_TRANSITIONS) {
            ESP_LOGI(TAG, "GPIO%d: %lu cambios en %d ms", g, (unsigned long)count[g], SCAN_MS);
            ++found;
        }
        if (used[g]) gpio_reset_pin((gpio_num_t)g);
    }
    if (!found) ESP_LOGW(TAG, "ningun GPIO libre se mueve: el LiDAR no transmite (alimentacion, cable o motor parado)");
}

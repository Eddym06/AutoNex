#include "hw_start.h"

#include "driver/gpio.h"
#include "sdkconfig.h"

static bool s_was_down;

void hw_start_init(void)
{
    if (CONFIG_WRO_START_GPIO < 0) return;
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << (CONFIG_WRO_START_GPIO & 63),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

bool hw_start_pressed(void *ctx)
{
    (void)ctx;
    if (CONFIG_WRO_START_GPIO < 0) return false;
    const bool down = gpio_get_level((gpio_num_t)CONFIG_WRO_START_GPIO) == 0;
    const bool edge = down && !s_was_down;
    s_was_down = down;
    return edge;
}

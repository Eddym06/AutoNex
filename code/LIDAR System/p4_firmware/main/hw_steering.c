#include "hw_steering.h"

#include <math.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "wro_config.h"

static bool ready;

bool hw_steering_init(void)
{
    const int l = CONFIG_WRO_SERVO_LEFT_US, c = CONFIG_WRO_SERVO_CENTER_US, r = CONFIG_WRO_SERVO_RIGHT_US;
    ready = false;
    /* Los extremos pueden estar invertidos, pero el centro debe quedar entre ambos. */
    if (!CONFIG_WRO_ACKERMANN_HW_CONFIGURED || CONFIG_WRO_SERVO_GPIO < 0 || CONFIG_WRO_SERVO_GPIO == CONFIG_WRO_MOTOR_IN1_GPIO ||
        CONFIG_WRO_SERVO_GPIO == CONFIG_WRO_MOTOR_IN2_GPIO || l < 500 || l > 2500 || c < 500 || c > 2500 ||
        r < 500 || r > 2500 || !((l < c && c < r) || (r < c && c < l)) ||
        CONFIG_WRO_SERVO_LEFT_DEG <= 0 || CONFIG_WRO_SERVO_LEFT_DEG >= 90 ||
        CONFIG_WRO_SERVO_RIGHT_DEG <= 0 || CONFIG_WRO_SERVO_RIGHT_DEG >= 90) {
        ESP_LOGW("steering", "servo sin calibrar: traccion bloqueada");
        return false;
    }
    ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_1, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK};
    if (ledc_timer_config(&timer) != ESP_OK) return false;
    ledc_channel_config_t channel = {.gpio_num = CONFIG_WRO_SERVO_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_1,
        .duty = (uint32_t)((float)c * 65535.0f / 20000.0f), .hpoint = 0};
    if (ledc_channel_config(&channel) != ESP_OK) return false;
    ready = true;
    return true;
}

bool hw_steering_ready(void) { return ready; }

void hw_steering_set(float degrees)
{
    if (!ready) return;
    if (!isfinite(degrees)) degrees = 0.0f;
    float ratio = degrees >= 0.0f ? degrees / CONFIG_WRO_SERVO_RIGHT_DEG : -degrees / CONFIG_WRO_SERVO_LEFT_DEG;
    if (ratio > 1.0f) ratio = 1.0f;
    const int end = degrees >= 0.0f ? CONFIG_WRO_SERVO_RIGHT_US : CONFIG_WRO_SERVO_LEFT_US;
    const float pulse = CONFIG_WRO_SERVO_CENTER_US + ratio * (end - CONFIG_WRO_SERVO_CENTER_US);
    const uint32_t duty = (uint32_t)(pulse * 65535.0f / 20000.0f + 0.5f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

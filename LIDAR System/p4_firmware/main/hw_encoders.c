#include "hw_encoders.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "wro_config.h"
#include "axle_encoder.h"

#define AXLE_COUNT_LIMIT 30000
#define PI_F 3.14159265358979f

static pcnt_unit_handle_t s_unit;
static bool s_ready;

void hw_encoders_init(void)
{
    const int a = CONFIG_WRO_AXLE_ENCODER_A_GPIO, b = CONFIG_WRO_AXLE_ENCODER_B_GPIO;
    s_ready = false;
    if (!CONFIG_WRO_ACKERMANN_HW_CONFIGURED || a < 0 || b < 0 || a == b ||
        a == CONFIG_WRO_SERVO_GPIO || b == CONFIG_WRO_SERVO_GPIO ||
        a == CONFIG_WRO_MOTOR_IN1_GPIO || a == CONFIG_WRO_MOTOR_IN2_GPIO ||
        b == CONFIG_WRO_MOTOR_IN1_GPIO || b == CONFIG_WRO_MOTOR_IN2_GPIO) {
        ESP_LOGW("encoder", "encoder A/B sin configurar o pines en conflicto");
        return;
    }
    pcnt_unit_config_t unit = {.high_limit = AXLE_COUNT_LIMIT, .low_limit = -AXLE_COUNT_LIMIT,
                               .flags.accum_count = 1};
    ESP_ERROR_CHECK(pcnt_new_unit(&unit, &s_unit));
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &filter));
    pcnt_chan_config_t config_a = {.edge_gpio_num = a, .level_gpio_num = b};
    pcnt_chan_config_t config_b = {.edge_gpio_num = b, .level_gpio_num = a};
    pcnt_channel_handle_t chan_a, chan_b;
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &config_a, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &config_b, &chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, AXLE_COUNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, -AXLE_COUNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit));
    s_ready = true;
}

bool hw_encoders_ready(void) { return s_ready; }

static bool axle_degrees(float *degrees)
{
    int count;
    if (!s_ready || pcnt_unit_get_count(s_unit, &count) != ESP_OK) return false;
    *degrees = axle_encoder_degrees(count, CONFIG_WRO_AXLE_ENCODER_COUNTS_PER_REV,
                                    CONFIG_WRO_MOTOR_GEAR_TEETH, CONFIG_WRO_DIFF_GEAR_TEETH,
                                    CONFIG_WRO_AXLE_ENCODER_INVERT ? -1 : 1);
    return true;
}

void hw_encoders_set_direction(int left_dir, int right_dir)
{
    /* La cuadratura da el sentido medido, no el sentido de la orden. */
    (void)left_dir; (void)right_dir;
}

void hw_encoders_position_m(float *left_m, float *right_m)
{
    float deg = 0.0f;
    if (CONFIG_WRO_AXLE_ENCODER_CALIBRATED) (void)axle_degrees(&deg);
    /* Dos campos de telemetria heredados: ambos representan el mismo eje. */
    *left_m = *right_m = deg * (PI_F / 180.0f) * (0.001f * CONFIG_WRO_WHEEL_RADIUS_MM);
}

void hw_encoders_feed(wro_controller_t *c, uint64_t t_us)
{
    float deg;
    if (CONFIG_WRO_AXLE_ENCODER_CALIBRATED && axle_degrees(&deg))
        wro_controller_axle_encoder(c, t_us, deg);
}

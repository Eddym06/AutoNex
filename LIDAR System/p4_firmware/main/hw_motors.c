#include "hw_motors.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "wro_config.h"
#include "hw_steering.h"

static const char *TAG = "motors";

#define PWM_FREQ_HZ   1000
#define PWM_RES_BITS  LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY  ((1u << 10) - 1u)

/* Canales: 0 = IN1 (izq. adelante), 1 = IN2 (izq. atras), 2 = IN3 (der. adelante), 3 = IN4 (der. atras). */
enum { CH_LEFT_FWD = 0, CH_LEFT_REV = 1, CH_RIGHT_FWD = 2, CH_RIGHT_REV = 3, CH_COUNT = 4 };

static const int s_pins[CH_COUNT] = {
    CONFIG_WRO_MOTOR_IN1_GPIO, CONFIG_WRO_MOTOR_IN2_GPIO,
    CONFIG_WRO_MOTOR_IN3_GPIO, CONFIG_WRO_MOTOR_IN4_GPIO,
};

static uint32_t s_last_duty[CH_COUNT];   /* ultimo duty escrito a cada salida (diagnostico) */
static bool s_ready;

static void set_duty(int ch, float frac)
{
    if (!s_ready || ch >= 2) return;
    /* Modelo aproximado de tension media. La caida real del L298N varia con la carga. */
    static const float duty_cap = (float)CONFIG_WRO_MOTOR_VMAX_MV / (float)(CONFIG_WRO_BATT_MV - CONFIG_WRO_MOTOR_DROP_MV);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    frac *= (duty_cap > 1.0f ? 1.0f : duty_cap);
    const uint32_t duty = (uint32_t)(frac * (float)PWM_MAX_DUTY + 0.5f);
    s_last_duty[ch] = duty;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

static void drive_wheel(int fwd, int rev, float v_cm_s, bool invert)
{
    if (invert) v_cm_s = -v_cm_s;
    const float frac = fabsf(v_cm_s) / (float)CONFIG_WRO_MOTOR_MAX_CM_S;
    if (v_cm_s >= 0.0f) {
        set_duty(rev, 0.0f);
        set_duty(fwd, frac);
    } else {
        set_duty(fwd, 0.0f);
        set_duty(rev, frac);
    }
}

/* Primera cosa al arrancar: las entradas del L298N a 0 con pull-down. Sin esto, mientras el P4 arranca (y antes
 * de configurar el PWM) los pines quedan flotando y el puente puede mover los motores solo. */
void hw_motors_safe_boot(void)
{
    if (!CONFIG_WRO_ACKERMANN_HW_CONFIGURED) return;
    const int pins[CH_COUNT] = {CONFIG_WRO_MOTOR_IN1_GPIO, CONFIG_WRO_MOTOR_IN2_GPIO,
                                CONFIG_WRO_MOTOR_IN3_GPIO, CONFIG_WRO_MOTOR_IN4_GPIO};
    for (int i = 0; i < 2; ++i) {
        if (pins[i] < 0) continue;
        gpio_reset_pin((gpio_num_t)pins[i]);
        gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)pins[i], GPIO_PULLDOWN_ONLY);
        gpio_set_level((gpio_num_t)pins[i], 0);
    }
}

void hw_motors_init(void)
{
    s_ready = false;
    if (!CONFIG_WRO_ACKERMANN_HW_CONFIGURED || s_pins[0] < 0 || s_pins[1] < 0 || s_pins[0] == s_pins[1]) {
        ESP_LOGW(TAG, "pines de traccion sin configurar: motor deshabilitado");
        return;
    }
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (int ch = 0; ch < 2; ++ch) {
        ledc_channel_config_t c = {
            .gpio_num = s_pins[ch],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)ch,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
    s_ready = true;
    hw_motors_stop();
    ESP_LOGI(TAG, "L298N: PWM %d Hz en GPIO %d/%d, traccion %s", PWM_FREQ_HZ,
             s_pins[0], s_pins[1],
             CONFIG_WRO_MOTORS_ENABLED ? "HABILITADOS" : "deshabilitados (solo calculo)");
}

void hw_motors_wheel_speeds(float speed_cm_s, float yaw_rate_dps, float *left_cm_s, float *right_cm_s)
{
    const float w = yaw_rate_dps * (float)M_PI / 180.0f;               /* rad/s, horario positivo */
    const float half_track_cm = 0.1f * (float)CONFIG_WRO_TRACK_MM * 0.5f;
    *left_cm_s = speed_cm_s + w * half_track_cm;
    *right_cm_s = speed_cm_s - w * half_track_cm;
}

void hw_motors_drive(float speed_cm_s, float yaw_rate_dps)
{
    (void)yaw_rate_dps;
    if (!CONFIG_WRO_MOTORS_ENABLED || !hw_steering_ready() || !isfinite(speed_cm_s)) {
        hw_motors_stop();
        return;
    }
    drive_wheel(CH_LEFT_FWD, CH_LEFT_REV, speed_cm_s, CONFIG_WRO_MOTOR_INVERT_LEFT);
}

void hw_motors_set_wheels(float left_cm_s, float right_cm_s)
{
    /* La prueba antigua de dos motores no es valida para Ackermann. */
    (void)left_cm_s;
    (void)right_cm_s;
    hw_motors_stop();
}

void hw_motors_last_duty(uint32_t duty[4])
{
    for (int i = 0; i < CH_COUNT; ++i) duty[i] = s_last_duty[i];
}

void hw_motors_stop(void)
{
    for (int ch = 0; ch < CH_COUNT; ++ch) set_duty(ch, 0.0f);
}

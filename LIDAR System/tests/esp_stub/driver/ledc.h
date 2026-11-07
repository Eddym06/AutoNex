/* Sustituto del driver LEDC de ESP-IDF: guarda el ultimo duty de cada canal (solo para pruebas en el PC). */
#ifndef LEDC_STUB_H
#define LEDC_STUB_H
#include <stdint.h>
#include "esp_err_stub.h"
typedef enum { LEDC_LOW_SPEED_MODE = 0 } ledc_mode_t;
typedef enum { LEDC_TIMER_0 = 0 } ledc_timer_t;
typedef enum { LEDC_TIMER_10_BIT = 10 } ledc_timer_bit_t;
typedef enum { LEDC_AUTO_CLK = 0 } ledc_clk_cfg_t;
typedef enum { LEDC_INTR_DISABLE = 0 } ledc_intr_type_t;
typedef int ledc_channel_t;
typedef struct { ledc_mode_t speed_mode; ledc_timer_bit_t duty_resolution; ledc_timer_t timer_num;
                 uint32_t freq_hz; ledc_clk_cfg_t clk_cfg; } ledc_timer_config_t;
typedef struct { int gpio_num; ledc_mode_t speed_mode; ledc_channel_t channel; ledc_intr_type_t intr_type;
                 ledc_timer_t timer_sel; uint32_t duty; int hpoint; } ledc_channel_config_t;
extern uint32_t ledc_stub_duty[4];
esp_err_t ledc_timer_config(const ledc_timer_config_t *c);
esp_err_t ledc_channel_config(const ledc_channel_config_t *c);
esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t ch, uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t ch);
#endif

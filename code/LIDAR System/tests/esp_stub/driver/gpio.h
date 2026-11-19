/* Sustituto del driver GPIO de ESP-IDF para pruebas en el PC (las funciones no hacen nada). */
#ifndef GPIO_STUB_H
#define GPIO_STUB_H
#include <stdint.h>
#include "esp_err_stub.h"
typedef enum { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 } gpio_mode_t;
typedef enum { GPIO_PULLDOWN_ONLY = 2 } gpio_pull_mode_t;
typedef int gpio_num_t;
esp_err_t gpio_reset_pin(gpio_num_t pin);
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode);
esp_err_t gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t pull);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
#endif

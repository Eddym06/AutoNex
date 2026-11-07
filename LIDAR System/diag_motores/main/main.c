/* Prueba: IN1..IN4 = 0000 y nada mas. Lee de vuelta el nivel de cada pin para ver si el puente y el cableado
 * coinciden con lo que el P4 manda. */
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int PINS[4] = {1, 36, 32, 34};

void app_main(void)
{
    for (int i = 0; i < 4; ++i) {
        gpio_reset_pin(PINS[i]);
        gpio_set_direction(PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(PINS[i], GPIO_PULLDOWN_ONLY);
        gpio_set_level(PINS[i], 0);
    }
    printf("DIAG: IN1=GPIO%d IN2=GPIO%d IN3=GPIO%d IN4=GPIO%d forzados a 0000\n", PINS[0], PINS[1], PINS[2], PINS[3]);
    for (;;) {
        printf("DIAG: niveles leidos IN1..IN4 = %d %d %d %d\n", gpio_get_level(PINS[0]), gpio_get_level(PINS[1]),
               gpio_get_level(PINS[2]), gpio_get_level(PINS[3]));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

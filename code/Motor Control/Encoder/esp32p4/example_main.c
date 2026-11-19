/*
 * example_main.c - Ejemplo de uso en ESP-IDF (ESP32-P4)
 *
 * 1) Primera vez (o si cambiaste el disco): se calibra con la rueda en el aire.
 * 2) Después: se lee posición y velocidad a 100 Hz.
 *
 * Cambia ENC_GPIO, WHEEL_R y las dos funciones motor_*() por las de tu driver.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "encoder_port.h"

#define ENC_GPIO   GPIO_NUM_4         /* salida del KLH-512 */
#define WHEEL_R    0.0325f            /* m, radio efectivo (ver README: calibrarlo) */

static const char *TAG = "main";

/* --- sustituye por tu driver de motor ---------------------------------- */
static void motor_spin_constant(void) { /* PWM fijo, p.ej. 60 %, rueda en el aire */ }
static void motor_stop(void)          { /* PWM = 0 */ }
/* ------------------------------------------------------------------------ */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    encoder_port_t *enc = NULL;
    encoder_port_config_t cfg = {
        .gpio = ENC_GPIO, .wheel_radius_m = WHEEL_R, .nvs_key = "enc_L",
        .harm_cut = 3, .pull_up = true,
    };
    ESP_ERROR_CHECK(encoder_port_init(&enc, &cfg));

    if (!encoder_port_has_table(enc)) {
        ESP_LOGW(TAG, "Calibrando: rueda EN EL AIRE, motor a velocidad constante");
        motor_spin_constant();
        vTaskDelay(pdMS_TO_TICKS(1500));                 /* que arranque y se estabilice */
        ESP_ERROR_CHECK(encoder_port_calibrate_start(enc));
        enc_cal_report_t rep;
        while (!encoder_port_calibrate_poll(enc, &rep)) vTaskDelay(pdMS_TO_TICKS(200));
        motor_stop();
        ESP_LOGI(TAG, "calibracion %s; armonicos 1..4 del error de tabla [grados]: %.2f %.2f %.2f %.2f",
                 rep.ok ? "OK" : "FALLIDA", rep.harm_deg[0], rep.harm_deg[1], rep.harm_deg[2], rep.harm_deg[3]);
        /* Si el armónico 1 es mucho mayor que el resto (decae como 1/h) es rizado del motor
         * y/o excentricidad: el filtro lo elimina. Ver README. */
    }

    encoder_port_zero(enc);
    encoder_port_set_direction(enc, +1);                 /* tu control debe actualizarlo */

    for (;;) {
        enc_est_t e;
        encoder_port_get(enc, &e);
        printf("pos=%8.2f mm  [%8.2f .. %8.2f]  v=%6.3f m/s  a=%6.2f m/s2  flancos=%ld  alineado=%d\n",
               e.pos_m * 1000.0f, e.pos_lo_m * 1000.0f, e.pos_hi_m * 1000.0f,
               e.vel_mps, e.acc_mps2, (long)e.edges, encoder_port_is_aligned(enc));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

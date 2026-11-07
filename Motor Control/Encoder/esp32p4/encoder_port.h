/*
 * encoder_port.h - Integración del estimador con ESP-IDF (ESP32-P4)
 *
 * Un encoder_port_t = un sensor ranurado (un GPIO) = una rueda.
 * Para dos ruedas crea dos instancias con GPIOs y claves NVS distintas.
 */
#ifndef ENCODER_PORT_H
#define ENCODER_PORT_H

#include "encoder_core.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct encoder_port encoder_port_t;

typedef struct {
    gpio_num_t  gpio;            /* salida del sensor KLH-512 (nivel tras cada flanco) */
    float       wheel_radius_m;  /* radio efectivo de la rueda                          */
    const char *nvs_key;         /* clave NVS de la tabla (<= 15 caracteres), ej. "enc_L" */
    int         harm_cut;        /* armónicos a filtrar de la tabla (recomendado: 3)    */
    bool        pull_up;         /* activar pull-up interno                             */
} encoder_port_config_t;

/* Crea la instancia, instala la ISR y carga la tabla de NVS si existe.
 * Requiere nvs_flash_init() hecho antes. */
esp_err_t encoder_port_init(encoder_port_t **out, const encoder_port_config_t *cfg);

/* Dirección real de giro: +1 adelante, -1 reversa. Con un solo sensor el
 * encoder no la detecta: llámala desde el control del motor. */
void encoder_port_set_direction(encoder_port_t *p, int dir);

/* Lee posición/velocidad en este instante (segura desde cualquier tarea).
 * Llamarla a 1 kHz o a la frecuencia del lazo de control. */
void encoder_port_get(encoder_port_t *p, enc_est_t *out);

/* Pone la posición a cero en el siguiente flanco. */
void encoder_port_zero(encoder_port_t *p);

/* Calibración: con la rueda en el aire y el motor girando a velocidad
 * CONSTANTE (PWM fijo). Llama a start, espera ~24 vueltas y consulta poll. */
esp_err_t encoder_port_calibrate_start(encoder_port_t *p);
/* true cuando terminó (aunque haya fallado: mira rep->ok). Si ok, la tabla
 * queda activa y guardada en NVS. */
bool encoder_port_calibrate_poll(encoder_port_t *p, enc_cal_report_t *rep);

bool encoder_port_has_table(const encoder_port_t *p);
bool encoder_port_is_aligned(const encoder_port_t *p);

/* Estadísticas de diagnóstico. */
void encoder_port_stats(encoder_port_t *p, uint32_t *accepted, uint32_t *glitches, uint32_t *missed);

#ifdef __cplusplus
}
#endif
#endif /* ENCODER_PORT_H */

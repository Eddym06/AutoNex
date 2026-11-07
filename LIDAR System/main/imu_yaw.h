#ifndef IMU_YAW_H
#define IMU_YAW_H

#include <stdbool.h>
#include <stdint.h>

/* Rumbo por integracion del giroscopio.
 *
 * El carrito no tiene odometria, asi que el rumbo es lo unico que lo ata al
 * mundo. Una brujula (magnetometro) cerca de los motores y de un chasis con
 * tornillos de acero cambia decenas de grados con la corriente; el giroscopio
 * no. Su unico defecto es una deriva lenta que se mide y se resta:
 *   1. con el carrito QUIETO se promedia la lectura (sesgo), antes de arrancar;
 *   2. la integracion resta ese sesgo;
 *   3. el controlador reancla el rumbo contra las paredes del carril en cada
 *      recta, que es una referencia absoluta gratuita.
 * El magnetometro es opcional (ganancia 0 por defecto). Convenio: grados, positivo
 * en sentido horario (como una brujula); el adaptador del MPU convierte el signo
 * de su eje Z si hace falta. */
typedef struct {
    float yaw_deg;          /* rumbo estimado, [-180, 180) */
    float rate_dps;         /* ultima velocidad angular, sin sesgo */
    float bias_dps;         /* sesgo medido con el carrito quieto */
    float mag_gain;         /* 0 = ignorar el magnetometro; ~0.002 si es fiable */
    float max_rate_dps;     /* descarta lecturas imposibles (saturacion, ruido) */
    uint64_t last_us;
    bool has_last;
    bool calibrating;
    double bias_sum;
    uint32_t bias_count;
    bool bias_valid;
} imu_yaw_t;

void imu_yaw_init(imu_yaw_t *y);
/* Con el carrito quieto: acumula lecturas para estimar el sesgo. */
void imu_yaw_begin_calibration(imu_yaw_t *y);
/* Fija el sesgo con lo acumulado (necesita al menos `min_samples` lecturas).
 * Devuelve false si no hubo suficientes. */
bool imu_yaw_end_calibration(imu_yaw_t *y, uint32_t min_samples);
/* gyro_dps: horario positivo. mag_heading_deg: NAN si no hay. */
void imu_yaw_update(imu_yaw_t *y, uint64_t t_us, float gyro_dps, float mag_heading_deg);
/* Corrige el rumbo hacia una referencia absoluta (p. ej. la direccion de las
 * paredes): yaw += gain * (measured - yaw). */
void imu_yaw_correct(imu_yaw_t *y, float measured_deg, float gain);
void imu_yaw_set(imu_yaw_t *y, float yaw_deg);
float imu_wrap_deg(float deg);

#endif

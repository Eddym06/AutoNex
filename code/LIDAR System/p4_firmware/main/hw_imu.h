#ifndef HW_IMU_H
#define HW_IMU_H

#include <stdbool.h>
#include <stdint.h>

/* MPU-6050 (o clon MPU-6500, WHO_AM_I 0x70) por I2C; INT no se usa: se consulta por tiempo. Acelerometro y giroscopio en una lectura. */
void hw_imu_init(void);

/* Diagnostico: muestras leidas y la ultima velocidad angular (grados/s). */
void hw_imu_stats(uint32_t *samples, float *last_dps, uint32_t *errors);

/* Firma de p4_hal_t.poll_gyro: grados/s, horario positivo. false si no hay muestra nueva. */
bool hw_imu_poll_gyro(void *ctx, float *dps, uint64_t *t_us);

/* Ultimo acelerometro leido (g, en el marco del IMU). false si aun no hay. */
bool hw_imu_accel(float a_g[3]);

/* Signo del giroscopio (+1/-1): por defecto el de menuconfig; el calibrado se guarda en NVS. */
int hw_imu_sign(void);
void hw_imu_set_sign(int sign);

#endif

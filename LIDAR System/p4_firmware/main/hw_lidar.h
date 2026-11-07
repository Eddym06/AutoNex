#ifndef HW_LIDAR_H
#define HW_LIDAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "local_obstacle_map.h"

/* WitMotion D6 (COIN-D6) por UART, 230400 8N1. Protocolo: LiDAR System/docs/WITMOTION_D6.md.
 * Junta las muestras de una vuelta completa y la entrega al bucle con hw_lidar_poll_scan. */
void hw_lidar_init(void);

/* Firma de p4_hal_t.poll_scan. true cuando hay una vuelta nueva. */
bool hw_lidar_poll_scan(void *ctx, local_lidar_point_t *points, size_t capacity, size_t *count, uint64_t *t_us);

/* Calibracion del angulo (se guarda en NVS): angulo del robot = sentido * angulo del LiDAR + desfase. */
void hw_lidar_calibration_get(float *sign, float *offset_deg);
void hw_lidar_calibration_set(float sign, float offset_deg);

/* Diagnostico: bytes crudos recibidos y los primeros 8 (aunque no formen paquetes validos). */
void hw_lidar_raw(uint32_t *bytes, uint8_t head[8]);

/* Diagnostico. */
void hw_lidar_stats(uint32_t *packets_ok, uint32_t *checksum_errors, uint32_t *revolutions, uint32_t *points,
                    uint32_t *points_dropped);

#endif

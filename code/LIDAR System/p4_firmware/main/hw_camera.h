#ifndef HW_CAMERA_H
#define HW_CAMERA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Enlace con el ESP32-S3 CAM: el S3 va por su puente CH340 a un puerto USB del P4 (host USB).
 * Los bytes recibidos se guardan en un buffer y los lee camera_table_receiver. */
void hw_camera_init(void);

/* Firma de p4_hal_t.read_camera: bytes recibidos desde la ultima llamada (sin esperar). */
size_t hw_camera_read(void *ctx, uint8_t *buffer, size_t capacity);

/* Envia bytes al S3 (p. ej. comandos de configuracion de la camara). false si no hay S3. */
bool hw_camera_write(const uint8_t *data, size_t len);

/* Diagnostico: bytes recibidos, bytes perdidos por buffer lleno y si hay un S3 conectado. */
void hw_camera_stats(uint32_t *bytes, uint32_t *lost, bool *connected);

#endif

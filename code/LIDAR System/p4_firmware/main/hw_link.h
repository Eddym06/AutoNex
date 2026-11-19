#ifndef HW_LINK_H
#define HW_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "link_protocol.h"

/* Enlace de control con el S3 (shared/link_protocol.h) sobre la UART de hw_camera. */

/* Firma de p4_hal_t.read_camera: entrega los bytes de la camara al decodificador de tablas y, de paso,
 * extrae las ordenes del S3 (iniciar/parar/latido). */
size_t hw_link_read(void *ctx, uint8_t *buffer, size_t capacity);

/* Ultima orden recibida (LINK_CMD_*), una sola vez: devuelve LINK_CMD_NONE si no hay nueva. */
uint8_t hw_link_take_cmd(void);

/* Microsegundos del ultimo latido con la pagina web viva (0 = nunca). */
int64_t hw_link_last_alive_us(void);

/* Envia el estado al S3 (bloquea unos ms: llamar desde una tarea de baja prioridad). */
bool hw_link_send_state(const link_state_t *state);

#endif

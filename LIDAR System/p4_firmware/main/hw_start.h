#ifndef HW_START_H
#define HW_START_H

#include <stdbool.h>

/* Boton de arranque (GPIO de WRO_START_GPIO, pull-up interno, activo en bajo). */
void hw_start_init(void);

/* Firma de p4_hal_t.start_pressed: true una sola vez por pulsacion. */
bool hw_start_pressed(void *ctx);

#endif

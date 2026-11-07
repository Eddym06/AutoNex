#ifndef HW_ENCODERS_H
#define HW_ENCODERS_H

#include <stdint.h>

#include "wro_controller.h"

/* Encoder A/B del eje motriz comun, lectura en cuadratura x4. */
void hw_encoders_init(void);
bool hw_encoders_ready(void);

/* Compatibilidad: el sentido lo determina la cuadratura. */
void hw_encoders_set_direction(int left_dir, int right_dir);

/* Entrega el angulo acumulado del eje al controlador cuando esta calibrado. */
void hw_encoders_feed(wro_controller_t *c, uint64_t t_us);

/* Diagnostico: posicion con signo de cada rueda (metros). */
void hw_encoders_position_m(float *left_m, float *right_m);

#endif

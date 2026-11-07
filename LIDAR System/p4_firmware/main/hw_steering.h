#ifndef HW_STEERING_H
#define HW_STEERING_H

#include <stdbool.h>

/* Angulo de direccion efectivo: derecha positivo. Limites y pulsos medidos. */
bool hw_steering_init(void);
bool hw_steering_ready(void);
void hw_steering_set(float degrees);

#endif

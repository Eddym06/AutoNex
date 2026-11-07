#ifndef HW_MOTORS_H
#define HW_MOTORS_H

#include <stdbool.h>
#include <stdint.h>

/* Un motor trasero por IN1/IN2 del L298N, PWM a 1 kHz; ENA puenteado. */
/* Llamar antes que nada: pone las entradas del puente a 0 mientras arranca el resto del firmware. */
void hw_motors_safe_boot(void);
void hw_motors_init(void);

/* Velocidad del eje en cm/s (negativa = atras); la direccion la controla el servo. */
void hw_motors_drive(float speed_cm_s, float yaw_rate_dps);

/* Velocidad de cada rueda que resulta del comando (para el sentido de los encoders). */
void hw_motors_wheel_speeds(float speed_cm_s, float yaw_rate_dps, float *left_cm_s, float *right_cm_s);

/* Interfaz antigua de prueba: detiene el motor en el chasis nuevo. */
void hw_motors_set_wheels(float left_cm_s, float right_cm_s);

/* Diagnostico: ultimo duty (0..1023) de IN1..IN4. */
void hw_motors_last_duty(uint32_t duty[4]);

void hw_motors_stop(void);

#endif

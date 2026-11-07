#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* Prueba de motores desde la pagina web, sin iniciar la mision. Solo con el robot parado y sin armar.
 *   M1 = rueda izquierda, M2 = rueda derecha. Cada pulsacion enciende su motor hacia delante durante 1 s.
 *   SECUENCIA: M1 durante 1 s, luego M2 durante 1 s. Cada motor se apaga al terminar su tiempo. */
#define MOTOR_TEST_PULSE_US 1000000u
#define MOTOR_TEST_CM_S 15.0f          /* velocidad de la rueda durante la prueba (lenta, a proposito) */

enum { MOTOR_TEST_REQ_M1 = 1, MOTOR_TEST_REQ_M2 = 2, MOTOR_TEST_REQ_SEQ = 3, MOTOR_TEST_REQ_STOP = 4 };
#define MOTOR_TEST_BIT_M1 0x01u
#define MOTOR_TEST_BIT_M2 0x02u

typedef struct {
    uint8_t mask;                  /* MOTOR_TEST_BIT_*: motores encendidos ahora */
    uint8_t seq_step;              /* 0 ninguna, 1 secuencia en M1, 2 secuencia en M2 */
    uint64_t until_us;             /* hasta cuando dura el paso actual */
} motor_test_t;

void motor_test_request(motor_test_t *t, int request, uint64_t now_us);
/* Apaga lo que ya termino su tiempo y avanza la secuencia. Llamar en cada vuelta del bucle. */
void motor_test_update(motor_test_t *t, uint64_t now_us);
/* Velocidad de cada rueda que pide la prueba (cm/s, 0 si el motor esta apagado). */
void motor_test_wheels(const motor_test_t *t, float *left_cm_s, float *right_cm_s);

#endif

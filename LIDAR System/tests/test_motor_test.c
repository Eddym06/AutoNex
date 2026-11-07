/* Logica de la prueba de motores: pulsacion, secuencia M1 -> M2 y parada. */
#include <assert.h>
#include <stdio.h>
#include "motor_test.h"

int main(void)
{
    motor_test_t t = {0};
    float l, r;

    /* Pulsar M1: solo la izquierda durante 1 s. */
    motor_test_request(&t, MOTOR_TEST_REQ_M1, 0);
    motor_test_wheels(&t, &l, &r);
    assert(t.mask == MOTOR_TEST_BIT_M1 && l > 0.0f && r == 0.0f);
    motor_test_update(&t, 999999u);
    assert(t.mask == MOTOR_TEST_BIT_M1);
    motor_test_update(&t, 1000000u);
    assert(t.mask == 0);
    motor_test_wheels(&t, &l, &r);
    assert(l == 0.0f && r == 0.0f);

    /* Pulsar M2: solo la derecha. */
    motor_test_request(&t, MOTOR_TEST_REQ_M2, 5000000u);
    motor_test_wheels(&t, &l, &r);
    assert(t.mask == MOTOR_TEST_BIT_M2 && l == 0.0f && r > 0.0f);
    motor_test_update(&t, 6000000u);
    assert(t.mask == 0);

    /* Secuencia: M1 1 s, luego M2 1 s, luego nada. Nunca los dos a la vez. */
    motor_test_request(&t, MOTOR_TEST_REQ_SEQ, 10000000u);
    assert(t.mask == MOTOR_TEST_BIT_M1);
    motor_test_update(&t, 10500000u);
    assert(t.mask == MOTOR_TEST_BIT_M1);
    motor_test_update(&t, 11000000u);
    assert(t.mask == MOTOR_TEST_BIT_M2);
    motor_test_wheels(&t, &l, &r);
    assert(l == 0.0f && r > 0.0f);
    motor_test_update(&t, 11500000u);
    assert(t.mask == MOTOR_TEST_BIT_M2);
    motor_test_update(&t, 12000000u);
    assert(t.mask == 0);

    /* Parar a mitad de un pulso apaga al instante. */
    motor_test_request(&t, MOTOR_TEST_REQ_SEQ, 20000000u);
    motor_test_request(&t, MOTOR_TEST_REQ_STOP, 20200000u);
    assert(t.mask == 0 && t.seq_step == 0);
    motor_test_update(&t, 21000000u);
    assert(t.mask == 0);

    puts("PASS: prueba de motores (M1, M2, secuencia, parada)");
    return 0;
}

#include "motor_test.h"

void motor_test_request(motor_test_t *t, int request, uint64_t now_us)
{
    switch (request) {
    case MOTOR_TEST_REQ_M1:
        t->mask = MOTOR_TEST_BIT_M1; t->seq_step = 0; t->until_us = now_us + MOTOR_TEST_PULSE_US;
        break;
    case MOTOR_TEST_REQ_M2:
        t->mask = MOTOR_TEST_BIT_M2; t->seq_step = 0; t->until_us = now_us + MOTOR_TEST_PULSE_US;
        break;
    case MOTOR_TEST_REQ_SEQ:
        t->mask = MOTOR_TEST_BIT_M1; t->seq_step = 1; t->until_us = now_us + MOTOR_TEST_PULSE_US;
        break;
    default:                       /* parar */
        t->mask = 0; t->seq_step = 0; t->until_us = 0;
        break;
    }
}

void motor_test_update(motor_test_t *t, uint64_t now_us)
{
    if (!t->mask || now_us < t->until_us) return;
    if (t->seq_step == 1) {        /* M1 termino: se apaga y empieza M2 */
        t->mask = MOTOR_TEST_BIT_M2; t->seq_step = 2; t->until_us = now_us + MOTOR_TEST_PULSE_US;
    } else {
        t->mask = 0; t->seq_step = 0; t->until_us = 0;
    }
}

void motor_test_wheels(const motor_test_t *t, float *left_cm_s, float *right_cm_s)
{
    *left_cm_s = (t->mask & MOTOR_TEST_BIT_M1) ? MOTOR_TEST_CM_S : 0.0f;
    *right_cm_s = (t->mask & MOTOR_TEST_BIT_M2) ? MOTOR_TEST_CM_S : 0.0f;
}

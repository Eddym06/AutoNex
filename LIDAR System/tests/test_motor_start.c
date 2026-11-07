/* Simula el arranque del carrito con el codigo real del P4: p4_runtime + wro_controller + hw_motors (el driver
 * LEDC esta sustituido por un registro de duty). Comprueba que los motores no se mueven hasta el boton, que al
 * pulsarlo las ruedas reciben PWM con el sentido correcto y que el tope de duty protege el motor de 12 V. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hw_motors.h"
#include "p4_runtime.h"
#include "wro_config.h"

extern uint32_t ledc_stub_duty[4];
static bool steering_ready = true;
bool hw_steering_ready(void) { return steering_ready; }

#define FULL_DUTY_LIMIT ((12000.0f / (16800.0f - 2000.0f)) * 1023.0f)   /* tope real con 16,8 V y 2 V de caida */

typedef struct {
    uint64_t now;
    bool button, armed;
    bool scan_ready, gyro_ready;
    float last_v, last_w;
    int drive_calls, moving_calls;
    uint32_t max_duty;
} sim_t;

static uint64_t s_now(void *c) { return ((sim_t *)c)->now; }
static bool s_start(void *c)
{
    sim_t *s = c;
    if (!s->button) return false;
    s->button = false;
    s->armed = true;                       /* lo mismo que hal_start_pressed en app_main.c */
    return true;
}
static bool s_gyro(void *c, float *dps, uint64_t *t)
{
    sim_t *s = c;
    if (!s->gyro_ready) return false;
    s->gyro_ready = false;
    *dps = 0.0f; *t = s->now;
    return true;
}
static bool s_scan(void *c, local_lidar_point_t *p, size_t cap, size_t *count, uint64_t *t)
{
    sim_t *s = c;
    if (!s->scan_ready) return false;
    s->scan_ready = false;
    size_t n = 0;                          /* pasillo recto de 100 cm, centrado */
    for (int deg = -180; deg < 180 && n < cap; ++deg) {
        const float a = (float)deg * 0.0174532925f;
        float d = 400.0f;
        if (fabsf(sinf(a)) > 0.1f) d = fminf(d, 50.0f / fabsf(sinf(a)));
        if (cosf(a) > 0.1f) d = fminf(d, 300.0f / cosf(a));
        p[n++] = (local_lidar_point_t){.angle_deg = (float)deg, .distance_cm = d, .valid = true};
    }
    *count = n; *t = s->now;
    return true;
}
static size_t s_cam(void *c, uint8_t *buf, size_t cap) { (void)c; (void)buf; (void)cap; return 0; }

/* Igual que drive() en app_main.c: sin armar, la orden es cero; luego al driver de motores. */
static void s_drive(void *c, float steer, float speed, float yaw)
{
    sim_t *s = c;
    (void)steer;
    if (!s->armed) speed = yaw = 0.0f;
    s->last_v = speed; s->last_w = yaw;
    ++s->drive_calls;
    if (fabsf(speed) > 0.1f || fabsf(yaw) > 0.1f) ++s->moving_calls;
    hw_motors_drive(speed, yaw);
    for (int c2 = 0; c2 < 4; ++c2) if (ledc_stub_duty[c2] > s->max_duty) s->max_duty = ledc_stub_duty[c2];
}

static void tick(p4_runtime_t *rt, sim_t *s, int n, bool scan_every_10)
{
    for (int i = 0; i < n; ++i) {
        s->now += 10000;
        s->gyro_ready = true;
        if (scan_every_10 && i % 10 == 9) s->scan_ready = true;
        p4_runtime_step(rt);
    }
}

static void duties(uint32_t d[4]) { for (int i = 0; i < 4; ++i) d[i] = ledc_stub_duty[i]; }

int main(void)
{
    sim_t s = {0};
    p4_hal_t hal = {.ctx = &s, .now_us = s_now, .start_pressed = s_start, .poll_scan = s_scan,
                    .poll_gyro = s_gyro, .read_camera = s_cam, .drive = s_drive};
    wro_controller_config_t cfg;
    wro_controller_config_default(&cfg);
    static p4_runtime_t rt;
    hw_motors_init();
    p4_runtime_init(&rt, &hal, &cfg);

    /* 1) Encendido: un segundo de sensores, sin boton. Ninguna rueda recibe PWM. */
    tick(&rt, &s, 100, true);
    uint32_t d[4];
    duties(d);
    assert(!s.armed && rt.controller.phase == WRO_PHASE_IDLE);
    assert(d[0] == 0 && d[1] == 0 && d[2] == 0 && d[3] == 0);
    assert(s.moving_calls == 0);
    printf("ok: encendido sin boton: PWM 0 en las 4 salidas durante 1 s\n");

    /* 2) Boton: el carrito arranca y avanza. Las dos ruedas giran hacia delante, sin tope excedido. */
    s.button = true;
    tick(&rt, &s, 40, true);
    duties(d);
    assert(s.armed && rt.controller.phase == WRO_PHASE_RUN);
    assert(s.last_v > 10.0f);
    assert(d[0] > 0 && d[1] == 0 && d[2] == 0 && d[3] == 0);
    assert(s.max_duty <= (uint32_t)FULL_DUTY_LIMIT + 1u);
    printf("ok: boton: avanza a %.1f cm/s; duty izq %u, der %u (tope %.0f de 1023)\n",
           s.last_v, d[0], d[2], FULL_DUTY_LIMIT);

    /* 3) Proporcion: duty = velocidad/40 x tope. */
    const float expect = fminf(1.0f, s.last_v / 40.0f) * FULL_DUTY_LIMIT;
    assert(fabsf((float)d[0] - expect) < 3.0f);
    printf("ok: duty esperado %.0f, medido %u\n", expect, d[0]);

    /* 4) Sentido: marcha atras y giro horario (izquierda adelante, derecha atras). */
    hw_motors_drive(-20.0f, 0.0f);
    duties(d);
    assert(d[0] == 0 && d[2] == 0 && d[1] > 0 && d[3] == 0);
    hw_motors_drive(0.0f, 90.0f);
    duties(d);
    assert(d[0] == 0 && d[1] == 0 && d[2] == 0 && d[3] == 0);
    steering_ready = false;
    hw_motors_drive(20.0f, 0.0f);
    duties(d);
    assert(d[0] == 0 && d[1] == 0);
    steering_ready = true;
    printf("ok: reversa y giro horario (izq %u / der-atras %u)\n", d[0], d[3]);

    /* 5) Sin LiDAR (cable suelto): para. */
    tick(&rt, &s, 40, false);
    duties(d);
    assert(s.last_v == 0.0f && d[0] == 0 && d[2] == 0);
    printf("ok: sin escaneos: paro (PWM 0)\n");

    /* 6) Parada de la pagina/boton: vuelve a espera y no se mueve. */
    hw_motors_stop();
    duties(d);
    assert(d[0] == 0 && d[1] == 0 && d[2] == 0 && d[3] == 0);
    puts("PASS: arranque del carrito (encendido sin boton, boton → avanza, sentido, giro, paro sin LiDAR)");
    return 0;
}

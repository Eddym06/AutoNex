/* El bucle del P4 con un hardware de mentira: cableado, arranque y paro de seguridad. */
#include "p4_runtime.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint64_t now;
    bool button;
    bool scan_ready;
    bool gyro_ready;
    float steer, speed;
    int drive_calls;
    uint8_t camera[128];
    size_t camera_len;
} fake_t;

static uint64_t f_now(void *c) { return ((fake_t *)c)->now; }
static bool f_start(void *c) { fake_t *f = c; const bool b = f->button; f->button = false; return b; }
static bool f_scan(void *c, local_lidar_point_t *p, size_t cap, size_t *count, uint64_t *t)
{
    fake_t *f = c;
    if (!f->scan_ready) return false;
    f->scan_ready = false;
    /* Pasillo recto de 100 cm con el robot centrado. */
    size_t n = 0;
    for (int deg = -180; deg < 180 && n < cap; deg += 1) {
        const float a = (float)deg * 0.0174532925f;
        float d = 400.0f;
        if (fabsf(sinf(a)) > 0.1f) d = fminf(d, 50.0f / fabsf(sinf(a)));
        if (cosf(a) > 0.1f) d = fminf(d, 300.0f / cosf(a));
        p[n++] = (local_lidar_point_t){.angle_deg = (float)deg, .distance_cm = d, .valid = true};
    }
    *count = n; *t = f->now;
    return true;
}
static bool f_gyro(void *c, float *dps, uint64_t *t)
{
    fake_t *f = c;
    if (!f->gyro_ready) return false;
    f->gyro_ready = false;
    *dps = 0.0f; *t = f->now;
    return true;
}
static size_t f_cam(void *c, uint8_t *buf, size_t cap)
{
    fake_t *f = c;
    const size_t n = f->camera_len < cap ? f->camera_len : cap;
    memcpy(buf, f->camera, n);
    f->camera_len = 0;
    return n;
}
static void f_drive(void *c, float steer, float speed, float yaw_rate)
{
    fake_t *f = c;
    (void)yaw_rate;
    f->steer = steer; f->speed = speed; ++f->drive_calls;
}

int main(void)
{
    fake_t fake = {0};
    p4_hal_t hal = {.ctx = &fake, .now_us = f_now, .start_pressed = f_start, .poll_scan = f_scan,
                    .poll_gyro = f_gyro, .read_camera = f_cam, .drive = f_drive};
    wro_controller_config_t cfg;
    wro_controller_config_default(&cfg);
    static p4_runtime_t rt;
    p4_runtime_init(&rt, &hal, &cfg);
    assert(fake.drive_calls == 1 && fake.speed == 0.0f);   /* arranca parado */

    /* Un segundo de giroscopio quieto: calibra el sesgo. Antes del boton no se mueve. */
    for (int i = 0; i < 100; ++i) {
        fake.now += 10000; fake.gyro_ready = true;
        if (i % 10 == 9) fake.scan_ready = true;
        p4_runtime_step(&rt);
    }
    assert(fake.speed == 0.0f && rt.controller.phase == WRO_PHASE_IDLE && rt.gyro_samples == 100);

    /* Boton: el carrito arranca y avanza recto por el pasillo. */
    fake.button = true;
    for (int i = 0; i < 30; ++i) {
        fake.now += 10000; fake.gyro_ready = true;
        if (i % 10 == 9) fake.scan_ready = true;
        p4_runtime_step(&rt);
    }
    assert(rt.controller.phase == WRO_PHASE_RUN);
    assert(fake.speed > 10.0f && fabsf(fake.steer) < 6.0f);

    /* Una trama del S3 corrupta no entra; una buena si. */
    wro_sensor_table_t table = {.valid = true, .action = 1, .object_count = 1,
        .objects = {{.color = 1, .center_x_px = 80, .width_px = 12, .distance_mm = 400}}};
    fake.camera_len = wro_sensor_table_encode(&table, fake.camera, sizeof(fake.camera));
    fake.camera[8] ^= 0x10;
    fake.now += 10000; p4_runtime_step(&rt);
    assert(rt.tables_ok == 0);
    fake.camera_len = wro_sensor_table_encode(&table, fake.camera, sizeof(fake.camera));
    fake.now += 10000; p4_runtime_step(&rt);
    assert(rt.tables_ok == 1);

    /* Sin escaneos durante 400 ms (cable del D6 suelto): paro de seguridad. */
    const int calls = fake.drive_calls;
    for (int i = 0; i < 40; ++i) { fake.now += 10000; fake.gyro_ready = true; p4_runtime_step(&rt); }
    assert(fake.speed == 0.0f && fake.drive_calls > calls);
    /* Vuelven los barridos, pero el giroscopio se queda sin muestras. */
    for (int i = 0; i < 40; ++i) {
        fake.now += 10000;
        if (i % 10 == 9) fake.scan_ready = true;
        p4_runtime_step(&rt);
        if (i >= 30) assert(fake.speed == 0.0f);
    }
    assert(fake.speed == 0.0f);
    /* Una muestra nueva permite reanudar con el siguiente barrido. */
    fake.now += 10000; fake.gyro_ready = true; fake.scan_ready = true;
    p4_runtime_step(&rt);
    assert(fake.speed > 0.0f);
    puts("PASS: bucle del P4 (arranca parado, espera el boton, avanza, descarta tramas corruptas, para sin LiDAR)");
    return 0;
}

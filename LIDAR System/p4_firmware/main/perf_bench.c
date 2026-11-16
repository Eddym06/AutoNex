/* Banco de rendimiento del P4 (CONFIG_WRO_PERF_BENCH): ejecuta el MISMO C del robot con datos sinteticos y manda los tiempos por la consola USB.
 * No mueve ningun motor (el puente queda con ENB bajo desde el arranque) ni toca el LiDAR, la IMU ni la camara. Cada linea de resultado empieza por "PERF ". */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "it_plan.h"
#include "p4_runtime.h"
#include "park_ackermann.h"
#include "perf_bench.h"

#define US() ((uint32_t)esp_timer_get_time())

static EXT_RAM_BSS_ATTR p4_runtime_t s_rt;
static it_plan_t s_plan, s_work;

/* ------------------------------------------------------------------ escena sintetica para el controlador */
static float ray_box_exit(float ox, float oz, float dx, float dz, float half)
{
    float t = 1.0e9f;
    if (dx > 1.0e-6f) t = fminf(t, (half - ox) / dx); else if (dx < -1.0e-6f) t = fminf(t, (-half - ox) / dx);
    if (dz > 1.0e-6f) t = fminf(t, (half - oz) / dz); else if (dz < -1.0e-6f) t = fminf(t, (-half - oz) / dz);
    return t;
}
static float ray_box_enter(float ox, float oz, float dx, float dz, float half)
{
    float t0 = -1.0e9f, t1 = 1.0e9f;
    const float o[2] = {ox, oz}, d[2] = {dx, dz};
    for (int k = 0; k < 2; ++k) {
        if (fabsf(d[k]) < 1.0e-6f) { if (fabsf(o[k]) > half) return 1.0e9f; continue; }
        float a = (-half - o[k]) / d[k], b = (half - o[k]) / d[k];
        if (a > b) { const float s = a; a = b; b = s; }
        t0 = fmaxf(t0, a); t1 = fminf(t1, b);
    }
    return (t0 <= t1 && t1 > 0.0f) ? (t0 > 0.0f ? t0 : 1.0e9f) : 1.0e9f;
}
static float ray_circle(float ox, float oz, float dx, float dz, float cx, float cz, float r)
{
    const float px = cx - ox, pz = cz - oz, b = px * dx + pz * dz, c = px * px + pz * pz - r * r, disc = b * b - c;
    if (disc < 0.0f) return 1.0e9f;
    const float t = b - sqrtf(disc);
    return t > 0.0f ? t : 1.0e9f;
}
static const float k_signs[5][2] = {{120, 30}, {10, 100}, {-80, -30}, {80, -10}, {-30, 120}};

static size_t synth_scan(float x, float z, float yaw_deg, local_lidar_point_t *pts, size_t cap)
{
    const size_t n = cap < 400 ? cap : 400;
    for (size_t i = 0; i < n; ++i) {
        const float a = 360.0f * (float)i / (float)n;                 /* derecha positiva desde el frente */
        const float h = (yaw_deg + a) * 0.017453292f, dx = sinf(h), dz = cosf(h);
        float t = ray_box_exit(x, z, dx, dz, 150.0f);
        t = fminf(t, ray_box_enter(x, z, dx, dz, 50.0f));
        for (int j = 0; j < 5; ++j) t = fminf(t, ray_circle(x, z, dx, dz, k_signs[j][0], k_signs[j][1], 3.0f));
        pts[i].angle_deg = a > 180.0f ? a - 360.0f : a;
        pts[i].distance_cm = t;
        pts[i].intensity = 50.0f;
        pts[i].valid = t > 3.0f && t < 400.0f;
        pts[i].blind = false;
    }
    return n;
}
static struct { uint64_t t_us, last_gyro_us, last_scan_us; size_t calls; } s_fake;
static uint64_t f_now(void *c) { (void)c; return s_fake.t_us; }
static bool f_start(void *c) { (void)c; return true; }
static bool f_scan(void *c, local_lidar_point_t *p, size_t cap, size_t *count, uint64_t *t_us)
{
    (void)c;
    if (s_fake.t_us - s_fake.last_scan_us < 100000u) return false;      /* una vuelta del LiDAR cada 100 ms virtuales */
    s_fake.last_scan_us = s_fake.t_us;
    *count = synth_scan(0.0f, -100.0f, 90.0f, p, cap);   /* en la recta de abajo, mirando al este (ccw) */
    *t_us = s_fake.t_us;
    return true;
}
static bool f_gyro(void *c, float *dps, uint64_t *t_us)
{
    (void)c;
    if (s_fake.t_us == s_fake.last_gyro_us) return false;                /* una muestra por paso (el bucle de p4_runtime lee hasta que no haya) */
    s_fake.last_gyro_us = s_fake.t_us;
    *dps = 0.0f; *t_us = s_fake.t_us;
    return true;
}
static size_t f_cam(void *c, uint8_t *b, size_t cap) { (void)c; (void)b; (void)cap; return 0; }
static void f_drive(void *c, float st, float sp, float yr) { (void)c; (void)st; (void)sp; (void)yr; }

static void sort_u32(uint32_t *a, int n)
{
    for (int i = 1; i < n; ++i) { const uint32_t v = a[i]; int j = i - 1; while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; --j; } a[j + 1] = v; }
}

static void bench_it(void)
{
    it_obstacle_t obs[5];
    for (int i = 0; i < 5; ++i) { obs[i].x_cm = k_signs[i][0]; obs[i].z_cm = k_signs[i][1]; obs[i].color = (i == 2) ? 2 : 1; }
    it_config_t cfg;
    it_config_default(&cfg);
    cfg.grip = 0.5f; cfg.v_max_cm_s = 188.7f; cfg.a_motor_cm_s2 = 300.0f;
    uint32_t t0 = US();
    const bool ok = it_plan_build(&s_plan, &cfg, -1, obs, 5);
    const uint32_t dt = US() - t0;
    printf("PERF it_plan_build ok=%d tiempo_ms=%.1f evals=%d us_por_eval=%.1f plan_s=%.3f\n", ok, dt / 1000.0, s_plan.evals,
           s_plan.evals ? (double)dt / s_plan.evals : 0.0, s_plan.time_s);
    /* En rebanadas como el controlador (300 evaluaciones por escaneo): tiempo de cada rebanada. */
    t0 = US();
    it_plan_begin(&s_work, &cfg, -1, obs, 5);
    const uint32_t t_begin = US() - t0;
    static uint32_t sl[400];
    int ns = 0;
    bool done = false;
    while (!done && ns < 400) { const uint32_t a = US(); done = it_plan_step(&s_work, 300); sl[ns++] = US() - a; }
    sort_u32(sl, ns);
    printf("PERF it_rebanadas_300 begin_ms=%.1f n=%d p50_ms=%.1f max_ms=%.1f total_ms=%.1f\n", t_begin / 1000.0, ns, sl[ns / 2] / 1000.0, sl[ns - 1] / 1000.0, 0.0);
    /* Refinado: rebanadas de 800 evaluaciones. */
    if (s_plan.valid && it_plan_refine_begin(&s_work, &s_plan, 20000)) {
        ns = 0; done = false;
        while (!done && ns < 400) { const uint32_t a = US(); done = it_plan_step(&s_work, 800); sl[ns++] = US() - a; }
        sort_u32(sl, ns);
        printf("PERF it_refinado_20000 rebanadas=%d p50_ms=%.1f max_ms=%.1f plan_s=%.3f (antes %.3f)\n", ns, sl[ns / 2] / 1000.0, sl[ns - 1] / 1000.0, s_work.time_s, s_plan.time_s);
    }
    /* Seguidor: una llamada. */
    it_tracker_t tr;
    it_tracker_reset(&tr);
    float st, sp;
    t0 = US();
    int reps = 200;
    for (int i = 0; i < reps; ++i) it_track(&tr, &s_plan, s_plan.x[10], s_plan.z[10], s_plan.heading_deg[10], 80.0f, &st, &sp);
    printf("PERF it_track us=%.1f\n", (double)(US() - t0) / reps);
}

static void bench_park(void)
{
    park_params_t p;
    park_params_default(&p, 13.0f, 26.0f, 3.775f);
    p.margin_cm = 1.5f;
    const park_ackermann_config_t cfg = {18.45f, 40.0f, 3.5f, 1};
    const park_pose_t poses[4] = {{25, 30, 5}, {-30, 45, 0}, {0, 28, 35}, {20, 30, 20}};
    for (int i = 0; i < 4; ++i) {
        const uint32_t t0 = US();
        const park_ackermann_result_t r = park_ackermann_plan(&p, &cfg, &poses[i], PARK_GOAL_IN);
        const uint32_t dt = US() - t0;
        printf("PERF park_astar pose=%d found=%d pasos=%u expandidos=%u tiempo_ms=%.1f\n", i, r.found, r.count, r.expanded, dt / 1000.0);
    }
    /* Solo el calculo de un arco (6 subpasos con colision), sin tocar las tablas del A* (que estan en la PSRAM): separa computo de memoria. */
    {
        const int reps = 20000;
        int freecount = 0;
        const uint32_t ta = US();
        for (int i = 0; i < reps; ++i) {
            const park_pose_t q = {-10.0f + 0.001f * (float)(i % 4000), 30.0f + 0.002f * (float)(i % 500), (float)(i % 40) - 20.0f};
            freecount += park_ackermann_arc_free(&p, &cfg, &q, (i & 1) ? 1 : -1, (i % 3) - 1);
        }
        printf("PERF park_arco us=%.2f (libres %d de %d)\n", (double)(US() - ta) / reps, freecount, reps);
    }
    park_ackermann_result_t r;
    memset(&r, 0, sizeof r);
    const uint32_t t0 = US();
    const bool ok = park_ackermann_entry_maneuver(&p, &cfg, &poses[1], &r);
    printf("PERF park_entry_maneuver ok=%d tiempo_ms=%.1f\n", ok, (US() - t0) / 1000.0);
}

uint32_t wro_prof_now_us(void) { return (uint32_t)esp_timer_get_time(); }
extern uint32_t wro_prof_acc[16];

static void bench_scan(void)
{
    wro_controller_config_t cfg;
    wro_controller_config_default(&cfg);
    cfg.pivot_enabled = false; cfg.sidestep_enabled = false; cfg.peek_enabled = false; cfg.start_in_lot = false;
    cfg.parking_enabled = false;   /* con parqueo el primer barrido pide el A* (12-27 s en el P4): se mide aparte */
    const p4_hal_t hal = {.ctx = NULL, .now_us = f_now, .start_pressed = f_start, .poll_scan = f_scan, .poll_gyro = f_gyro, .read_camera = f_cam, .drive = f_drive};
    const uint32_t ti = US();
    p4_runtime_init(&s_rt, &hal, &cfg);
    printf("PERF p4_runtime_init ms=%.1f\n", (US() - ti) / 1000.0);
    enum { N = 150 };
    static uint32_t dts[N];
    int k = 0;
    /* El tiempo de un paso con barrido nuevo (una vuelta del LiDAR cada 100 ms virtuales). */
    for (int i = 0; i < 16; ++i) wro_prof_acc[i] = 0;
    for (int i = 0; i < 400 && k < N; ++i) {
        s_fake.t_us += 5000;                                   /* 200 Hz de giroscopio */
        const uint32_t before = (uint32_t)s_rt.scans;
        const uint32_t t0 = US();
        p4_runtime_step(&s_rt);
        const uint32_t dt = US() - t0;
        if ((uint32_t)s_rt.scans != before) { if (k < 6) printf("PERF controlador_escaneo_%d ms=%.1f fase=%d\n", k, dt / 1000.0, (int)s_rt.controller.phase); dts[k++] = dt; }
    }
    {
        static const char *const names[16] = {"arena_step", "compensar_movimiento", "mapa_local_update", "trace_map", "analyze", "track_signs", "memoria_letreros", "esquinas", "race_step", "it_manage", "recomendar_planificador", "it_override", "plan_campo_distancias", "plan_antes_cortinas", "plan_cortinas", "plan_corredor"};
        for (int i = 0; i < 16; ++i) printf("PERF seccion %-24s ms_por_barrido=%.2f\n", names[i], (double)wro_prof_acc[i] / 1000.0 / (k ? k : 1));
    }
    if (k > 0) {
        sort_u32(dts, k);
        printf("PERF controlador_escaneo n=%d p50_ms=%.1f p95_ms=%.1f max_ms=%.1f (presupuesto 100 ms; 400 puntos por vuelta)\n", k, dts[k / 2] / 1000.0,
               dts[(95 * k + 99) / 100 - 1] / 1000.0, dts[k - 1] / 1000.0);
    } else {
        printf("PERF controlador_escaneo sin barridos procesados\n");
    }
}

static void perf_bench_task(void *arg)
{
    (void)arg;
    esp_task_wdt_deinit();      /* los pasos largos (A*, plan del IT) no son un fallo aqui: se miden */
    int round = 0;
    while (1) {
        printf("PERF ==== ronda %d ==== IDF %s, CPU %lu MHz\n", ++round, esp_get_idf_version(), (unsigned long)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
        printf("PERF memoria interna libre=%u KB (mayor bloque %u KB), PSRAM libre=%u KB de %u KB\n",
               (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
               (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024));
        bench_scan();
        bench_it();
        bench_park();
        printf("PERF pila libre de esta tarea=%u bytes\n", (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
        printf("PERF FIN ronda %d\n", round);
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (round >= 3) { while (1) vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
}

#ifndef PERF_STACK_BYTES
#define PERF_STACK_BYTES 32768
#endif
void perf_bench_run(void)
{
    /* Tarea propia con pila grande: en la pila de app_main (6 KB) el controlador se estrello con 'Stack protection fault' en su 7.o barrido. */
    xTaskCreatePinnedToCore(perf_bench_task, "perf", PERF_STACK_BYTES, NULL, 5, NULL, 0);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}


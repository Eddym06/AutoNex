#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../main/map_pose.h"

#define PI 3.14159265358979f

static float ray_seg(float ox, float oy, float dx, float dy, float x1, float y1, float x2, float y2)
{
    const float sx = x2 - x1, sy = y2 - y1, den = dx * sy - dy * sx;
    if (fabsf(den) < 1e-9f) return -1.0f;
    const float t = ((x1 - ox) * sy - (y1 - oy) * sx) / den, u = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
    return (t > 0.0f && u >= 0.0f && u <= 1.0f) ? t : -1.0f;
}

/* Cuarto de 300 x 300 cm con una caja y un pilar, para que el perfil de distancias tenga rasgos. */
static float cast(float ox, float oy, float dx, float dy)
{
    const float W[12][4] = {{-150, -150, 150, -150}, {150, -150, 150, 150}, {150, 150, -150, 150}, {-150, 150, -150, -150},
                            {45, 25, 75, 25}, {75, 25, 75, 55}, {75, 55, 45, 55}, {45, 55, 45, 25},
                            {-80, -60, -60, -60}, {-60, -60, -60, -30}, {-60, -30, -80, -30}, {-80, -30, -80, -60}};
    float best = 1e9f;
    for (int i = 0; i < 12; ++i) {
        const float t = ray_seg(ox, oy, dx, dy, W[i][0], W[i][1], W[i][2], W[i][3]);
        if (t > 0.0f && t < best) best = t;
    }
    return best;
}

static int scan(local_lidar_point_t *p, float heading_deg, float noise)
{
    const float th = heading_deg * PI / 180.0f, fe = sinf(th), fn = cosf(th), re = cosf(th), rn = -sinf(th);
    int k = 0;
    for (float a = -179.5f; a < 180.0f; a += 0.9f) {
        const float ar = a * PI / 180.0f, dx = fe * cosf(ar) + re * sinf(ar), dy = fn * cosf(ar) + rn * sinf(ar);
        p[k].angle_deg = a;
        p[k].distance_cm = cast(0.0f, 0.0f, dx, dy) + noise * ((float)rand() / RAND_MAX - 0.5f) * 2.0f;
        p[k].valid = true; p[k].blind = false; p[k].intensity = 1.0f;
        ++k;
    }
    return k;
}

static uint64_t T;                                         /* reloj del giroscopio, a 400 Hz */
static void gyro_for(map_pose_t *m, float dps, float seconds)
{
    for (int i = 0; i < (int)(seconds * 400.0f); ++i) { T += 2500; map_pose_gyro(m, dps, T); }
}

int main(void)
{
    static local_lidar_point_t pts[512];
    static map_pose_t m;
    srand(11);

    /* 1) Robot quieto con un sesgo de 1,0 grados/s en el giroscopio: debe aprenderlo y dejar de derivar. */
    memset(&m, 0, sizeof m);
    map_pose_reset(&m);
    for (int i = 0; i < 300; ++i) {                        /* 30 s a 10 barridos/s */
        gyro_for(&m, 1.0f, 0.1f);
        const int n = scan(pts, 0.0f, 0.3f);
        map_pose_scan(&m, pts, (size_t)n, 0.0f);
    }
    printf("reposo: sesgo aprendido %.2f dps (real 1.00), %d actualizaciones\n", m.bias_dps, m.bias_updates);
    assert(fabsf(m.bias_dps - 1.0f) < 0.15f);
    const float h0 = m.heading_deg;
    for (int i = 0; i < 100; ++i) { gyro_for(&m, 1.0f, 0.1f); const int n = scan(pts, 0.0f, 0.3f); map_pose_scan(&m, pts, (size_t)n, 0.0f); }
    printf("deriva en 10 s tras aprender: %.2f grados (antes 10.0)\n", m.heading_deg - h0);
    assert(fabsf(m.heading_deg - h0) < 1.0f);
    assert(!m.flip_request);

    /* 2) Giro horario de 6 grados por barrido con el giroscopio bien (+60 dps): no se pide invertir nada. */
    memset(&m, 0, sizeof m);
    map_pose_reset(&m);
    float truth = 0.0f;
    for (int i = 0; i < 20; ++i) {
        truth += 6.0f;
        gyro_for(&m, 60.0f, 0.1f);
        const int n = scan(pts, truth, 0.2f);
        map_pose_scan(&m, pts, (size_t)n, 0.0f);
    }
    printf("giro correcto: rumbo %.1f (real %.1f), votos ok %d, flip %d\n", m.heading_deg, truth, m.votes_ok, m.flip_request);
    assert(!m.flip_request && m.votes_ok >= 3);

    /* 3) El mismo giro con el giroscopio invertido (-60 dps): hay que pedir invertir el signo. */
    memset(&m, 0, sizeof m);
    map_pose_reset(&m);
    truth = 0.0f;
    for (int i = 0; i < 20; ++i) {
        truth += 6.0f;
        gyro_for(&m, -60.0f, 0.1f);
        const int n = scan(pts, truth, 0.2f);
        map_pose_scan(&m, pts, (size_t)n, 0.0f);
    }
    printf("giro invertido: flip_request %d\n", m.flip_request);
    assert(m.flip_request);

    puts("gyro_cal OK");
    return 0;
}

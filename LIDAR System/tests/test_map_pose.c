#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../main/map_pose.h"

#define PI 3.14159265358979f

/* Interseccion de un rayo (ox,oy)+t(dx,dy) con un segmento; devuelve t o -1. */
static float ray_seg(float ox, float oy, float dx, float dy, float x1, float y1, float x2, float y2)
{
    const float sx = x2 - x1, sy = y2 - y1, den = dx * sy - dy * sx;
    if (fabsf(den) < 1e-9f) return -1.0f;
    const float t = ((x1 - ox) * sy - (y1 - oy) * sx) / den, u = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
    return (t > 0.0f && u >= 0.0f && u <= 1.0f) ? t : -1.0f;
}

/* Cuarto de 300 x 300 cm centrado en el origen con una caja de 30 cm en (60, 40). */
static float cast(float ox, float oy, float dx, float dy)
{
    const float W[8][4] = {{-150, -150, 150, -150}, {150, -150, 150, 150}, {150, 150, -150, 150}, {-150, 150, -150, -150},
                           {45, 25, 75, 25}, {75, 25, 75, 55}, {75, 55, 45, 55}, {45, 55, 45, 25}};
    float best = 1e9f;
    for (int i = 0; i < 8; ++i) {
        const float t = ray_seg(ox, oy, dx, dy, W[i][0], W[i][1], W[i][2], W[i][3]);
        if (t > 0.0f && t < best) best = t;
    }
    return best;
}

static int scan(local_lidar_point_t *p, float e, float n, float heading_deg, float noise)
{
    const float th = heading_deg * PI / 180.0f, fe = sinf(th), fn = cosf(th), re = cosf(th), rn = -sinf(th);
    int k = 0;
    for (float a = -179.5f; a < 180.0f; a += 0.9f) {
        const float ar = a * PI / 180.0f, dx = fe * cosf(ar) + re * sinf(ar), dy = fn * cosf(ar) + rn * sinf(ar);
        const float d = cast(e, n, dx, dy) + noise * ((float)rand() / RAND_MAX - 0.5f) * 2.0f;
        p[k].angle_deg = a; p[k].distance_cm = d; p[k].valid = true; p[k].blind = false; p[k].intensity = 1.0f;
        ++k;
    }
    return k;
}

int main(void)
{
    static local_lidar_point_t pts[512];
    map_pose_t m;
    memset(&m, 0, sizeof m);
    map_pose_reset(&m);
    srand(7);

    /* Giroscopio: 90 grados/s durante 0,5 s a 400 Hz = 45 grados; el rumbo es horario positivo. */
    for (int i = 1; i <= 200; ++i) map_pose_gyro(&m, 90.0f, (uint64_t)i * 2500u);
    assert(fabsf(m.heading_deg - 45.0f) < 0.5f);

    /* Reposo con ruido de +-0,5 cm: la posicion no debe derivar. */
    float te = -30.0f, tn = 20.0f;                       /* posicion real: el robot esta en (-30, 20) */
    int n = scan(pts, te, tn, 45.0f, 0.5f);
    map_pose_scan(&m, pts, (size_t)n, 0.0f);             /* primer barrido: define el origen (0,0) */
    for (int i = 0; i < 40; ++i) { n = scan(pts, te, tn, 45.0f, 0.5f); map_pose_scan(&m, pts, (size_t)n, 0.0f); }
    assert(fabsf(m.e_cm) < 1.0f && fabsf(m.n_cm) < 1.0f);

    /* Movimiento a mano: 3 cm por barrido hacia el este y 1 cm hacia el sur durante 25 barridos (= 75 cm, -25 cm). */
    for (int i = 0; i < 25; ++i) {
        te += 3.0f; tn -= 1.0f;
        n = scan(pts, te, tn, 45.0f, 0.5f);
        map_pose_scan(&m, pts, (size_t)n, 0.0f);
    }
    printf("movimiento: estimado (%.1f, %.1f), real (75.0, -25.0)\n", m.e_cm, m.n_cm);
    assert(fabsf(m.e_cm - 75.0f) < 3.0f && fabsf(m.n_cm + 25.0f) < 3.0f);

    /* Movimiento lento (0,2 cm por barrido, bajo la zona muerta): se acumula en vez de perderse. */
    for (int i = 0; i < 50; ++i) {
        tn += 0.2f;
        n = scan(pts, te, tn, 45.0f, 0.0f);
        map_pose_scan(&m, pts, (size_t)n, 0.0f);
    }
    printf("lento: norte estimado %.1f, real -15.0\n", m.n_cm);
    assert(fabsf(m.n_cm - (-25.0f + 10.0f)) < 3.0f);
    puts("map_pose OK");
    return 0;
}

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../main/height_est.h"

#define PI 3.14159265358979f

/* Fuerza especifica de un IMU con el morro (eje x) levantado theta grados: u = z cos + x sin. */
static void tilted(float theta_deg, float a[3])
{
    const float t = theta_deg * PI / 180.0f;
    a[0] = sinf(t); a[1] = 0.0f; a[2] = cosf(t);
}

static void settle(height_est_t *h, const float a[3])
{
    for (int i = 0; i < 1000; ++i) height_accel(h, a, 0.0025f);   /* 2,5 s simulados: el filtro (150 ms) se asienta */
}

int main(void)
{
    height_est_t h;
    float flat[3] = {0, 0, 1}, up10[3], g2[3] = {0.9f, 0.0f, 1.0f};
    tilted(10.0f, up10);

    height_reset(&h);
    settle(&h, flat);
    height_start(&h);                                    /* referencia: robot plano, altura 0 */
    assert(h.have_ref && fabsf(h.height_cm) < 1e-3f);

    settle(&h, up10);                                    /* levantar el morro 10 grados para aprender el eje delantero */
    assert(h.tilt_deg > 9.0f && h.tilt_deg < 11.0f);
    assert(height_calibrate_axis(&h) && h.fwd_cal);
    assert(fabsf(h.pitch_deg - 10.0f) < 0.6f);           /* morro arriba = positivo */

    /* Una aceleracion fuerte (1,34 g) no cuenta como inclinacion. */
    const float before = h.pitch_deg;
    for (int i = 0; i < 100; ++i) height_accel(&h, g2, 0.0025f);
    assert(fabsf(h.pitch_deg - before) < 1e-3f);

    height_start(&h);                                    /* nueva referencia con el robot inclinado: de nuevo 0 */
    settle(&h, flat);
    height_start(&h);
    settle(&h, up10);
    int changed = 0;
    for (int i = 0; i < 20; ++i) changed += height_move(&h, 10.0f);     /* 200 cm cuesta arriba: 200 sen 10 = 34,7 cm */
    printf("subida: altura %.1f cm, nivel %d\n", h.height_cm, h.level);
    assert(fabsf(h.height_cm - 34.73f) < 1.0f && h.level == 1 && changed == 1);

    for (int i = 0; i < 10; ++i) height_move(&h, -10.0f);                 /* marcha atras 100 cm cuesta abajo */
    printf("marcha atras: altura %.1f cm\n", h.height_cm);
    assert(fabsf(h.height_cm - 17.36f) < 1.0f && h.level == 1);

    settle(&h, flat);                                    /* en plano no cambia la altura */
    const float hh = h.height_cm;
    for (int i = 0; i < 50; ++i) height_move(&h, 10.0f);
    assert(fabsf(h.height_cm - hh) < 0.5f);

    puts("height_est OK");
    return 0;
}

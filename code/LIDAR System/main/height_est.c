#include "height_est.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f

static float dot3(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static int normalize3(float v[3])
{
    const float l = sqrtf(dot3(v, v));
    if (l < 1e-6f) return 0;
    v[0] /= l; v[1] /= l; v[2] /= l;
    return 1;
}

/* Hace v perpendicular a n (unitario) y lo normaliza. */
static int perp_unit(float v[3], const float n[3])
{
    const float d = dot3(v, n);
    v[0] -= d * n[0]; v[1] -= d * n[1]; v[2] -= d * n[2];
    return normalize3(v);
}

static void default_forward(height_est_t *h)
{
    float f[3] = {1.0f, 0.0f, 0.0f};                    /* GY-521 con X hacia el morro, hasta aprenderlo */
    if (!perp_unit(f, h->u0)) { f[0] = 0.0f; f[1] = 1.0f; f[2] = 0.0f; perp_unit(f, h->u0); }
    memcpy(h->fwd, f, sizeof f);
}

void height_reset(height_est_t *h)
{
    memset(h, 0, sizeof *h);
    h->u0[2] = 1.0f;
    h->u[2] = 1.0f;
    default_forward(h);
}

void height_accel(height_est_t *h, const float a[3], float dt)
{
    const float m = sqrtf(dot3(a, a));
    if (m < 0.93f || m > 1.07f) return;                  /* hay una aceleracion fuerte: no es solo la gravedad */
    float v[3] = {a[0] / m, a[1] / m, a[2] / m};
    if (!h->have_u) { memcpy(h->u, v, sizeof v); h->have_u = true; }
    else {
        const float k = dt > 0.05f ? 1.0f : dt / (dt + 0.15f);    /* filtro de primer orden, ~150 ms */
        for (int i = 0; i < 3; ++i) h->u[i] += k * (v[i] - h->u[i]);
        normalize3(h->u);
    }
    const float c = fmaxf(-1.0f, fminf(1.0f, dot3(h->u, h->u0)));
    h->tilt_deg = acosf(c) * 180.0f / PI_F;
    float s = dot3(h->u, h->fwd);                        /* sen(inclinacion) = componente de u sobre el eje delantero */
    s = fmaxf(-1.0f, fminf(1.0f, s));
    h->pitch_deg = asinf(s) * 180.0f / PI_F;
}

void height_start(height_est_t *h)
{
    if (!h->have_u) return;
    memcpy(h->u0, h->u, sizeof h->u);
    if (h->fwd_cal) perp_unit(h->fwd, h->u0); else default_forward(h);
    h->have_ref = true;
    h->height_cm = 0.0f;
    h->level_origin_cm = 0.0f;
    h->level = 0;
    h->pitch_deg = h->tilt_deg = 0.0f;
}

bool height_calibrate_axis(height_est_t *h)
{
    if (!h->have_u || !h->have_ref) return false;
    if (h->tilt_deg < 5.0f) return false;
    float f[3] = {h->u[0] - h->u0[0], h->u[1] - h->u0[1], h->u[2] - h->u0[2]};
    if (!perp_unit(f, h->u0)) return false;
    memcpy(h->fwd, f, sizeof f);
    h->fwd_cal = true;
    return true;
}

bool height_move(height_est_t *h, float forward_cm)
{
    if (!h->have_ref) return false;
    h->height_cm += forward_cm * sinf(h->pitch_deg * PI_F / 180.0f);
    if (fabsf(h->height_cm - h->level_origin_cm) > HEIGHT_LEVEL_CM) {
        ++h->level;
        h->level_origin_cm = h->height_cm;               /* el nivel nuevo mide desde aqui */
        return true;
    }
    return false;
}

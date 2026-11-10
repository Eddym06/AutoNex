#include "race_line.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD_TO_DEG 57.29577951308232f
#define PI_F 3.14159265358979323846f

void race_config_default(race_config_t *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->outer_half_cm = 150.0f;
    c->inner_half_cm = 50.0f;
    c->half_width_cm = 6.5f;
    c->wall_margin_cm = 7.0f;
    c->sign_margin_cm = 3.0f;
    c->sign_half_cm = 2.5f;
    c->curtain_cm = 100.0f;
    c->approach_cm = 25.0f;
    c->pass_side_red = 1;
    c->pass_side_green = -1;
    c->station_cm = 5.0f;
    c->corner_radius_cm = 40.0f;
    c->max_offset_cm = 24.0f;
    c->v_max_cm_s = 52.0f;
    c->v_sign_cm_s = 35.0f;
    c->v_corner_cm_s = 35.0f;
    c->a_lat_cm_s2 = 120.0f;
    c->a_acc_cm_s2 = 45.0f;
    c->a_dec_cm_s2 = 60.0f;
    c->sign_before_cm = 60.0f;
    c->sign_after_cm = 12.0f;
    c->corner_before_cm = 30.0f;
    c->corner_after_cm = 10.0f;
    c->iterations = 900;
}

static bool in_range(float *v, float lo, float hi)
{
    bool ok = isfinite(*v);
    if (!ok) *v = lo;
    if (*v < lo) { *v = lo; ok = false; }
    if (*v > hi) { *v = hi; ok = false; }
    return ok;
}

bool race_config_sanitize(race_config_t *c)
{
    if (!c) return false;
    bool ok = true;
    ok &= in_range(&c->outer_half_cm, 60.0f, 500.0f);
    ok &= in_range(&c->inner_half_cm, 10.0f, c->outer_half_cm - 30.0f);
    ok &= in_range(&c->half_width_cm, 2.0f, 30.0f);
    ok &= in_range(&c->wall_margin_cm, 0.0f, 20.0f);
    ok &= in_range(&c->sign_margin_cm, 0.0f, 20.0f);
    ok &= in_range(&c->sign_half_cm, 1.0f, 15.0f);
    ok &= in_range(&c->curtain_cm, 20.0f, 300.0f);
    ok &= in_range(&c->approach_cm, 0.0f, 100.0f);
    if (c->pass_side_red > 1) c->pass_side_red = 1;
    if (c->pass_side_red < -1) c->pass_side_red = -1;
    if (c->pass_side_green > 1) c->pass_side_green = 1;
    if (c->pass_side_green < -1) c->pass_side_green = -1;
    ok &= in_range(&c->station_cm, 3.0f, 12.0f);
    ok &= in_range(&c->max_offset_cm, 0.0f, 100.0f);
    ok &= in_range(&c->corner_radius_cm, 15.0f, 0.5f * (c->outer_half_cm + c->inner_half_cm) - 5.0f);
    ok &= in_range(&c->v_max_cm_s, 5.0f, 120.0f);
    ok &= in_range(&c->v_sign_cm_s, 5.0f, c->v_max_cm_s);
    ok &= in_range(&c->v_corner_cm_s, 5.0f, c->v_max_cm_s);
    ok &= in_range(&c->a_lat_cm_s2, 10.0f, 2000.0f);
    ok &= in_range(&c->a_acc_cm_s2, 5.0f, 400.0f);
    ok &= in_range(&c->a_dec_cm_s2, 5.0f, 600.0f);
    ok &= in_range(&c->sign_before_cm, 0.0f, 200.0f);
    ok &= in_range(&c->sign_after_cm, 0.0f, 200.0f);
    ok &= in_range(&c->corner_before_cm, 0.0f, 200.0f);
    ok &= in_range(&c->corner_after_cm, 0.0f, 200.0f);
    if (c->iterations < 50) { c->iterations = 50; ok = false; }
    if (c->iterations > 5000) { c->iterations = 5000; ok = false; }
    return ok;
}

/* ------------------------------------------------------------ linea central */

/* Punto de la linea central (antihoraria) a `s` cm del inicio: esquinas redondeadas de radio rc. */
static void center_at(const race_config_t *c, float s, float *x, float *z, float *tx, float *tz, bool *arc)
{
    const float h = 0.5f * (c->outer_half_cm + c->inner_half_cm);
    const float rc = c->corner_radius_cm;
    const float len = 2.0f * (h - rc), al = 0.5f * PI_F * rc;
    const float period = 4.0f * (len + al);
    s = fmodf(s, period);
    if (s < 0.0f) s += period;
    int piece = (int)(s / (len + al));
    if (piece > 3) piece = 3;
    float t = s - (float)piece * (len + al);
    static const float DX[4] = {1.0f, 0.0f, -1.0f, 0.0f}, DZ[4] = {0.0f, 1.0f, 0.0f, -1.0f};
    /* Inicio de cada recta, y centro y angulo inicial de cada arco. */
    const float sx[4] = {-(h - rc), h, h - rc, -h}, sz[4] = {-h, -(h - rc), h, h - rc};
    const float ax[4] = {h - rc, h - rc, -(h - rc), -(h - rc)}, az[4] = {-(h - rc), h - rc, h - rc, -(h - rc)};
    *arc = false;
    if (t <= len) {
        *x = sx[piece] + DX[piece] * t; *z = sz[piece] + DZ[piece] * t;
        *tx = DX[piece]; *tz = DZ[piece];
    } else {
        t -= len;
        const float th = (-0.5f + (float)piece * 0.5f) * PI_F + t / rc;   /* -90, 0, 90, 180 grados al empezar */
        *x = ax[piece] + rc * cosf(th); *z = az[piece] + rc * sinf(th);
        *tx = -sinf(th); *tz = cosf(th);
        *arc = true;
    }
}

/* ------------------------------------------------------------ espacio libre */

static bool free_at(const race_line_t *r, float x, float z)
{
    const race_config_t *c = &r->cfg;
    const float rho = fmaxf(fabsf(x), fabsf(z));
    const float m = c->half_width_cm + c->wall_margin_cm;
    if (rho > c->outer_half_cm - m || rho < c->inner_half_cm + m) return false;
    const float rs = c->sign_half_cm * 1.4142f + c->half_width_cm + c->sign_margin_cm;
    for (int j = 0; j < r->signs; ++j) {
        const race_sign_t *g = &r->sign[j];
        if (hypotf(x - g->x_cm, z - g->z_cm) < rs) return false;
        /* Cortina: cierra el lado por el que el color no deja pasar, y la entrada a el unos centimetros antes del
         * letrero (hace falta distancia para cambiar de lado). Su grosor es el radio del letrero: toda estacion cuya
         * normal cruza el disco queda con ese lado cerrado, y ninguna ruta se cuela entre dos. */
        const int side = g->color == 1 ? c->pass_side_red : g->color == 2 ? c->pass_side_green : 0;
        if (side == 0) continue;
        const float rx = r->sign_rx[j], rz = r->sign_rz[j];   /* derecha del avance en el letrero */
        const float fx = -(float)side * rx, fz = -(float)side * rz;   /* hacia el lado prohibido */
        const float tx = -rz, tz = rx;                                 /* avance */
        const float px = x - g->x_cm, pz = z - g->z_cm;
        const float lam = px * fx + pz * fz, ell = px * tx + pz * tz;
        if (lam <= c->curtain_cm && lam >= -rs && ell >= -(rs + c->approach_cm) && ell <= rs && lam >= 0.0f) return false;
    }
    return true;
}

/* ------------------------------------------------------------ perfil */

static float cyc(const race_line_t *r, float ds)
{
    /* Distancia con signo en (-P/2, P/2]. */
    const float p = r->length_cm;
    while (ds > 0.5f * p) ds -= p;
    while (ds <= -0.5f * p) ds += p;
    return ds;
}

/* Arco, curvatura, rumbo y velocidad de la linea (x, z) ya colocada. */
static void profile(race_line_t *r)
{
    const race_config_t *c = &r->cfg;
    const int n = r->n;
    float acc = 0.0f;
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        r->s[i] = acc;
        r->h[i] = hypotf(r->x[j] - r->x[i], r->z[j] - r->z[i]);
        if (r->h[i] < 0.01f) r->h[i] = 0.01f;
        acc += r->h[i];
    }
    r->length_cm = acc;
    for (int i = 0; i < n; ++i) {
        const int a = (i + n - 1) % n, b = (i + 1) % n;
        const float ux = r->x[i] - r->x[a], uz = r->z[i] - r->z[a];
        const float vx = r->x[b] - r->x[i], vz = r->z[b] - r->z[i];
        const float wx = r->x[b] - r->x[a], wz = r->z[b] - r->z[a];
        const float la = hypotf(ux, uz), lb = hypotf(vx, vz), lc = hypotf(wx, wz);
        const float cross = ux * vz - uz * vx;
        r->kappa[i] = (la > 0.01f && lb > 0.01f && lc > 0.01f) ? 2.0f * fabsf(cross) / (la * lb * lc) : 0.0f;
        r->heading_deg[i] = atan2f(wx, wz) * RAD_TO_DEG;
    }
    /* Arco de cada letrero sobre la linea. */
    for (int j = 0; j < r->signs; ++j) {
        int best = 0;
        float bd = 1.0e9f;
        for (int i = 0; i < n; ++i) {
            const float d = hypotf(r->x[i] - r->sign[j].x_cm, r->z[i] - r->sign[j].z_cm);
            if (d < bd) { bd = d; best = i; }
        }
        r->sign_s[j] = r->s[best];
    }
    /* Topes de velocidad. */
    for (int i = 0; i < n; ++i) {
        float v = c->v_max_cm_s;
        const float k = r->kappa[i];
        if (k > 1.0e-4f) {
            const float vk = sqrtf(c->a_lat_cm_s2 / k);
            if (vk < v) v = vk;
        }
        for (int j = 0; j < r->signs && v > c->v_sign_cm_s; ++j) {
            const float along = cyc(r, r->sign_s[j] - r->s[i]);       /* + si el letrero esta por delante */
            if ((along >= 0.0f && along <= c->sign_before_cm) || (along < 0.0f && -along <= c->sign_after_cm)) v = c->v_sign_cm_s;
        }
        for (int k2 = 0; k2 < n && v > c->v_corner_cm_s; ++k2) {
            if (!r->corner[k2]) continue;
            const float along = cyc(r, r->s[k2] - r->s[i]);
            if ((along >= 0.0f && along <= c->corner_before_cm) || (along < 0.0f && -along <= c->corner_after_cm)) v = c->v_corner_cm_s;
        }
        /* Nunca por debajo de la velocidad de zona: esa es la que el robot ya conduce con seguridad (en una esquina
         * el controlador gira en el sitio, no sigue el radio de la linea). */
        const float floor_v = c->v_sign_cm_s < c->v_corner_cm_s ? c->v_sign_cm_s : c->v_corner_cm_s;
        if (v < floor_v) v = floor_v;
        r->v[i] = v;
    }
    /* Rampas: aceleracion hacia delante y frenada hacia atras, dando dos vueltas para cerrar el anillo. */
    for (int pass = 0; pass < 2; ++pass) {
        for (int k = 0; k < n; ++k) {
            const int i = (k + 1) % n, a = k;
            const float vmax = sqrtf(r->v[a] * r->v[a] + 2.0f * c->a_acc_cm_s2 * r->h[a]);
            if (r->v[i] > vmax) r->v[i] = vmax;
        }
        for (int k = n - 1; k >= 0; --k) {
            const int i = k, b = (k + 1) % n;
            const float vmax = sqrtf(r->v[b] * r->v[b] + 2.0f * c->a_dec_cm_s2 * r->h[i]);
            if (r->v[i] > vmax) r->v[i] = vmax;
        }
    }
    float t = 0.0f;
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        float vm = 0.5f * (r->v[i] + r->v[j]);
        if (vm < 5.0f) vm = 5.0f;
        t += r->h[i] / vm;
    }
    r->time_s = t;
}

/* Sobrerrelajacion proyectada: minimiza  1/2 sum |p[i+1]-p[i]|^2 + mix/2 sum |p[i+1]-2p[i]+p[i-1]|^2  con
 * p[i] = c[i] + d[i] n[i] y lo[i] <= d[i] <= hi[i]. */
static void relax(race_line_t *r, float mix, float *d)
{
    const int n = r->n;
    static float px[RACE_MAX_STATIONS], pz[RACE_MAX_STATIONS];
    for (int i = 0; i < n; ++i) { px[i] = r->cx[i] + d[i] * r->nx[i]; pz[i] = r->cz[i] + d[i] * r->nz[i]; }
    const float hess = 2.0f + 6.0f * mix, omega = 1.85f;
    for (int it = 0; it < r->cfg.iterations; ++it) {
        float worst = 0.0f;
        for (int i = 0; i < n; ++i) {
            const int a = (i + n - 1) % n, a2 = (i + n - 2) % n, b = (i + 1) % n, b2 = (i + 2) % n;
            const float lx = px[a] - 2.0f * px[i] + px[b], lz = pz[a] - 2.0f * pz[i] + pz[b];
            const float bx = px[a2] - 4.0f * px[a] + 6.0f * px[i] - 4.0f * px[b] + px[b2];
            const float bz = pz[a2] - 4.0f * pz[a] + 6.0f * pz[i] - 4.0f * pz[b] + pz[b2];
            const float g = (-lx + mix * bx) * r->nx[i] + (-lz + mix * bz) * r->nz[i];
            float nd = d[i] - omega * g / hess;
            if (nd < r->lo[i]) nd = r->lo[i];
            if (nd > r->hi[i]) nd = r->hi[i];
            const float dd = nd - d[i];
            if (fabsf(dd) > worst) worst = fabsf(dd);
            d[i] = nd;
            px[i] += dd * r->nx[i]; pz[i] += dd * r->nz[i];
        }
        if (worst < 0.005f) break;
    }
}

bool race_line_build(race_line_t *r, const race_config_t *cfg, int direction, const race_sign_t *signs, int count)
{
    if (!r) return false;
    memset(r, 0, sizeof(*r));
    if (cfg) r->cfg = *cfg; else race_config_default(&r->cfg);
    race_config_sanitize(&r->cfg);
    const race_config_t *c = &r->cfg;
    r->direction = direction >= 0 ? 1 : -1;
    if (count < 0) count = 0;
    if (count > RACE_MAX_SIGNS) count = RACE_MAX_SIGNS;
    r->signs = count;
    for (int j = 0; j < count; ++j) r->sign[j] = signs[j];

    const float h = 0.5f * (c->outer_half_cm + c->inner_half_cm);
    const float rc = c->corner_radius_cm;
    const float period = 4.0f * (2.0f * (h - rc) + 0.5f * PI_F * rc);
    int n = (int)(period / c->station_cm + 0.5f);
    if (n > RACE_MAX_STATIONS) n = RACE_MAX_STATIONS;
    if (n < 16) n = 16;
    r->n = n;
    const float step = period / (float)n;
    for (int i = 0; i < n; ++i) {
        /* Sentido antihorario: el parametro crece; horario: la misma linea recorrida al reves. */
        const float sp = r->direction < 0 ? (float)i * step : period - (float)i * step;
        float tx, tz;
        bool arc;
        center_at(c, sp, &r->cx[i], &r->cz[i], &tx, &tz, &arc);
        if (r->direction > 0) { tx = -tx; tz = -tz; }
        r->nx[i] = tz; r->nz[i] = -tx;           /* a la derecha del avance */
        r->corner[i] = arc;
    }
    /* Derecha del avance en cada letrero (estacion central mas cercana). */
    for (int j = 0; j < count; ++j) {
        int best = 0;
        float bd = 1.0e9f;
        for (int i = 0; i < n; ++i) {
            const float d = hypotf(r->cx[i] - signs[j].x_cm, r->cz[i] - signs[j].z_cm);
            if (d < bd) { bd = d; best = i; }
        }
        r->sign_rx[j] = r->nx[best]; r->sign_rz[j] = r->nz[best];
    }
    /* Tramo libre de cada estacion. Si la normal cruza un letrero o su cortina hay mas de uno: se elige uno por
     * estacion minimizando el salto entre estaciones vecinas (programacion dinamica sobre el anillo), para que la linea
     * no salte de un lado al otro ni se meta en un tramo sin salida. */
    const float reach = 0.5f * (c->outer_half_cm - c->inner_half_cm) + 5.0f;
    enum { MAXIV = 6 };
    static float iv_lo[RACE_MAX_STATIONS][MAXIV], iv_hi[RACE_MAX_STATIONS][MAXIV];
    static int iv_n[RACE_MAX_STATIONS];
    for (int i = 0; i < n; ++i) {
        int ni = 0;
        float run_lo = 0.0f, last = 0.0f;
        bool in_run = false;
        for (float d = -reach; d <= reach + 0.01f; d += 1.0f) {
            const bool f = free_at(r, r->cx[i] + d * r->nx[i], r->cz[i] + d * r->nz[i]);
            if (f && !in_run) { in_run = true; run_lo = d; }
            if (!f && in_run) {
                if (ni < MAXIV) { iv_lo[i][ni] = run_lo; iv_hi[i][ni] = last; ++ni; }
                in_run = false;
            }
            last = d;
        }
        if (in_run && ni < MAXIV) { iv_lo[i][ni] = run_lo; iv_hi[i][ni] = last; ++ni; }
        if (ni == 0) { r->valid = false; return false; }
        iv_n[i] = ni;
    }
    {
        static float cost[RACE_MAX_STATIONS][MAXIV];
        static int8_t from[RACE_MAX_STATIONS][MAXIV];
        float best_total = 1.0e18f;
        static int8_t best_pick[RACE_MAX_STATIONS];
        for (int s0 = 0; s0 < iv_n[0]; ++s0) {
            for (int k = 0; k < MAXIV; ++k) cost[0][k] = 1.0e18f;
            cost[0][s0] = 0.0f;
            for (int i = 1; i < n; ++i) {
                for (int k = 0; k < iv_n[i]; ++k) {
                    const float mx = r->cx[i] + 0.5f * (iv_lo[i][k] + iv_hi[i][k]) * r->nx[i];
                    const float mz = r->cz[i] + 0.5f * (iv_lo[i][k] + iv_hi[i][k]) * r->nz[i];
                    cost[i][k] = 1.0e18f;
                    from[i][k] = 0;
                    for (int q = 0; q < iv_n[i - 1]; ++q) {
                        if (cost[i - 1][q] >= 1.0e17f) continue;
                        const float qx = r->cx[i - 1] + 0.5f * (iv_lo[i - 1][q] + iv_hi[i - 1][q]) * r->nx[i - 1];
                        const float qz = r->cz[i - 1] + 0.5f * (iv_lo[i - 1][q] + iv_hi[i - 1][q]) * r->nz[i - 1];
                        const float cst = cost[i - 1][q] + hypotf(mx - qx, mz - qz);
                        if (cst < cost[i][k]) { cost[i][k] = cst; from[i][k] = (int8_t)q; }
                    }
                }
            }
            for (int k = 0; k < iv_n[n - 1]; ++k) {
                if (cost[n - 1][k] >= 1.0e17f) continue;
                const float qx = r->cx[n - 1] + 0.5f * (iv_lo[n - 1][k] + iv_hi[n - 1][k]) * r->nx[n - 1];
                const float qz = r->cz[n - 1] + 0.5f * (iv_lo[n - 1][k] + iv_hi[n - 1][k]) * r->nz[n - 1];
                const float mx = r->cx[0] + 0.5f * (iv_lo[0][s0] + iv_hi[0][s0]) * r->nx[0];
                const float mz = r->cz[0] + 0.5f * (iv_lo[0][s0] + iv_hi[0][s0]) * r->nz[0];
                const float total = cost[n - 1][k] + hypotf(mx - qx, mz - qz);
                if (total < best_total) {
                    best_total = total;
                    int pick = k;
                    for (int i = n - 1; i >= 0; --i) { best_pick[i] = (int8_t)pick; pick = from[i][pick]; }
                }
            }
        }
        if (best_total >= 1.0e17f) { r->valid = false; return false; }
        for (int i = 0; i < n; ++i) { r->lo[i] = iv_lo[i][best_pick[i]]; r->hi[i] = iv_hi[i][best_pick[i]]; }
        /* Cota de seguimiento: la linea no se separa del centro mas de max_offset_cm (salvo que un letrero lo obligue). */
        for (int i = 0; i < n; ++i) {
            const float nlo = r->lo[i] > -c->max_offset_cm ? r->lo[i] : -c->max_offset_cm;
            const float nhi = r->hi[i] < c->max_offset_cm ? r->hi[i] : c->max_offset_cm;
            if (nlo <= nhi) { r->lo[i] = nlo; r->hi[i] = nhi; }
        }
    }

    /* Prueba mezclas de longitud y curvatura y se queda con la vuelta mas rapida. */
    static const float MIX[RACE_CANDIDATES] = {0.0f, 0.3f, 1.0f, 3.0f, 10.0f, 30.0f};
    static float d_try[RACE_MAX_STATIONS], d_best[RACE_MAX_STATIONS];
    float best_time = 1.0e9f;
    for (int k = 0; k < RACE_CANDIDATES; ++k) {
        for (int i = 0; i < n; ++i) {
            float v0 = 0.5f * (r->lo[i] + r->hi[i]);
            d_try[i] = v0;
        }
        relax(r, MIX[k], d_try);
        for (int i = 0; i < n; ++i) { r->x[i] = r->cx[i] + d_try[i] * r->nx[i]; r->z[i] = r->cz[i] + d_try[i] * r->nz[i]; }
        profile(r);
        if (r->time_s < best_time) {
            best_time = r->time_s;
            memcpy(d_best, d_try, sizeof(float) * (size_t)n);
            r->mix = MIX[k];
        }
    }
    /* Referencia: la linea central, acotada a su tramo libre. */
    for (int i = 0; i < n; ++i) {
        float v0 = 0.0f;
        if (v0 < r->lo[i]) v0 = r->lo[i];
        if (v0 > r->hi[i]) v0 = r->hi[i];
        r->x[i] = r->cx[i] + v0 * r->nx[i]; r->z[i] = r->cz[i] + v0 * r->nz[i];
    }
    profile(r);
    r->time_center_s = r->time_s;
    for (int i = 0; i < n; ++i) {
        r->d[i] = d_best[i];
        r->x[i] = r->cx[i] + d_best[i] * r->nx[i];
        r->z[i] = r->cz[i] + d_best[i] * r->nz[i];
    }
    profile(r);
    r->valid = true;
    return true;
}

/* ------------------------------------------------------------ consulta */

int race_line_locate(const race_line_t *r, float x, float z, int hint, float window_cm, float *lateral_cm, float *dist_cm)
{
    if (!r || !r->valid) return -1;
    const int n = r->n;
    int best = -1;
    float bd = 1.0e18f;
    if (hint >= 0 && hint < n && window_cm > 0.0f) {
        const int span = (int)(window_cm / (r->length_cm / (float)n)) + 1;
        for (int k = -span; k <= span; ++k) {
            const int i = ((hint + k) % n + n) % n;
            const float d2 = (x - r->x[i]) * (x - r->x[i]) + (z - r->z[i]) * (z - r->z[i]);
            if (d2 < bd) { bd = d2; best = i; }
        }
    } else {
        for (int i = 0; i < n; ++i) {
            const float d2 = (x - r->x[i]) * (x - r->x[i]) + (z - r->z[i]) * (z - r->z[i]);
            if (d2 < bd) { bd = d2; best = i; }
        }
    }
    if (best >= 0) {
        if (dist_cm) *dist_cm = sqrtf(bd);
        if (lateral_cm) {
            const float hd = r->heading_deg[best] / RAD_TO_DEG;
            const float rx = cosf(hd), rz = -sinf(hd);
            *lateral_cm = (x - r->x[best]) * rx + (z - r->z[best]) * rz;
        }
    }
    return best;
}

int race_line_advance(const race_line_t *r, int idx, float ahead_cm)
{
    if (!r || !r->valid || idx < 0) return -1;
    const int n = r->n;
    float left = ahead_cm;
    int i = idx % n;
    while (left > 0.0f) {
        if (left < r->h[i]) break;
        left -= r->h[i];
        i = (i + 1) % n;
    }
    return i;
}

void race_line_point(const race_line_t *r, int idx, float ahead_cm, float *x, float *z, float *heading_deg)
{
    if (!r || !r->valid || idx < 0) return;
    const int n = r->n;
    float left = ahead_cm > 0.0f ? ahead_cm : 0.0f;
    int i = idx % n;
    for (int guard = 0; guard < 2 * n && left >= r->h[i]; ++guard) {
        left -= r->h[i];
        i = (i + 1) % n;
    }
    const int j = (i + 1) % n;
    const float t = left / r->h[i];
    if (x) *x = r->x[i] + t * (r->x[j] - r->x[i]);
    if (z) *z = r->z[i] + t * (r->z[j] - r->z[i]);
    if (heading_deg) *heading_deg = atan2f(r->x[j] - r->x[i], r->z[j] - r->z[i]) * RAD_TO_DEG;
}

float race_line_speed(const race_line_t *r, int idx, float ahead_cm)
{
    if (!r || !r->valid || idx < 0) return 0.0f;
    const int i = race_line_advance(r, idx, ahead_cm);
    return i >= 0 ? r->v[i] : 0.0f;
}

float race_line_corner_factor(const race_line_t *r, float s_cm, float before_cm, float after_cm)
{
    if (!r || !r->valid) return 1.0f;
    float f = 1.0f;
    for (int k = 0; k < r->n; ++k) {
        if (!r->corner[k]) continue;
        const float d = cyc(r, r->s[k] - s_cm);          /* + : la esquina esta por delante */
        if (d >= 0.0f && before_cm > 0.0f && d < before_cm && d / before_cm < f) f = d / before_cm;
        if (d < 0.0f && after_cm > 0.0f && -d < after_cm && -d / after_cm < f) f = -d / after_cm;
    }
    return f;
}

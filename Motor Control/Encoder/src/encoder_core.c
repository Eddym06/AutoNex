/*
 * encoder_core.c - ver encoder_core.h y README.md
 */
#include "encoder_core.h"
#include <math.h>
#include <string.h>

#define ENC_TWO_PI 6.28318530717958647692f
#define ENC_ALL60  ((((uint64_t)1) << ENC_EDGES) - 1u)

/* ------------------------------------------------------------------ */
/* Utilidades                                                         */
/* ------------------------------------------------------------------ */
static int32_t floordiv_i32(int32_t a, int32_t b)
{
    int32_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
    return q;
}

static int32_t mod_i32(int32_t a, int32_t b)
{
    return a - floordiv_i32(a, b) * b;
}

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float median_f(float *v, int n)
{
    for (int i = 1; i < n; i++) {                 /* ordenamiento por inserción */
        float k = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
        v[j + 1] = k;
    }
    return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

void enc_config_default(enc_config_t *c)
{
    c->wheel_radius_m  = 0.0325f;
    c->min_interval_us = 400;
    c->stop_timeout_us = 150000;
    c->accel_max       = 12.0f;
    c->accel_alpha     = 0.75f;
    c->hold_frac       = 0.5f;
    c->overdue_tol     = 0.25f;
    c->align_cost_max  = 0.0030f;
    c->align_ratio_min = 2.5f;
}

/* ------------------------------------------------------------------ */
/* Tabla de posiciones                                                */
/* ------------------------------------------------------------------ */
static void build_cum(float *cum, const float *frac)
{
    cum[0] = 0.0f;
    for (int i = 1; i <= ENC_EDGES; i++)
        cum[i] = cum[i - 1] + frac[i % ENC_EDGES];   /* intervalo que TERMINA en el flanco i */
    const float s = cum[ENC_EDGES];
    for (int i = 0; i <= ENC_EDGES; i++) cum[i] /= s;
    cum[ENC_EDGES] = 1.0f;
}

/* Posición (m) del flanco n, relativa al primer flanco. */
static float edge_pos(const enc_t *e, int32_t n)
{
    const float *cum = e->aligned ? e->cum_c : e->cum_u;
    const int32_t ph = e->aligned ? e->phase : 0;
    const int32_t m   = n + ph;
    const int32_t rev = floordiv_i32(m, ENC_EDGES);
    const int32_t idx = m - rev * ENC_EDGES;
    return (float)rev * e->circ + (cum[idx] - cum[ph]) * e->circ;
}

/* Longitud del intervalo entre el flanco n y su vecino en la dirección dir. */
static float interval_len(const enc_t *e, int32_t n, int dir)
{
    return fabsf(edge_pos(e, n + dir) - edge_pos(e, n));
}

/* ------------------------------------------------------------------ */
/* Inicialización                                                     */
/* ------------------------------------------------------------------ */
void enc_init(enc_t *e, const enc_config_t *cfg)
{
    memset(e, 0, sizeof *e);
    if (cfg) e->cfg = *cfg; else enc_config_default(&e->cfg);
    e->circ = ENC_TWO_PI * e->cfg.wheel_radius_m;
    float uni[ENC_EDGES];
    for (int i = 0; i < ENC_EDGES; i++) uni[i] = 1.0f / ENC_EDGES;
    build_cum(e->cum_u, uni);
    memcpy(e->cum_c, e->cum_u, sizeof e->cum_c);
    for (int i = 0; i < ENC_EDGES; i++) e->frac_c[i] = uni[i];
}

void enc_reset(enc_t *e)
{
    memset(&e->d, 0, sizeof e->d);
    e->snap_valid = false;
    e->glitches = e->missed = e->accepted = 0;
}

void enc_set_table(enc_t *e, const float frac[ENC_EDGES])
{
    memcpy(e->frac_c, frac, sizeof e->frac_c);
    build_cum(e->cum_c, e->frac_c);
    e->cal_valid = true;
    e->aligned   = false;
    e->phase     = 0;
}

void enc_set_phase(enc_t *e, int32_t phase)
{
    if (!e->cal_valid) return;
    e->phase   = mod_i32(phase, ENC_EDGES);
    e->aligned = true;
}

/* ------------------------------------------------------------------ */
/* Proceso de un flanco                                               */
/* ------------------------------------------------------------------ */
bool enc_on_edge(enc_t *e, uint64_t t_us, int level, int dir)
{
    enc_dyn_t *d = &e->d;
    level = level ? 1 : 0;
    dir   = dir >= 0 ? 1 : -1;

    if (!d->started) {                       /* primer flanco: define el cero */
        memset(d, 0, sizeof *d);
        d->started    = true;
        d->t_last_us  = t_us;
        d->level_last = (uint8_t)level;
        d->dir        = (int8_t)dir;
        e->snap_valid = false;
        e->last_steps = 0;
        e->last_ivl_ok = false;
        e->last_dt_us = 0;
        e->accepted++;
        return true;
    }

    const uint64_t dt_us = (t_us > d->t_last_us) ? (t_us - d->t_last_us) : 0;

    /* Filtro de glitches: dos eventos tan juntos no pueden ser flancos reales.
     * Se cancela también el evento previo (si era la mitad de un par espurio). */
    if (dt_us < e->cfg.min_interval_us) {
        e->glitches++;
        if (e->snap_valid) { *d = e->snap; e->snap_valid = false; }
        return false;
    }

    e->snap = *d;
    e->snap_valid = true;

    const bool flipped = (dir != d->dir);
    const bool parity_ok = (level != d->level_last);
    int steps;
    if (flipped) steps = parity_ok ? 0 : 1;  /* reversa: se vuelve a cruzar el mismo flanco */
    else         steps = parity_ok ? 1 : 2;  /* nivel repetido: se perdió un flanco          */
    if (!parity_ok) e->missed++;

    const float s_old = edge_pos(e, d->n);
    d->n += dir * steps;
    const float s_new = edge_pos(e, d->n);
    const float ds    = fabsf(s_new - s_old);
    const float dt_s  = (float)dt_us * 1e-6f;

    const bool restart = flipped || steps == 0 || dt_us > e->cfg.stop_timeout_us;

    d->dir        = (int8_t)dir;
    d->level_last = (uint8_t)level;
    d->t_last_us  = t_us;
    e->last_steps = steps;
    e->last_dt_us = (uint32_t)(dt_us > 0xFFFFFFFFu ? 0xFFFFFFFFu : dt_us);
    e->last_ivl_ok = (!restart && steps == 1);
    e->accepted++;

    if (restart) {                            /* arranque desde parada o reversa */
        d->motion_valid = false;
        d->v_valid      = false;
        d->v = 0.0f;
        d->a = 0.0f;
        d->ivl_count = 0;
        return true;
    }

    const float vbar = ds / dt_s;             /* velocidad media del intervalo */
    float a_raw = 0.0f;
    if (d->v_valid) {
        /* diferencia entre puntos medios de los dos intervalos */
        a_raw = (vbar - d->vbar_prev) / (0.5f * (dt_s + d->dt_prev_s));
        a_raw = clampf(a_raw, -e->cfg.accel_max, e->cfg.accel_max);
        d->a += e->cfg.accel_alpha * (a_raw - d->a);
    } else {
        d->a = 0.0f;
    }
    d->v = vbar + 0.5f * d->a * dt_s;         /* velocidad al final del intervalo */
    if (d->v < 0.0f) d->v = 0.0f;
    d->vbar_prev    = vbar;
    d->dt_prev_s    = dt_s;
    d->v_valid      = true;
    d->motion_valid = true;

    if (steps == 1) {                         /* anillo de intervalos para la huella */
        d->ivl[d->ivl_head] = dt_s;
        d->ivl_head = (uint8_t)((d->ivl_head + 1) % ENC_ALIGN_WINDOW);
        if (d->ivl_count < ENC_ALIGN_WINDOW) d->ivl_count++;
    } else {
        d->ivl_count = 0;
    }

    if (e->cal_valid && !e->aligned) enc_align_try(e);
    return true;
}

/* ------------------------------------------------------------------ */
/* Estimación en cualquier instante                                   */
/* ------------------------------------------------------------------ */
void enc_estimate(const enc_t *e, uint64_t t_now_us, enc_est_t *o)
{
    const enc_dyn_t *d = &e->d;
    memset(o, 0, sizeof *o);
    if (!d->started) return;

    const int   dir = d->dir;
    const float s_k = edge_pos(e, d->n);
    const float D   = interval_len(e, d->n, dir);     /* hasta el siguiente flanco */
    const uint64_t tau_us = (t_now_us > d->t_last_us) ? (t_now_us - d->t_last_us) : 0;
    const float tau = (float)tau_us * 1e-6f;

    float delta = 0.0f, vnow = 0.0f, anow = 0.0f;
    bool moving = false;

    bool model_stop = false;
    if (d->motion_valid && e->d.a < -1e-3f && tau >= d->v / (-e->d.a)) model_stop = true;

    if (d->motion_valid && (tau_us < e->cfg.stop_timeout_us || model_stop)) {
        const float v = d->v, a = d->a;
        float te = tau;
        if (a < -1e-3f) {                              /* frenando: se detiene en v/|a| */
            const float ts = v / (-a);
            if (te > ts) te = ts;
        }
        delta = v * te + 0.5f * a * te * te;
        vnow  = v + a * te;
        if (vnow < 0.0f) vnow = 0.0f;
        anow  = a;
        moving = true;

        if (delta > D) {
            /* El modelo dice que ya debió llegar el flanco y no llegó: el
             * robot frena más de lo previsto. Se acerca al punto de reposo. */
            float w = ((delta - D) / D) / e->cfg.overdue_tol;
            if (w > 1.0f) w = 1.0f;
            const float dmax = 0.98f * D;
            delta = dmax + (e->cfg.hold_frac * D - dmax) * w;
            vnow *= (1.0f - w);
            anow = 0.0f;
        } else if (delta > 0.98f * D) {
            delta = 0.98f * D;
        }
    } else {
        /* Sin velocidad conocida (arranque desde reposo, reversa) o flanco muy
         * atrasado sin frenado previsto: la posición dentro del intervalo es
         * incierta. Se estaciona en el punto medio: error máximo D/2. */
        delta = e->cfg.hold_frac * D;
    }

    if (dir > 0) { o->pos_lo_m = s_k;     o->pos_hi_m = s_k + D; }
    else         { o->pos_lo_m = s_k - D; o->pos_hi_m = s_k;     }

    o->pos_m   = s_k + (float)dir * delta;
    o->vel_mps = (float)dir * vnow;
    o->acc_mps2 = (float)dir * anow;
    o->moving  = moving && vnow > 1e-4f;
    o->edges   = d->n;
}

/* ------------------------------------------------------------------ */
/* Alineación por huella                                              */
/* ------------------------------------------------------------------ */
bool enc_align_try(enc_t *e)
{
    if (!e->cal_valid || e->aligned) return e->aligned;
    const enc_dyn_t *d = &e->d;
    if (d->ivl_count < ENC_ALIGN_WINDOW) return false;

    enum { W = ENC_ALIGN_WINDOW };
    float obs[W];
    for (int k = 0; k < W; k++) obs[k] = d->ivl[(d->ivl_head + k) % W];   /* k=0: el más viejo */

    const float xm = 0.5f * (W - 1);
    float sxx = 0.0f;
    for (int k = 0; k < W; k++) sxx += (k - xm) * (k - xm);

    float best = 1e30f, second = 1e30f;
    int   bp = -1;
    for (int p = 0; p < ENC_EDGES; p++) {
        float r[W], mean = 0.0f, sxr = 0.0f;
        for (int k = 0; k < W; k++) {
            const int phys = mod_i32(d->n - (W - 1) + k + p, ENC_EDGES);
            r[k] = obs[k] / e->frac_c[phys];
            mean += r[k];
        }
        mean /= W;
        for (int k = 0; k < W; k++) sxr += (k - xm) * r[k];
        const float beta = sxr / sxx;               /* tendencia lineal = aceleración */
        float res = 0.0f;
        for (int k = 0; k < W; k++) {
            const float q = r[k] - mean - beta * (k - xm);
            res += q * q;
        }
        const float cost = res / (W * mean * mean);
        if (cost < best)        { second = best; best = cost; bp = p; }
        else if (cost < second) { second = cost; }
    }
    if (bp >= 0 && best < e->cfg.align_cost_max && second > best * e->cfg.align_ratio_min) {
        e->phase   = bp;
        e->aligned = true;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Calibración                                                        */
/* ------------------------------------------------------------------ */
void enc_cal_begin(enc_cal_t *c)
{
    memset(c, 0, sizeof *c);
}

void enc_cal_feed(enc_cal_t *c, int32_t n, uint32_t dt_us, bool valid)
{
    if (n < 1) return;
    const int32_t j = n - 1;
    const int32_t r = j / ENC_EDGES;
    const int32_t idx = n % ENC_EDGES;               /* intervalo que termina en el flanco n */
    if (r >= ENC_CAL_MAX_REVS) return;
    c->dt_us[r][idx] = dt_us;
    if (valid) c->valid[r] |=  ((uint64_t)1 << idx);
    else       c->valid[r] &= ~((uint64_t)1 << idx);
}

void enc_cal_feed_from(enc_cal_t *c, const enc_t *e)
{
    enc_cal_feed(c, e->d.n, e->last_dt_us, e->last_ivl_ok);
}

/* Analiza el error de posición de la tabla (acumulado - ideal) por armónicos y
 * elimina los armónicos 1..hc. Deja suma(frac) = 1 y el flanco 0 como referencia. */
static void filter_harmonics(float frac[ENC_EDGES], int hc, float harm_deg[8])
{
    float cum[ENC_EDGES], err[ENC_EDGES];
    cum[0] = 0.0f;
    for (int i = 1; i < ENC_EDGES; i++) cum[i] = cum[i - 1] + frac[i];
    for (int i = 0; i < ENC_EDGES; i++) err[i] = cum[i] - (float)i / (float)ENC_EDGES;

    float ac[8], bs[8];
    for (int h = 1; h <= 8; h++) {
        float a = 0.0f, b = 0.0f;
        for (int i = 0; i < ENC_EDGES; i++) {
            const float ang = ENC_TWO_PI * (float)(h * i) / (float)ENC_EDGES;
            a += err[i] * cosf(ang);
            b += err[i] * sinf(ang);
        }
        ac[h - 1] = 2.0f * a / (float)ENC_EDGES;
        bs[h - 1] = 2.0f * b / (float)ENC_EDGES;
        if (harm_deg) harm_deg[h - 1] = 360.0f * sqrtf(ac[h - 1] * ac[h - 1] + bs[h - 1] * bs[h - 1]);
    }
    if (hc > 8) hc = 8;
    for (int h = 1; h <= hc; h++)
        for (int i = 0; i < ENC_EDGES; i++) {
            const float ang = ENC_TWO_PI * (float)(h * i) / (float)ENC_EDGES;
            err[i] -= ac[h - 1] * cosf(ang) + bs[h - 1] * sinf(ang);
        }
    float c2[ENC_EDGES];
    for (int i = 0; i < ENC_EDGES; i++) c2[i] = (float)i / (float)ENC_EDGES + err[i] - err[0];
    for (int i = 1; i < ENC_EDGES; i++) frac[i] = c2[i] - c2[i - 1];
    frac[0] = 1.0f - c2[ENC_EDGES - 1];
}

bool enc_cal_finish(const enc_cal_t *c, float frac_out[ENC_EDGES], enc_cal_report_t *rep, int harm_cut)
{
    enc_cal_report_t R;
    memset(&R, 0, sizeof R);

    int   rv[ENC_CAL_MAX_REVS];
    float T[ENC_CAL_MAX_REVS], Ts[ENC_CAL_MAX_REVS];
    int   nc = 0;
    for (int r = ENC_CAL_SKIP_REVS; r < ENC_CAL_MAX_REVS; r++) {
        if (c->valid[r] != ENC_ALL60) continue;
        double s = 0.0;
        for (int i = 0; i < ENC_EDGES; i++) s += c->dt_us[r][i];
        rv[nc] = r;
        T[nc]  = (float)s;
        nc++;
    }
    R.revs_complete = nc;
    if (nc < 5) { if (rep) *rep = R; return false; }

    memcpy(Ts, T, sizeof(float) * (size_t)nc);
    const float Tmed = median_f(Ts, nc);

    int use[ENC_CAL_MAX_REVS], nu = 0;               /* descarta vueltas con tiempo atípico */
    for (int k = 0; k < nc; k++)
        if (fabsf(T[k] - Tmed) <= 0.02f * Tmed) use[nu++] = k;
    R.revs_used  = nu;
    R.rev_time_s = Tmed * 1e-6f;
    if (nu < 5) { if (rep) *rep = R; return false; }

    float f[ENC_EDGES], sum = 0.0f, sp_sum = 0.0f, sp_max = 0.0f;
    float tmp[ENC_CAL_MAX_REVS];
    for (int i = 0; i < ENC_EDGES; i++) {
        float mean = 0.0f;
        for (int u = 0; u < nu; u++) {
            const int k = use[u];
            tmp[u] = (float)c->dt_us[rv[k]][i] / T[k];   /* fracción de ESA vuelta */
            mean += tmp[u];
        }
        mean /= nu;
        float var = 0.0f;
        for (int u = 0; u < nu; u++) var += (tmp[u] - mean) * (tmp[u] - mean);
        const float sd = sqrtf(var / (nu - 1)) / mean;
        sp_sum += sd;
        if (sd > sp_max) sp_max = sd;
        f[i] = median_f(tmp, nu);
        sum += f[i];
    }
    R.ratio_min = 1e9f; R.ratio_max = 0.0f;
    for (int i = 0; i < ENC_EDGES; i++) {
        frac_out[i] = f[i] / sum;
        const float q = frac_out[i] * ENC_EDGES;
        if (q < R.ratio_min) R.ratio_min = q;
        if (q > R.ratio_max) R.ratio_max = q;
    }
    R.spread_mean = sp_sum / ENC_EDGES;
    R.spread_max  = sp_max;
    filter_harmonics(frac_out, harm_cut, R.harm_deg);
    R.ratio_min = 1e9f; R.ratio_max = 0.0f;           /* tras filtrar */
    for (int i = 0; i < ENC_EDGES; i++) {
        const float q = frac_out[i] * ENC_EDGES;
        if (q < R.ratio_min) R.ratio_min = q;
        if (q > R.ratio_max) R.ratio_max = q;
    }
    R.ok = (R.spread_max < 0.06f);
    if (rep) *rep = R;
    return R.ok;
}

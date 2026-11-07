/*
 * encoder_sim.c - Simulador de planta + banco de pruebas del estimador
 *
 * Compilar (PC):   gcc -O2 -std=c99 -o encoder_sim encoder_sim.c ../src/encoder_core.c -lm
 * Ejecutar:        ./encoder_sim            (todo, ~1-2 min)
 *                  ./encoder_sim quick      (versión rápida)
 *
 * Qué modela:
 *   - Disco impreso: cada flanco tiene un error de posición aleatorio (sigma),
 *     sesgo de ancho de ranura (ranuras más anchas/angostas), excentricidad.
 *   - Rizado de velocidad del motorreductor (ripple) sincronizado con el ángulo.
 *   - Retardo distinto en flanco de subida y de bajada, jitter de la ISR,
 *     timestamps cuantizados a 1 us.
 *   - Opcional: glitches (pares espurios) y flancos perdidos.
 *   - Perfiles de movimiento con aceleración, frenado brusco y reversa.
 * El estimador evaluado es el MISMO encoder_core.c (float) que va al ESP32-P4.
 * La verdad (posición real de la rueda) se integra en double cada 5 us.
 */
#include "../src/encoder_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_D   3.14159265358979323846
#define NE     60
#define DT_S   5e-6
#define DT_US  5

/* ------------------------------------------------------------------ */
/* RNG                                                                */
/* ------------------------------------------------------------------ */
typedef struct { uint64_t s; } rng_t;
static uint64_t rng64(rng_t *r)
{
    uint64_t x = r->s;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    r->s = x;
    return x * 2685821657736338717ULL;
}
static double rnd_u(rng_t *r) { return (double)(rng64(r) >> 11) * (1.0 / 9007199254740992.0); }
static double rnd_n(rng_t *r)
{
    double u1 = rnd_u(r), u2 = rnd_u(r);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI_D * u2);
}

/* ------------------------------------------------------------------ */
/* Planta                                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    double sigma_mm;       /* error aleatorio de posición de cada flanco (mm en el disco) */
    double bias_mm;        /* sesgo de ancho de ranura: sube +bias, baja -bias            */
    double ecc_mm;         /* excentricidad del disco                                     */
    double ripple_scale;   /* 1.0 = ripple típico del TT                                  */
    double tau_r_us, tau_f_us, jit_us;
    double glitch_hz, miss_prob;
} plant_params_t;

typedef struct {
    double R;
    double theta[NE];
    double rip_amp[3], rip_h[3], rip_ph[3];
    plant_params_t pp;
} plant_t;

static void plant_make(plant_t *p, rng_t *r, const plant_params_t *pp, double wheel_R)
{
    const double r_mean = 9.0; /* mm, radio medio de las ranuras */
    p->pp = *pp;
    p->R  = wheel_R;
    double psi = 2.0 * PI_D * rnd_u(r);
    for (int j = 0; j < NE; j++) {
        double ang = 2.0 * PI_D * j / NE;
        double e_mm = pp->sigma_mm * rnd_n(r) + ((j % 2 == 0) ? pp->bias_mm : -pp->bias_mm)
                    + pp->ecc_mm * sin(ang + psi);
        p->theta[j] = ang + e_mm / r_mean;
    }
    const double amp[3] = {0.020, 0.015, 0.010};
    const double hh[3]  = {1.0, 3.0, 12.0};
    for (int h = 0; h < 3; h++) {
        p->rip_amp[h] = amp[h] * pp->ripple_scale;
        p->rip_h[h]   = hh[h];
        p->rip_ph[h]  = 2.0 * PI_D * rnd_u(r);
    }
}

static long floordiv_l(long a, long b) { long q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
static int  mod_i(long a, int b) { return (int)(a - floordiv_l(a, b) * b); }

static double phi_edge(const plant_t *p, long k)
{
    return 2.0 * PI_D * (double)floordiv_l(k, NE) + p->theta[mod_i(k, NE)];
}

/* ------------------------------------------------------------------ */
/* Perfil de movimiento                                               */
/* ------------------------------------------------------------------ */
typedef struct { double v_target, accel, hold_s; } seg_t;

typedef struct {
    const seg_t *seg; int nseg, cur;
    double hold_left, v;
} prof_t;

static void prof_init(prof_t *p, const seg_t *seg, int nseg)
{
    p->seg = seg; p->nseg = nseg; p->cur = 0; p->v = 0.0;
    p->hold_left = nseg ? seg[0].hold_s : 0.0;
}
static void prof_step(prof_t *p, double dt)
{
    if (p->cur >= p->nseg) return;
    const seg_t *s = &p->seg[p->cur];
    if (p->v < s->v_target) {
        p->v += s->accel * dt;
        if (p->v >= s->v_target) p->v = s->v_target;
    } else if (p->v > s->v_target) {
        p->v -= s->accel * dt;
        if (p->v <= s->v_target) p->v = s->v_target;
    } else {
        p->hold_left -= dt;
        if (p->hold_left <= 0.0) {
            p->cur++;
            if (p->cur < p->nseg) p->hold_left = p->seg[p->cur].hold_s;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Motor de simulación                                                */
/* ------------------------------------------------------------------ */
typedef struct {
    void (*on_event)(void *ctx, uint64_t t_us, int level, int dir, int phys, int is_true, double phi_e);
    void (*on_sample)(void *ctx, uint64_t t_us, double phi, double vcmd);
    void *ctx;
} sim_cb_t;

typedef struct { uint64_t stamp; int level, dir, phys, is_true; double phi_e; } pend_t;

#define T0_US 5000000ULL

static void sim_run(const plant_t *pl, rng_t *rng, const seg_t *seg, int nseg, double rest_s,
                    int inject, double phi0, const sim_cb_t *cb)
{
    prof_t pr; prof_init(&pr, seg, nseg);
    double phi = phi0;
    long k = (long)floor(phi / (2.0 * PI_D / NE)) - 1;
    while (phi_edge(pl, k + 1) <= phi) k++;
    while (phi_edge(pl, k) > phi) k--;

    pend_t q[64]; int nq = 0;
    uint64_t now = T0_US, last_stamp = 0;
    double rest_left = rest_s;
    int last_dir = 1;
    long step = 0;

    while (1) {
        prof_step(&pr, DT_S);
        if (pr.cur >= pr.nseg) { rest_left -= DT_S; if (rest_left <= 0.0) break; }

        double rip = 1.0;
        for (int h = 0; h < 3; h++) rip += pl->rip_amp[h] * sin(pl->rip_h[h] * phi + pl->rip_ph[h]);
        const double w = pr.v / pl->R * rip;
        const double phi_old = phi;
        phi += w * DT_S;
        now += DT_US; step++;
        const uint64_t t_start_us = now - DT_US;

        /* cruces de flancos */
        while (phi >= phi_edge(pl, k + 1)) {
            long j = k + 1;
            double pe = phi_edge(pl, j);
            double frac = (phi > phi_old) ? (pe - phi_old) / (phi - phi_old) : 1.0;
            int level = (mod_i(j, 2) == 0) ? 1 : 0;                  /* intervalo j */
            double delay = (level ? pl->pp.tau_r_us : pl->pp.tau_f_us) + pl->pp.jit_us * rnd_u(rng);
            double tc = (double)t_start_us + frac * DT_US;
            uint64_t stamp = (uint64_t)llround(tc + delay);
            if (stamp <= last_stamp) stamp = last_stamp + 1;
            if (stamp < now) stamp = now;
            last_stamp = stamp; last_dir = 1;
            if (!(inject && rnd_u(rng) < pl->pp.miss_prob) && nq < 60) {
                q[nq++] = (pend_t){stamp, level, 1, mod_i(j, NE), 1, pe};
            }
            k = j;
        }
        while (phi < phi_edge(pl, k)) {
            long j = k;
            double pe = phi_edge(pl, j);
            double frac = (phi_old != phi) ? (phi_old - pe) / (phi_old - phi) : 1.0;
            int level = (mod_i(j - 1, 2) == 0) ? 1 : 0;              /* intervalo j-1 */
            double delay = (level ? pl->pp.tau_r_us : pl->pp.tau_f_us) + pl->pp.jit_us * rnd_u(rng);
            double tc = (double)t_start_us + frac * DT_US;
            uint64_t stamp = (uint64_t)llround(tc + delay);
            if (stamp <= last_stamp) stamp = last_stamp + 1;
            if (stamp < now) stamp = now;
            last_stamp = stamp; last_dir = -1;
            if (!(inject && rnd_u(rng) < pl->pp.miss_prob) && nq < 60) {
                q[nq++] = (pend_t){stamp, level, -1, mod_i(j, NE), 1, pe};
            }
            k = j - 1;
        }

        /* glitches (par espurio) */
        if (inject && pl->pp.glitch_hz > 0.0 && rnd_u(rng) < pl->pp.glitch_hz * DT_S && nq < 58) {
            int cur = (mod_i(k, 2) == 0) ? 1 : 0;
            uint64_t s1 = now > last_stamp ? now : last_stamp + 1;
            uint64_t gap = 10 + (uint64_t)(rnd_u(rng) * 70.0);
            q[nq++] = (pend_t){s1,       !cur, last_dir, 0, 0, 0.0};
            q[nq++] = (pend_t){s1 + gap, cur,  last_dir, 0, 0, 0.0};
            last_stamp = s1 + gap;
        }

        /* entrega de eventos en orden de timestamp */
        while (1) {
            int bi = -1;
            for (int i = 0; i < nq; i++) if (q[i].stamp <= now && (bi < 0 || q[i].stamp < q[bi].stamp)) bi = i;
            if (bi < 0) break;
            pend_t ev = q[bi]; q[bi] = q[--nq];
            if (cb->on_event) cb->on_event(cb->ctx, ev.stamp, ev.level, ev.dir, ev.phys, ev.is_true, ev.phi_e);
        }

        if (step % 200 == 0 && cb->on_sample) cb->on_sample(cb->ctx, now, phi, pr.v);
    }
}

/* ------------------------------------------------------------------ */
/* Estadísticas                                                       */
/* ------------------------------------------------------------------ */
#define HBINS 2000          /* 0.01 mm por bin hasta 20 mm */
typedef struct {
    long n; double sum, sum2, maxabs; long hist[HBINS + 1];
} stats_t;

static void st_add(stats_t *s, double e_mm)
{
    double a = fabs(e_mm);
    s->n++; s->sum += e_mm; s->sum2 += e_mm * e_mm;
    if (a > s->maxabs) s->maxabs = a;
    long b = (long)(a * 100.0); if (b > HBINS) b = HBINS;
    s->hist[b]++;
}
static void st_merge(stats_t *d, const stats_t *s)
{
    d->n += s->n; d->sum += s->sum; d->sum2 += s->sum2;
    if (s->maxabs > d->maxabs) d->maxabs = s->maxabs;
    for (int i = 0; i <= HBINS; i++) d->hist[i] += s->hist[i];
}
static double st_rms(const stats_t *s)  { return s->n ? sqrt(s->sum2 / (double)s->n) : 0.0; }
static double st_mean(const stats_t *s) { return s->n ? s->sum / (double)s->n : 0.0; }
static double st_pct(const stats_t *s, double pct)
{
    if (!s->n) return 0.0;
    long target = (long)ceil(pct * (double)s->n), acc = 0;
    for (int i = 0; i <= HBINS; i++) { acc += s->hist[i]; if (acc >= target) return (i + 1) * 0.01; }
    return 20.0;
}

/* ------------------------------------------------------------------ */
/* Estimadores bajo prueba                                            */
/* ------------------------------------------------------------------ */
typedef enum { K_COUNT, K_ORIG, K_CORE } kind_t;

typedef struct {
    const char *name;
    kind_t kind;
    float  alpha;          /* K_CORE */
    int    tab;            /* -1 uniforme; 0..8 calibrada (corte de armonicos); 99 oracle (geometria real) */
    int    auto_align;     /* K_CORE: fase desconocida    */
    float  hold_frac;
    int    skip_reversal;  /* no soporta reversa          */
} spec_t;

typedef struct {
    const spec_t *spec;
    /* conteo / original */
    int started; long n; uint64_t t_last; double S_edge, v_last, a, dt_last;
    int ph; double Ci[NE];
    /* núcleo */
    enc_t core;
    stats_t move, rest;
} est_t;

typedef struct {
    est_t *e; int ne;
    double R; int p0_cal; const float *frac_cal;
    int first_done; double phi_ref; int p0; uint64_t t_first;
    double rest_timer; double meas_from_s; uint64_t t_ref;
    int reversal;
    double final_err_mm[16]; /* se llena al final con el último error por estimador */
    double max_err_run[16];
} harness_t;

static void h_on_event(void *ctx, uint64_t t_us, int level, int dir, int phys, int is_true, double phi_e)
{
    harness_t *h = ctx;
    if (!h->first_done) {
        if (!is_true) return;
        h->first_done = 1; h->phi_ref = phi_e; h->p0 = phys; h->t_first = t_us;
        for (int i = 0; i < h->ne; i++) {
            est_t *x = &h->e[i];
            x->ph = mod_i((long)phys - h->p0_cal, NE);
            if (x->spec->kind == K_CORE && x->spec->tab >= 0 && !x->spec->auto_align)
                enc_set_phase(&x->core, x->ph);
        }
    }
    for (int i = 0; i < h->ne; i++) {
        est_t *x = &h->e[i];
        const spec_t *s = x->spec;
        if (s->kind == K_CORE) {
            enc_on_edge(&x->core, t_us, level, dir);
        } else if (s->kind == K_COUNT) {
            if (!x->started) { x->started = 1; x->n = 0; }
            else x->n += dir;
            x->v_last = dir;            /* se guarda la dirección para centrar */
            x->t_last = t_us;
        } else { /* K_ORIG: algoritmo original (solo adelante) */
            const double dth = 2.0 * PI_D / NE;
            if (!x->started) {
                x->started = 1; x->n = 0; x->t_last = t_us; x->S_edge = 0.0; x->v_last = 0.0; x->a = 0.0; x->dt_last = 0.0;
            } else {
                double dt = (double)(t_us - x->t_last) * 1e-6;
                x->n++;
                int idx = mod_i(x->n + x->ph, NE);
                double dtc = dt * x->Ci[idx];
                double w = dth / dtc;
                double v = w * h->R;
                x->a = (x->dt_last > 0.0) ? (v - x->v_last) / dtc : 0.0;
                x->v_last = v; x->dt_last = dt;
                x->S_edge += h->R * dth;
                x->t_last = t_us;
            }
        }
    }
}

static double est_pos(const harness_t *h, est_t *x, uint64_t t_us)
{
    const spec_t *s = x->spec;
    const double pitch = 2.0 * PI_D * h->R / NE;
    if (s->kind == K_CORE) {
        enc_est_t o; enc_estimate(&x->core, t_us, &o); return (double)o.pos_m;
    } else if (s->kind == K_COUNT) {
        return ((double)x->n + 0.5 * x->v_last) * pitch;     /* centro del intervalo en curso */
    } else {
        double tau = (double)(t_us - x->t_last) * 1e-6;
        double cap = 1.5 * x->dt_last;
        if (x->dt_last > 0.0 && tau > cap) tau = cap;
        return x->S_edge + x->v_last * tau + 0.5 * x->a * tau * tau;
    }
}

static void h_on_sample(void *ctx, uint64_t t_us, double phi, double vcmd)
{
    harness_t *h = ctx;
    if (!h->first_done) return;
    if (fabs(vcmd) < 1e-9) h->rest_timer += 1e-3; else h->rest_timer = 0.0;
    double t_s = (double)(t_us - T0_US) * 1e-6;
    double truth = (phi - h->phi_ref) * h->R;
    for (int i = 0; i < h->ne; i++) {
        est_t *x = &h->e[i];
        if (x->spec->kind == K_ORIG && h->reversal) continue;
        double err = (est_pos(h, x, t_us) - truth) * 1000.0;
        h->final_err_mm[i] = err;
        if (t_s < h->meas_from_s) continue;
        if (fabs(err) > h->max_err_run[i]) h->max_err_run[i] = fabs(err);
        if (h->rest_timer >= 0.25) st_add(&x->rest, err);
        else                       st_add(&x->move, err);
    }
}

/* ------------------------------------------------------------------ */
/* Calibración en la simulación                                       */
/* ------------------------------------------------------------------ */
typedef struct { enc_t e; enc_cal_t cal; int first; int p0; } calctx_t;

static void cal_on_event(void *ctx, uint64_t t_us, int level, int dir, int phys, int is_true, double phi_e)
{
    (void)phi_e;
    calctx_t *c = ctx;
    if (!is_true) return;
    if (!c->first) { c->first = 1; c->p0 = phys; }
    if (enc_on_edge(&c->e, t_us, level, dir)) enc_cal_feed_from(&c->cal, &c->e);
}

static calctx_t g_c;

static int sim_calibrate(const plant_t *pl, rng_t *rng, float frac[NE], enc_cal_report_t *rep, double v_cal)
{
    calctx_t *cp = &g_c;
    memset(cp, 0, sizeof *cp);
    enc_config_t cfg; enc_config_default(&cfg); cfg.wheel_radius_m = (float)pl->R;
    enc_init(&cp->e, &cfg);
    enc_cal_begin(&cp->cal);
    seg_t seg[1] = {{v_cal, 1.0, 24.0 * 2.0 * PI_D * pl->R / v_cal + 1.0}};
    sim_cb_t cb = {cal_on_event, NULL, cp};
    sim_run(pl, rng, seg, 1, 0.0, 0, 2.0 * PI_D * rnd_u(rng), &cb);
    int ok = enc_cal_finish(&cp->cal, frac, rep, 0);
    return ok ? cp->p0 : -1;
}

/* Tabla segun el modo: calibrada con corte de armonicos, u oracle (geometria real) */
static void sim_table(const plant_t *pl, int tab, int p0_cal, float frac[NE])
{
    if (tab == 99) {
        for (int t = 0; t < NE; t++) {
            int j = mod_i((long)t + p0_cal, NE), jm = mod_i((long)j - 1, NE);
            double d = pl->theta[j] - pl->theta[jm];
            if (j == 0) d += 2.0 * PI_D;
            frac[t] = (float)(d / (2.0 * PI_D));
        }
    } else {
        enc_cal_report_t rep;
        enc_cal_finish(&g_c.cal, frac, &rep, tab);
    }
}

/* ------------------------------------------------------------------ */
/* Evaluación de un escenario sobre varios discos                     */
/* ------------------------------------------------------------------ */
typedef struct {
    const char *name;
    const seg_t *seg; int nseg;
    double rest_s, meas_from_s;
    int inject, reversal;
    int trials_per_disc;
} scen_t;

typedef struct {
    stats_t move[16], rest[16];
    double worst_final[16];       /* peor |error| al final del escenario */
    double max_run[16];
    int    cal_fail, discs;
    double spread_mean_sum, spread_max_sum, rmin_sum, rmax_sum;
    double harm_sum[8];
} result_t;

static void eval_scenario(const plant_params_t *pp, const scen_t *sc, int ndiscs, uint64_t seed,
                          const spec_t *specs, int ns, result_t *res, double wheel_R)
{
    memset(res, 0, sizeof *res);
    rng_t rng = {seed ? seed : 88172645463325252ULL};
    for (int d = 0; d < ndiscs; d++) {
        plant_t pl; plant_make(&pl, &rng, pp, wheel_R);
        float frac[NE]; enc_cal_report_t rep;
        int p0_cal = sim_calibrate(&pl, &rng, frac, &rep, 0.25);
        if (p0_cal < 0) { res->cal_fail++; continue; }
        res->discs++;
        res->spread_mean_sum += rep.spread_mean; res->spread_max_sum += rep.spread_max;
        res->rmin_sum += rep.ratio_min; res->rmax_sum += rep.ratio_max;
        for (int hh = 0; hh < 8; hh++) res->harm_sum[hh] += rep.harm_deg[hh];

        for (int tr = 0; tr < sc->trials_per_disc; tr++) {
            static est_t ests[16];
            memset(ests, 0, sizeof ests);
            harness_t h; memset(&h, 0, sizeof h);
            h.e = ests; h.ne = ns; h.R = wheel_R; h.p0_cal = p0_cal; h.frac_cal = frac;
            h.meas_from_s = sc->meas_from_s; h.reversal = sc->reversal;
            for (int i = 0; i < ns; i++) {
                est_t *x = &ests[i]; x->spec = &specs[i];
                if (specs[i].kind == K_CORE) {
                    enc_config_t cfg; enc_config_default(&cfg);
                    cfg.wheel_radius_m = (float)wheel_R; cfg.accel_alpha = specs[i].alpha;
                    cfg.hold_frac = specs[i].hold_frac;
                    enc_init(&x->core, &cfg);
                    if (specs[i].tab >= 0) {
                        float tf[NE]; sim_table(&pl, specs[i].tab, p0_cal, tf);
                        enc_set_table(&x->core, tf);
                    }
                } else if (specs[i].kind == K_ORIG) {
                    float tf[NE]; sim_table(&pl, 0, p0_cal, tf);       /* su C_i sale de la calibracion cruda */
                    for (int j = 0; j < NE; j++) x->Ci[j] = (1.0 / NE) / (double)tf[j];   /* C_i = dt_prom/dt_i */
                }
            }
            sim_cb_t cb = {h_on_event, h_on_sample, &h};
            sim_run(&pl, &rng, sc->seg, sc->nseg, sc->rest_s, sc->inject, 2.0 * PI_D * rnd_u(&rng), &cb);
            for (int i = 0; i < ns; i++) {
                if (specs[i].kind == K_ORIG && sc->reversal) continue;
                st_merge(&res->move[i], &ests[i].move);
                st_merge(&res->rest[i], &ests[i].rest);
                double fe = fabs(h.final_err_mm[i]);
                if (fe > res->worst_final[i]) res->worst_final[i] = fe;
                if (h.max_err_run[i] > res->max_run[i]) res->max_run[i] = h.max_err_run[i];
            }
        }
    }
}

static void print_result(const char *title, const spec_t *specs, int ns, const result_t *r, int show_rest)
{
    printf("\n=== %s ===  (discos: %d, calibraciones fallidas: %d)\n", title, r->discs, r->cal_fail);
    printf("%-44s | %7s %7s %7s %7s", "estimador", "RMS", "media", "p99", "max");
    if (show_rest) printf(" | %7s %7s %7s", "RMSrep", "maxrep", "final*");
    printf("   [mm]\n");
    for (int i = 0; i < ns; i++) {
        if (!r->move[i].n && !r->rest[i].n) { printf("%-44s |   (no aplica)\n", specs[i].name); continue; }
        printf("%-44s | %7.3f %7.3f %7.3f %7.3f", specs[i].name,
               st_rms(&r->move[i]), st_mean(&r->move[i]), st_pct(&r->move[i], 0.99), r->move[i].maxabs);
        if (show_rest) printf(" | %7.3f %7.3f %7.3f", st_rms(&r->rest[i]), r->rest[i].maxabs, r->worst_final[i]);
        printf("\n");
    }
}

/* ------------------------------------------------------------------ */
/* Prueba de alineación automática                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    enc_t e; int p0_cal, phys0, first, locked; long edges_at_lock;
} alctx_t;

static void al_on_event(void *ctx, uint64_t t_us, int level, int dir, int phys, int is_true, double phi_e)
{
    (void)phi_e;
    alctx_t *a = ctx;
    if (!is_true) return;
    if (!a->first) { a->first = 1; a->phys0 = phys; }
    int was = a->e.aligned;
    enc_on_edge(&a->e, t_us, level, dir);
    if (!was && a->e.aligned) { a->locked = 1; a->edges_at_lock = a->e.d.n; }
}

static void test_alignment(const plant_params_t *pp, double v_cruise, double accel, int ntr, uint64_t seed, double wheel_R)
{
    rng_t rng = {seed};
    int locked = 0, right = 0, wrong = 0; double sum_edges = 0.0; long max_edges = 0;
    for (int t = 0; t < ntr; t++) {
        plant_t pl; plant_make(&pl, &rng, pp, wheel_R);
        float frac[NE]; enc_cal_report_t rep;
        int p0_cal = sim_calibrate(&pl, &rng, frac, &rep, 0.25);
        if (p0_cal < 0) continue;
        static alctx_t a; memset(&a, 0, sizeof a);
        enc_config_t cfg; enc_config_default(&cfg); cfg.wheel_radius_m = (float)wheel_R;
        enc_init(&a.e, &cfg); enc_set_table(&a.e, frac);
        seg_t seg[2] = {{v_cruise, accel, 3.0}, {0.0, 3.0, 0.2}};
        sim_cb_t cb = {al_on_event, NULL, &a};
        sim_run(&pl, &rng, seg, 2, 0.1, 0, 2.0 * PI_D * rnd_u(&rng), &cb);
        if (a.locked) {
            locked++;
            int truth = mod_i((long)a.phys0 - p0_cal, NE);
            if (a.e.phase == truth) right++; else wrong++;
            sum_edges += (double)a.edges_at_lock; if (a.edges_at_lock > max_edges) max_edges = a.edges_at_lock;
        }
    }
    printf("  v=%.2f m/s, acel=%.1f m/s^2: fase correcta %d/%d, falsa %d, sin cerrar %d, flancos hasta cerrar: media %.0f, max %ld\n",
           v_cruise, accel, right, ntr, wrong, ntr - locked, locked ? sum_edges / locked : 0.0, max_edges);
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    const int quick = (argc > 1 && strcmp(argv[1], "quick") == 0);
    const int ND = quick ? 4 : 20;
    const double R = 0.0325;

    plant_params_t typ = {0.08, 0.04, 0.06, 1.0, 20.0, 10.0, 3.0, 0.0, 0.0};

    spec_t specs[] = {
        {"0 Solo conteo de flancos (centrado)",         K_COUNT, 0,     -1, 0, 0.5f, 0},
        {"1 Algoritmo original (MRUA + Ci)",            K_ORIG,  0,      0, 0, 0.5f, 1},
        {"2 Nuevo, sin calibrar (tabla uniforme)",      K_CORE, 0.75f,  -1, 0, 0.5f, 0},
        {"3 Nuevo, calibracion cruda (sin filtro)",     K_CORE, 0.75f,   0, 0, 0.5f, 0},
        {"4 NUEVO POR DEFECTO (calib. + filtro 1-3)",   K_CORE, 0.75f,   3, 0, 0.5f, 0},
        {"5 Idem 4 pero sin termino de aceleracion",    K_CORE, 0.00f,   3, 0, 0.5f, 0},
        {"6 Idem 4, fase desconocida (auto-alinea)",    K_CORE, 0.75f,   3, 1, 0.5f, 0},
        {"7 Limite: geometria real del disco conocida", K_CORE, 0.75f,  99, 0, 0.5f, 0},
    };
    const int NS = (int)(sizeof specs / sizeof specs[0]);

    printf("Simulador de encoder: disco 30 ranuras (60 flancos), rueda R=%.1f mm, paso nominal %.3f mm/flanco\n",
           R * 1000.0, 2.0 * PI_D * R * 1000.0 / NE);
    printf("Disco 'tipico': sigma=%.2f mm, sesgo=%.2f mm, excentricidad=%.2f mm, ripple x%.1f, retardo sensor %.0f/%.0f us, jitter %.0f us\n",
           typ.sigma_mm, typ.bias_mm, typ.ecc_mm, typ.ripple_scale, typ.tau_r_us, typ.tau_f_us, typ.jit_us);

    result_t *res = malloc(sizeof *res);
    if (!res) return 1;


    if (argc > 1 && strcmp(argv[1], "explore") == 0) {
        spec_t ex[] = {
            {"uniforme",            K_CORE, 0.5f, -1, 0, 0.5f, 0},
            {"calibrada corte 0",   K_CORE, 0.5f, 0, 0, 0.5f, 0},
            {"calibrada corte 1",   K_CORE, 0.5f, 1, 0, 0.5f, 0},
            {"calibrada corte 2",   K_CORE, 0.5f, 2, 0, 0.5f, 0},
            {"calibrada corte 3",   K_CORE, 0.5f, 3, 0, 0.5f, 0},
            {"calibrada corte 4",   K_CORE, 0.5f, 4, 0, 0.5f, 0},
            {"calibrada corte 6",   K_CORE, 0.5f, 6, 0, 0.5f, 0},
            {"calibrada corte 8",   K_CORE, 0.5f, 8, 0, 0.5f, 0},
            {"oracle",              K_CORE, 0.5f, 99, 0, 0.5f, 0},
        };
        const int NX = (int)(sizeof ex / sizeof ex[0]);
        seg_t cr[1] = {{0.30, 2.0, 5.0}};
        scen_t s1 = {"crucero", cr, 1, 0.0, 1.0, 0, 0, 1};
        seg_t ms[] = {{0.5, 2.0, 1.0}, {0, 4.0, 0.5}, {0.3, 1.5, 1.0}, {0, 3.0, 0.5}, {0.65, 2.0, 0.8}, {0, 5.0, 0.6}, {0.15, 1.0, 1.0}, {0, 2.0, 0.6}};
        scen_t s2 = {"mision", ms, 8, 0.3, 0.0, 0, 0, 1};
        const double rs[4] = {0.5, 1.0, 2.0, 4.0};
        result_t *r1 = malloc(sizeof *r1);
        for (int q = 0; q < 4; q++) {
            plant_params_t pq = typ; pq.ripple_scale = rs[q];
            printf("\n##### ripple x%.1f  (1x:%.1f%%  3x:%.1f%%  12x:%.1f%%)\n", rs[q], 2.0*rs[q], 1.5*rs[q], 1.0*rs[q]);
            eval_scenario(&pq, &s1, 10, 8000 + (uint64_t)q, ex, NX, res, R);
            eval_scenario(&pq, &s2, 10, 8100 + (uint64_t)q, ex, NX, r1, R);
            printf("armonicos de la tabla cruda (grados de eje, h=1..8): ");
            for (int hh = 0; hh < 8; hh++) printf("%.2f ", res->harm_sum[hh] / (res->discs ? res->discs : 1));
            printf("\n%-22s | crucero RMS  max | mision mov RMS  max | mision reposo RMS  max\n", "tabla");
            for (int i = 0; i < NX; i++)
                printf("%-22s |   %6.3f %6.3f |      %6.3f %6.3f |         %6.3f %6.3f\n", ex[i].name,
                       st_rms(&res->move[i]), res->move[i].maxabs, st_rms(&r1->move[i]), r1->move[i].maxabs,
                       st_rms(&r1->rest[i]), r1->rest[i].maxabs);
        }
        free(r1); free(res);
        return 0;
    }


    if (argc > 1 && strcmp(argv[1], "explore2") == 0) {
        spec_t ex[] = {
            {"alpha 0.00 (v const)", K_CORE, 0.00f, 3, 0, 0.5f, 0},
            {"alpha 0.25",           K_CORE, 0.25f, 3, 0, 0.5f, 0},
            {"alpha 0.50",           K_CORE, 0.50f, 3, 0, 0.5f, 0},
            {"alpha 0.75",           K_CORE, 0.75f, 3, 0, 0.5f, 0},
            {"alpha 1.00 (sin filtro)", K_CORE, 1.00f, 3, 0, 0.5f, 0},
            {"alpha .5 hold 0.35",   K_CORE, 0.50f, 3, 0, 0.35f, 0},
            {"alpha .5 hold 0.65",   K_CORE, 0.50f, 3, 0, 0.65f, 0},
        };
        const int NX = (int)(sizeof ex / sizeof ex[0]);
        seg_t ms[] = {{0.5, 2.0, 1.0}, {0, 4.0, 0.5}, {0.3, 1.5, 1.0}, {0, 3.0, 0.5}, {0.65, 2.0, 0.8}, {0, 5.0, 0.6}, {0.15, 1.0, 1.0}, {0, 2.0, 0.6}};
        scen_t s2 = {"mision", ms, 8, 0.3, 0.0, 0, 0, 1};
        seg_t br[] = {{0.4, 3.0, 0.6}, {0, 15.0, 0.4}};
        scen_t s3 = {"brusco", br, 2, 0.2, 0.0, 0, 0, 10};
        seg_t hard[] = {{0.6, 4.0, 0.6}, {0.15, 4.0, 0.6}, {0.6, 4.0, 0.4}, {0, 4.0, 0.5}};  /* aceleraciones fuertes */
        scen_t s4 = {"fuerte", hard, 4, 0.2, 0.0, 0, 0, 1};
        result_t *r1 = malloc(sizeof *r1), *r2 = malloc(sizeof *r2);
        eval_scenario(&typ, &s2, 15, 9000, ex, NX, res, R);
        eval_scenario(&typ, &s3, 15, 9001, ex, NX, r1, R);
        eval_scenario(&typ, &s4, 15, 9002, ex, NX, r2, R);
        printf("\n%-24s | mision mov RMS  max | mision rep RMS max | brusco rep RMS max | acel 4m/s2 mov RMS max\n", "config");
        for (int i = 0; i < NX; i++)
            printf("%-24s |    %6.3f %6.3f |   %6.3f %6.3f   |  %6.3f %6.3f    |    %6.3f %6.3f\n", ex[i].name,
                   st_rms(&res->move[i]), res->move[i].maxabs, st_rms(&res->rest[i]), res->rest[i].maxabs,
                   st_rms(&r1->rest[i]), r1->rest[i].maxabs, st_rms(&r2->move[i]), r2->move[i].maxabs);
        free(r1); free(r2); free(res);
        return 0;
    }

    /* A-C: crucero a 3 velocidades */
    const double vs[3] = {0.05, 0.30, 0.65};
    for (int i = 0; i < 3; i++) {
        seg_t seg[1] = {{vs[i], 2.0, 5.0}};
        scen_t sc = {"crucero", seg, 1, 0.0, 1.0, 0, 0, 1};
        char title[96]; snprintf(title, sizeof title, "Crucero a %.2f m/s (error en movimiento)", vs[i]);
        eval_scenario(&typ, &sc, ND, 1000 + (uint64_t)i, specs, NS, res, R);
        print_result(title, specs, NS, res, 0);
    }
    /* D: arranque-parada */
    seg_t mission[] = {{0.5, 2.0, 1.0}, {0, 4.0, 0.5}, {0.3, 1.5, 1.0}, {0, 3.0, 0.5},
                       {0.65, 2.0, 0.8}, {0, 5.0, 0.6}, {0.15, 1.0, 1.0}, {0, 2.0, 0.6}};
    {
        scen_t sc = {"mision", mission, 8, 0.3, 0.0, 0, 0, 1};
        eval_scenario(&typ, &sc, ND, 2000, specs, NS, res, R);
        print_result("Arranque-parada con aceleraciones de 1-5 m/s^2 (movimiento + reposo)", specs, NS, res, 1);
        printf("   * final = peor error absoluto al terminar la mision\n");
    }
    /* E: frenado brusco, parada en punto aleatorio */
    seg_t brusco[] = {{0.4, 3.0, 0.6}, {0, 15.0, 0.4}};
    {
        scen_t sc = {"brusco", brusco, 2, 0.2, 0.0, 0, 0, quick ? 5 : 15};
        eval_scenario(&typ, &sc, ND, 3000, specs, NS, res, R);
        print_result("Frenado brusco (15 m/s^2) en puntos aleatorios", specs, NS, res, 1);
    }
    /* F: reversa */
    seg_t rev[] = {{0.3, 2.0, 1.0}, {0, 3.0, 0.4}, {-0.3, 2.0, 1.0}, {0, 3.0, 0.4}, {0.3, 2.0, 0.8}, {0, 3.0, 0.4}};
    {
        scen_t sc = {"reversa", rev, 6, 0.2, 0.0, 0, 1, 1};
        eval_scenario(&typ, &sc, ND, 4000, specs, NS, res, R);
        print_result("Avance-reversa-avance (direccion conocida)", specs, NS, res, 1);
    }
    /* G: recorrido largo, limpio y con fallas */
    {
        seg_t lng[] = {{0.4, 1.0, quick ? 40.0 : 250.0}, {0, 2.0, 0.5}};
        scen_t sc = {"largo", lng, 2, 0.5, 0.0, 0, 0, 1};
        eval_scenario(&typ, &sc, quick ? 2 : 3, 5000, specs, NS, res, R);
        char title[128]; snprintf(title, sizeof title, "Recorrido largo (%.0f m), sin fallas", quick ? 16.0 : 100.0);
        print_result(title, specs, NS, res, 1);
        plant_params_t flt = typ; flt.glitch_hz = 2.0; flt.miss_prob = 5e-4;
        scen_t sc2 = {"largo+fallas", lng, 2, 0.5, 0.0, 1, 0, 1};
        eval_scenario(&flt, &sc2, quick ? 2 : 3, 5001, specs, NS, res, R);
        snprintf(title, sizeof title, "Recorrido largo con 2 glitches/s y 0.05%% de flancos perdidos");
        print_result(title, specs, NS, res, 1);
    }
    /* H: sensibilidad a la calidad de impresión */
    {
        printf("\n=== Sensibilidad a la calidad de impresion (misión arranque-parada, RMS/max en mm) ===\n");
        printf("%-32s | %-26s | %-26s | %-26s\n", "sigma del flanco (mm en disco)", "conteo centrado", "original", "nuevo por defecto (4)");
        const double sg[4] = {0.02, 0.05, 0.10, 0.20};
        scen_t sc = {"mision", mission, 8, 0.3, 0.0, 0, 0, 1};
        for (int i = 0; i < 4; i++) {
            plant_params_t pq = typ; pq.sigma_mm = sg[i]; pq.bias_mm = sg[i] * 0.5; pq.ecc_mm = sg[i] * 0.75;
            eval_scenario(&pq, &sc, quick ? 3 : 10, 6000 + (uint64_t)i, specs, NS, res, R);
            printf("sigma=%.2f mm (borde %.0f um)    | %7.3f / %7.3f        | %7.3f / %7.3f        | %7.3f / %7.3f\n",
                   sg[i], sg[i] * 1000.0, st_rms(&res->move[0]), res->move[0].maxabs,
                   st_rms(&res->move[1]), res->move[1].maxabs, st_rms(&res->move[4]), res->move[4].maxabs);
        }
    }
    /* I: alineación */
    {
        printf("\n=== Alineacion automatica tabla<->disco (huella de intervalos), %d discos por caso ===\n", quick ? 20 : 100);
        const int nt = quick ? 20 : 100;
        test_alignment(&typ, 0.10, 1.0, nt, 7001, R);
        test_alignment(&typ, 0.30, 2.0, nt, 7002, R);
        test_alignment(&typ, 0.65, 2.0, nt, 7003, R);
        plant_params_t good = typ; good.sigma_mm = 0.03; good.bias_mm = 0.02; good.ecc_mm = 0.03;
        printf("  disco de buena calidad (sigma 0.03 mm):\n");
        test_alignment(&good, 0.30, 2.0, nt, 7004, R);
    }
    free(res);
    return 0;
}

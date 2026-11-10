/* Prueba de MIC contra un motor + coche simulados con la fisica del propio JGB37-520 (R, Ke, masa,
 * friccion, caida del L298N, encoder cuantizado), pero con parametros DISTINTOS a los que MIC conoce.
 * NO es el motor real.
 *   gcc -O2 -std=c11 -Wall -Wextra -I. mic.c test_mic.c -o test_mic -lm && ./test_mic */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mic.h"

typedef struct {
    float s, v;                 /* m, m/s (verdad) */
    float r, ke_lin, kf_lin, m, fc, b, vdrop, vbat, cm_per_count;
    float i_max_seen;
    float t_plug, t_brake, t_coast, t_drive;
} plant_t;

/* Misma estructura fisica que MIC, con desviaciones: `k` escala resistencia, masa, friccion y Ke. */
static plant_t make_plant(const mic_t *m, float k_r, float k_m, float k_f, float k_e, float vbat)
{
    plant_t p; memset(&p, 0, sizeof p);
    p.r = m->cfg.r_ohm * k_r;
    p.ke_lin = m->ke_lin * k_e;
    p.kf_lin = m->kf_lin * k_e;
    p.m = m->m_eff * k_m;
    p.fc = m->cfg.f_coulomb_n * k_f;
    p.b = m->cfg.b_viscous_ns_m * k_f;
    p.vdrop = m->cfg.vdrop_volts;
    p.vbat = vbat;
    p.cm_per_count = m->cm_per_count * 1.003f;   /* la rueda real mide 0,3 % distinto del que cree el firmware (calibrar el diametro) */
    return p;
}

static void plant_step(plant_t *p, const mic_out_t *o, float dt)
{
    const float vav = p->vbat - p->vdrop;
    const int sub = 4;
    const float h = dt / (float)sub;
    for (int i = 0; i < sub; ++i) {
        const float vm = o->brake_short ? 0.0f : o->duty * vav;
        float cur = (vm - p->ke_lin * p->v) / p->r;
        float f = p->kf_lin * cur;
        const float fr = p->fc * tanhf(p->v / 0.01f) + p->b * p->v;
        float net = f - fr;
        if (fabsf(p->v) < 1e-4f && fabsf(f) <= p->fc) net = 0.0f, p->v = 0.0f;   /* friccion estatica */
        const float vn = p->v + net / p->m * h;
        if (p->v != 0.0f && ((vn > 0) != (p->v > 0)) && fabsf(f) < p->fc) p->v = 0.0f; else p->v = vn;
        p->s += p->v * h;
        if (fabsf(cur) > p->i_max_seen) p->i_max_seen = fabsf(cur);
    }
}

static int counts_of(const plant_t *p) { return (int)floorf(p->s * 100.0f / p->cm_per_count); }

static int g_fail;
#define CHECK(cond, ...) do { if (!(cond)) { ++g_fail; printf("  FALLA: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct { float t, s_end, v_max, v_at[4], min_v, i_max; float t_phase[5]; bool done, emergency; } run_t;

static run_t run(mic_t *m, plant_t *p, float t_max, float watch[4], int nwatch, float v_at_watch[4])
{
    run_t r; memset(&r, 0, sizeof r);
    r.min_v = 1e9f;
    mic_in_t in; mic_out_t o;
    float t = 0.0f;
    const float dt = 0.001f;
    int w = 0;
    while (t < t_max) {
        in.counts = counts_of(p); in.vbat = p->vbat;
        mic_update(m, &in, dt, &o);
        plant_step(p, &o, dt);
        t += dt;
        if (p->v > r.v_max) r.v_max = p->v;
        if (p->v < r.min_v) r.min_v = p->v;
        r.t_phase[o.phase] += dt;
        if (o.emergency) r.emergency = true;
        while (w < nwatch && p->s * 100.0f >= watch[w]) { v_at_watch[w] = p->v * 100.0f; ++w; }
        if (o.done && fabsf(p->v) < 0.005f) { r.done = true; break; }
    }
    r.t = t; r.s_end = p->s * 100.0f; r.i_max = p->i_max_seen;
    return r;
}

int main(void)
{
    mic_cfg_t cfg; mic_cfg_jgb37_520_1000rpm(&cfg);
    mic_t proto; mic_init(&proto, &cfg);
    printf("modelo JGB37-520 1000 rpm: Ke=%.4f V s/rad  ke_lin=%.2f V/(m/s)  kf=%.2f N/A  M_eff=%.2f kg  %.4f cm/cuenta\n",
           proto.ke_motor, proto.ke_lin, proto.kf_lin, proto.m_eff, proto.cm_per_count);
    printf("capacidades: acel(0)=%.0f  acel(100 cm/s)=%.0f cm/s2 | frenado progresivo(100)=%.0f  agresivo(100)=%.0f cm/s2\n",
           mic_max_accel_cm_s2(&proto, 0), mic_max_accel_cm_s2(&proto, 100),
           mic_max_decel_cm_s2(&proto, 100, MIC_DECEL_PROGRESSIVE), mic_max_decel_cm_s2(&proto, 100, MIC_DECEL_AGGRESSIVE));
    printf("distancia de parada desde 100 cm/s: progresiva %.1f cm, agresiva %.1f cm\n",
           mic_stop_distance_cm(&proto, 100, MIC_DECEL_PROGRESSIVE), mic_stop_distance_cm(&proto, 100, MIC_DECEL_AGGRESSIVE));

    printf("\n== A) recta rapida -> curva -> aproximacion al parqueo (ruta con anticipacion)\n");
    {
        const struct { const char *name; float kr, km, kf, ke, vbat; } cases[3] = {
            {"modelo exacto  ", 1.00f, 1.00f, 1.00f, 1.00f, 11.1f},
            {"R+15 M+30 F+40 ", 1.15f, 1.30f, 1.40f, 0.97f, 10.3f},
            {"R-10 M-15 F-30 ", 0.90f, 0.85f, 0.70f, 1.03f, 12.2f},
        };
        for (int c = 0; c < 3; ++c) {
            mic_t m; mic_init(&m, &cfg);
            plant_t p = make_plant(&m, cases[c].kr, cases[c].km, cases[c].kf, cases[c].ke, cases[c].vbat);
            mic_route_begin(&m);
            mic_route_add(&m, 200.0f, 110.0f, 28.0f);   /* recta a 110 cm/s, entrar a la curva a <=28 */
            mic_route_add(&m, 45.0f, 28.0f, 12.0f);     /* curva a 28, salir a <=12 */
            mic_route_add(&m, 55.0f, 12.0f, 0.0f);      /* aproximacion al parqueo */
            mic_route_commit(&m, 0.0f, MIC_DECEL_PROGRESSIVE);
            float watch[4] = {196.0f, 199.0f, 244.0f, 249.0f}, vw[4] = {0};
            run_t r = run(&m, &p, 14.0f, watch, 4, vw);
            printf("  %s fin=%.2f cm (meta 300)  t=%.2f s  vmax=%.0f  v@196=%.0f v@244=%.0f cm/s  imax=%.2f A  fases: acel %.2f rueda %.2f freno %.2f activo %.2f%s\n",
                   cases[c].name, r.s_end, r.t, r.v_max * 100.0f, vw[0], vw[2], r.i_max,
                   r.t_phase[MIC_PHASE_DRIVE], r.t_phase[MIC_PHASE_COAST], r.t_phase[MIC_PHASE_BRAKE], r.t_phase[MIC_PHASE_PLUG],
                   r.emergency ? " EMERGENCIA" : "");
            CHECK(r.done, "no termino la ruta (%s)", cases[c].name);
            CHECK(fabsf(r.s_end - 300.0f) < 1.4f, "error de parada %.2f cm (%s)", r.s_end - 300.0f, cases[c].name);
            CHECK(vw[1] < 28.0f + 6.0f, "entro a la curva a %.0f cm/s (limite 28; el 0,3 %% de rueda mal calibrada son 0,9 cm) (%s)", vw[1], cases[c].name);
            CHECK(vw[3] < 12.0f + 4.0f, "entro a la aproximacion a %.0f cm/s (limite 12) (%s)", vw[3], cases[c].name);
            CHECK(r.i_max < cfg.i_max_a * 1.25f, "corriente %.2f A supera el limite del puente (%s)", r.i_max, cases[c].name);
            CHECK(r.min_v > -0.01f, "retrocedio (%s)", cases[c].name);
            CHECK(r.t_phase[MIC_PHASE_PLUG] < 0.15f, "uso demasiado frenado activo (%.2f s) (%s)", r.t_phase[MIC_PHASE_PLUG], cases[c].name);
        }
        /* Referencia ingenua: perfil trapezoidal conservador (a=40 cm/s2, v=28 cm/s) para la misma ruta */
        const float t_naive = 28.0f / 40.0f + (300.0f - 2.0f * 0.5f * 28.0f * 28.0f / 40.0f) / 28.0f + 28.0f / 40.0f;
        printf("  (referencia: rampas a 28 cm/s y 40 cm/s2 tardarian ~%.1f s)\n", t_naive);
    }

    printf("\n== B) avances cortos exactos (arcos de parqueo)\n");
    {
        const float dists[] = {3.5f, 7.0f, 12.0f, 25.0f, -3.5f, -12.0f};
        for (unsigned i = 0; i < sizeof dists / sizeof dists[0]; ++i) {
            mic_t m; mic_init(&m, &cfg);
            plant_t p = make_plant(&m, 1.1f, 1.2f, 1.3f, 0.98f, 10.6f);
            mic_request_move(&m, dists[i], 14.0f, 0.0f, MIC_DECEL_PROGRESSIVE);
            float dummy[4] = {0}, vw[4] = {0};
            run_t r = run(&m, &p, 6.0f, dummy, 0, vw);
            const float err = r.s_end - dists[i];
            printf("  avance %6.1f cm: error %+.2f cm  t=%.2f s  vmax=%.1f vmin=%.1f\n", dists[i], err, r.t, r.v_max * 100, r.min_v * 100);
            CHECK(r.done, "avance %.1f no termino", dists[i]);
            CHECK(fabsf(err) < 0.4f, "error %.2f cm en avance %.1f", err, dists[i]);
        }
    }

    printf("\n== C) parada desde 100 cm/s: progresiva frente a agresiva\n");
    {
        float dist[2], vminv[2];
        for (int md = 0; md < 2; ++md) {
            mic_t m; mic_init(&m, &cfg);
            plant_t p = make_plant(&m, 1.0f, 1.0f, 1.0f, 1.0f, 11.1f);
            mic_request_speed(&m, 100.0f, MIC_DECEL_PROGRESSIVE);
            mic_in_t in; mic_out_t o;
            for (int i = 0; i < 1500; ++i) { in.counts = counts_of(&p); in.vbat = p.vbat; mic_update(&m, &in, 0.001f, &o); plant_step(&p, &o, 0.001f); }
            const float v0 = p.v * 100.0f, s0 = p.s;
            mic_request_stop(&m, md ? MIC_DECEL_AGGRESSIVE : MIC_DECEL_PROGRESSIVE);
            float tstop = 0.0f; vminv[md] = 1e9f;
            for (int i = 0; i < 3000; ++i) {
                in.counts = counts_of(&p); in.vbat = p.vbat; mic_update(&m, &in, 0.001f, &o); plant_step(&p, &o, 0.001f);
                if (p.v < vminv[md]) vminv[md] = p.v;
                if (tstop == 0.0f && fabsf(p.v) < 0.005f) tstop = (float)(i + 1) * 0.001f;
            }
            dist[md] = (p.s - s0) * 100.0f;
            printf("  %-11s desde %.0f cm/s: %.1f cm en %.2f s (vmin %.2f)\n", md ? "agresivo" : "progresivo", v0, dist[md], tstop, vminv[md] * 100.0f);
            CHECK(fabsf(v0 - 100.0f) < 3.0f, "no alcanzo 100 cm/s (%.1f)", v0);
            CHECK(tstop > 0.0f, "no se detuvo");
            CHECK(vminv[md] > -0.01f, "retrocedio");
        }
        CHECK(dist[1] < dist[0] * 0.8f, "agresivo no acorta la parada (%.1f vs %.1f)", dist[1], dist[0]);
    }

    printf("\n== D) mantener velocidad con perturbacion (subida: friccion x3 a mitad)\n");
    {
        mic_t m; mic_init(&m, &cfg);
        plant_t p = make_plant(&m, 1.0f, 1.0f, 1.0f, 1.0f, 11.1f);
        mic_request_speed(&m, 60.0f, MIC_DECEL_PROGRESSIVE);
        mic_in_t in; mic_out_t o; float worst = 0.0f;
        for (int i = 0; i < 4000; ++i) {
            if (i == 2000) { p.fc *= 3.0f; p.b *= 2.0f; }
            in.counts = counts_of(&p); in.vbat = p.vbat; mic_update(&m, &in, 0.001f, &o); plant_step(&p, &o, 0.001f);
            if (i > 3000) worst = fmaxf(worst, fabsf(p.v * 100.0f - 60.0f));
        }
        printf("  v final %.2f cm/s, error maximo tras recuperar %.2f cm/s\n", p.v * 100.0f, worst);
        CHECK(worst < 1.0f, "no recupera la velocidad tras la perturbacion (%.2f)", worst);
    }

    if (g_fail) { printf("\nFALLARON %d comprobaciones\n", g_fail); return 1; }
    printf("\nPASS: MIC\n");
    return 0;
}

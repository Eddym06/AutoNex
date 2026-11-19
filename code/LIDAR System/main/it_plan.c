#include "it_plan.h"

#include <math.h>
#include <string.h>
#ifdef IT_DEBUG
#include <stdio.h>
#endif

#ifndef IT_WIDTHS
#define IT_WIDTHS {2, 5, 10}
#endif
#ifndef IT_AMPS
#define IT_AMPS {3.0f, 1.5f, 0.6f}
#endif
#define IT_NW ((int)(sizeof((int[])IT_WIDTHS) / sizeof(int)))
#define IT_NA ((int)(sizeof((float[])IT_AMPS) / sizeof(float)))
#ifndef IT_STEER_USE_DEFAULT
#define IT_STEER_USE_DEFAULT 0.72f
#endif
/* Error de seguimiento que el plan reserva (cm) a la velocidad v: floor + err * (v/100)^exp, ademas del error de pose. Calibrado con el error REAL
 * de IT en la mision simulada (32 rondas, 5 letreros, sin los 2 s de enganche; tools/it_err_stats.js): p99 = 2,4 cm a 45 cm/s, 4,2 a 75, 5,3 a 105,
 * 6,4 a 125 y 7,4 a 145 (max 8,1): casi lineal en v, y 14 * (v/100)^2 da 29 cm a 145 cm/s (4 veces lo medido). AUN ASI el modelo medido
 * NO se usa: en 32 rondas con 5 letreros, piso 1 + 5*(v/100) dio 74,1 s (antes 73,9), vuelta 2 de 12,7 s (11,4) y 11 contactos en una ronda; 8 cm: 83,9 s,
 * 2 rondas sin acabar; 11 cm: 80,8 s; piso 2 + 8: 81,9 s. El plan baja de 7,8 a 6,6 s pero el seguidor no lo ejecuta: se queda en los valores de siempre. */
#ifndef IT_TRACK_ERR_DEFAULT
#define IT_TRACK_ERR_DEFAULT 14.0f
#endif
#ifndef IT_TRACK_FLOOR_DEFAULT
#define IT_TRACK_FLOOR_DEFAULT 0.0f
#endif
#ifndef IT_TRACK_EXP_DEFAULT
#define IT_TRACK_EXP_DEFAULT 2.0f
#endif
#ifndef IT_GAP_ERR_SCALE_SQ
#define IT_GAP_ERR_SCALE_SQ 1.0f   /* 1 = como antes (la huella usa el mismo margen); 4 = un cuarto del error en la huella */
#endif
#ifndef IT_SA_T0
#define IT_SA_T0 0.05f
#endif
#ifndef IT_SA_T1
#define IT_SA_T1 0.0005f
#endif
#ifndef IT_START_MASK
#define IT_START_MASK 63   /* arranques de la etapa 0 que se usan (bit q = MIX[q]) */
#endif
#ifndef IT_RELAX_ITER_DEFAULT
#define IT_RELAX_ITER_DEFAULT 300   /* 4000 y 300 dan el mismo plan (120 pistas: 7,65 y 7,64 s, 108 validas las dos) y 300 tarda la mitad. La ronda ccw 22 que fallaba con 300 era una adquisicion falsa del cajon (park_localize), no el plan */
#endif
#ifndef IT_STEER_RETRIES
#define IT_STEER_RETRIES 2
#endif
#ifndef IT_RAND_EVALS
#define IT_RAND_EVALS 0
#endif
#define G_CM 981.0f
#define PI_F 3.14159265358979323846f
#define RAD2DEG 57.29577951308232f
#define DEG2RAD 0.017453292519943295f


/* Matematica barata. hypotf y atan2f de newlib cuestan cientos de ciclos en el P4 (se llaman ~15 veces por estacion y evaluacion); sqrtf es una
 * instruccion. atanf de Cephes: error de ~1 ulp en float, sin diferencia apreciable con la de la libm. */
#define IT_HYP(a, b) sqrtf((a) * (a) + (b) * (b))
static float it_atanf(float x)
{
    float sgn = 1.0f, y = 0.0f;
    if (x < 0.0f) { x = -x; sgn = -1.0f; }
    if (x > 2.414213562373095f) { y = 1.5707963267948966f; x = -1.0f / x; }
    else if (x > 0.4142135623730950f) { y = 0.7853981633974483f; x = (x - 1.0f) / (x + 1.0f); }
    const float z = x * x;
    y += (((8.05374449538e-2f * z - 1.38776856032e-1f) * z + 1.99777106478e-1f) * z - 3.33329491539e-1f) * z * x + x;
    return sgn * y;
}
static float it_atan2f(float y, float x)
{
    if (x == 0.0f) return y > 0.0f ? 1.5707963267948966f : (y < 0.0f ? -1.5707963267948966f : 0.0f);
    float a = it_atanf(y / x);
    if (x < 0.0f) a += (y >= 0.0f ? PI_F : -PI_F);
    return a;
}

void it_config_default(it_config_t *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->outer_half_cm = 150.0f; c->inner_half_cm = 50.0f; c->corner_radius_cm = 40.0f;
    c->wheelbase_cm = 18.45f; c->half_width_cm = 6.5f; c->front_ext_cm = 22.2f; c->rear_ext_cm = 3.8f;
    c->steer_max_deg = 40.0f; c->steer_rate_dps = 420.0f; c->lag_s = 0.09f; c->steer_use = IT_STEER_USE_DEFAULT; c->rate_use = 0.45f; c->ctrl_mode = 0; c->k_psi = 0.6f; c->k_e = 2.0f;
    c->mu = 0.8f; c->rear_weight = 0.62f; c->cg_height_cm = 4.0f;
    c->grip = 0.7f; c->safety = 0.85f; c->a_motor_cm_s2 = 350.0f;
    c->v_max_cm_s = 150.0f; c->v_min_cm_s = 15.0f;
    c->wall_margin_cm = 1.0f; c->obs_margin_cm = 1.0f; c->obs_half_cm = 3.5f;     /* obs_half: radio circunscrito del letrero (5 cm de lado) */
    c->pose_err_cm = 0.8f; c->track_err_cm = IT_TRACK_ERR_DEFAULT; c->track_floor_cm = IT_TRACK_FLOOR_DEFAULT; c->track_exp = IT_TRACK_EXP_DEFAULT;     /* error lateral de seguimiento a 100 cm/s dentro de la mision (escaneo a 10 Hz + lazo rapido + pose del LiDAR): 6 en el banco ideal, 14 medido en la mision completa */
    c->curtain_cm = 100.0f; c->approach_cm = 0.0f;     /* el lado se exige a la altura del letrero; cambiar de lado antes es decision del optimizador */
    c->side_red = 1; c->side_green = -1;
    c->station_cm = 5.0f; c->passes = 3; c->sweeps = 3; c->relax_iterations = IT_RELAX_ITER_DEFAULT;
}

static bool in_range(float *v, float lo, float hi)
{
    bool ok = isfinite(*v);
    if (!ok) *v = lo;
    if (*v < lo) { *v = lo; ok = false; }
    if (*v > hi) { *v = hi; ok = false; }
    return ok;
}

bool it_config_sanitize(it_config_t *c)
{
    if (!c) return false;
    bool ok = true;
    ok &= in_range(&c->outer_half_cm, 60.0f, 500.0f);
    ok &= in_range(&c->inner_half_cm, 10.0f, c->outer_half_cm - 30.0f);
    ok &= in_range(&c->corner_radius_cm, 15.0f, 0.5f * (c->outer_half_cm + c->inner_half_cm) - 5.0f);
    ok &= in_range(&c->wheelbase_cm, 5.0f, 60.0f);
    ok &= in_range(&c->half_width_cm, 2.0f, 30.0f);
    ok &= in_range(&c->front_ext_cm, c->wheelbase_cm, 80.0f);
    ok &= in_range(&c->rear_ext_cm, 0.0f, 30.0f);
    ok &= in_range(&c->steer_max_deg, 10.0f, 60.0f);
    ok &= in_range(&c->steer_rate_dps, 60.0f, 2000.0f);
    ok &= in_range(&c->lag_s, 0.0f, 0.5f);
    ok &= in_range(&c->steer_use, 0.3f, 1.0f);
    ok &= in_range(&c->rate_use, 0.1f, 1.0f);
    ok &= in_range(&c->k_psi, 0.0f, 3.0f);
    ok &= in_range(&c->k_e, 0.0f, 10.0f);
    if (c->ctrl_mode < 0 || c->ctrl_mode > 1) { c->ctrl_mode = 0; ok = false; }
    ok &= in_range(&c->mu, 0.2f, 1.5f);
    ok &= in_range(&c->rear_weight, 0.3f, 0.9f);
    ok &= in_range(&c->cg_height_cm, 0.0f, 20.0f);
    ok &= in_range(&c->grip, 0.2f, 1.0f);
    ok &= in_range(&c->safety, 0.3f, 1.0f);
    ok &= in_range(&c->a_motor_cm_s2, 30.0f, 1500.0f);
    ok &= in_range(&c->v_max_cm_s, 10.0f, 400.0f);
    ok &= in_range(&c->v_min_cm_s, 5.0f, c->v_max_cm_s);
    ok &= in_range(&c->wall_margin_cm, 0.0f, 20.0f);
    ok &= in_range(&c->obs_margin_cm, 0.0f, 20.0f);
    ok &= in_range(&c->obs_half_cm, 1.0f, 15.0f);
    ok &= in_range(&c->pose_err_cm, 0.0f, 10.0f);
    ok &= in_range(&c->track_err_cm, 0.0f, 20.0f);
    ok &= in_range(&c->track_floor_cm, 0.0f, 10.0f);
    ok &= in_range(&c->track_exp, 0.5f, 3.0f);
    ok &= in_range(&c->curtain_cm, 20.0f, 300.0f);
    ok &= in_range(&c->approach_cm, 0.0f, 100.0f);
    ok &= in_range(&c->station_cm, 3.0f, 12.0f);
    if (c->side_red > 1) c->side_red = 1;
    if (c->side_red < -1) c->side_red = -1;
    if (c->side_green > 1) c->side_green = 1;
    if (c->side_green < -1) c->side_green = -1;
    if (c->passes < 1) { c->passes = 1; ok = false; }
    if (c->passes > 6) { c->passes = 6; ok = false; }
    if (c->sweeps < 0) { c->sweeps = 0; ok = false; }
    if (c->sweeps > 12) { c->sweeps = 12; ok = false; }
    if (c->relax_iterations < 50) { c->relax_iterations = 50; ok = false; }
    if (c->relax_iterations > 20000) { c->relax_iterations = 20000; ok = false; }
    return ok;
}

/* ------------------------------------------------------------ linea central (esquinas redondeadas) */

static void center_at(const it_config_t *c, float s, float *x, float *z, float *tx, float *tz, bool *arc)
{
    const float h = 0.5f * (c->outer_half_cm + c->inner_half_cm), rc = c->corner_radius_cm;
    const float len = 2.0f * (h - rc), al = 0.5f * PI_F * rc, period = 4.0f * (len + al);
    s = fmodf(s, period);
    if (s < 0.0f) s += period;
    int piece = (int)(s / (len + al));
    if (piece > 3) piece = 3;
    float t = s - (float)piece * (len + al);
    static const float DX[4] = {1.0f, 0.0f, -1.0f, 0.0f}, DZ[4] = {0.0f, 1.0f, 0.0f, -1.0f};
    const float sx[4] = {-(h - rc), h, h - rc, -h}, sz[4] = {-h, -(h - rc), h, h - rc};
    const float ax[4] = {h - rc, h - rc, -(h - rc), -(h - rc)}, az[4] = {-(h - rc), h - rc, h - rc, -(h - rc)};
    *arc = false;
    if (t <= len) {
        *x = sx[piece] + DX[piece] * t; *z = sz[piece] + DZ[piece] * t;
        *tx = DX[piece]; *tz = DZ[piece];
    } else {
        t -= len;
        const float th = (-0.5f + (float)piece * 0.5f) * PI_F + t / rc;
        *x = ax[piece] + rc * cosf(th); *z = az[piece] + rc * sinf(th);
        *tx = -sinf(th); *tz = cosf(th);
        *arc = true;
    }
}

/* ------------------------------------------------------------ espacio libre */

/* El chasis es un rectangulo (eje trasero como origen: `front_ext` por delante, `rear_ext` por detras, `half_width` a cada lado) con
 * el rumbo psi (rad, horario desde el norte). Tratarlo como un punto con semiancho falla en el apice de una esquina: con el rumbo a
 * 45 grados de la pared, el borde del chasis sobresale ~20 cm.
 * Devuelve la holgura (cm) hasta lo mas cercano: negativa = penetracion (cota inferior de la distancia de separacion por el mejor eje). */
static float footprint_gap(const it_config_t *c, float x, float z, float psi, const it_obstacle_t *obs, int count)
{
    const float s = sinf(psi), co = cosf(psi);
    const float w = c->half_width_cm, f = c->front_ext_cm, r = c->rear_ext_cm;
    const float U[4] = {f, f, -r, -r}, V[4] = {w, -w, w, -w};
    float minx = 1.0e9f, maxx = -1.0e9f, minz = 1.0e9f, maxz = -1.0e9f;
    for (int k = 0; k < 4; ++k) {
        const float cx = x + U[k] * s + V[k] * co, cz = z + U[k] * co - V[k] * s;      /* adelante (s, co); derecha (co, -s) */
        if (cx < minx) minx = cx;
        if (cx > maxx) maxx = cx;
        if (cz < minz) minz = cz;
        if (cz > maxz) maxz = cz;
    }
    float gap = c->outer_half_cm - fmaxf(fmaxf(maxx, -minx), fmaxf(maxz, -minz));
    /* Bloque interior: mejor eje de separacion entre sus dos ejes y los del chasis. */
    const float q = c->inner_half_cm;
    float g_in = fmaxf(fmaxf(minx - q, -q - maxx), fmaxf(minz - q, -q - maxz));
    if (g_in < 0.0f) {
        const float sx[4] = {-q, q, q, -q}, sz[4] = {-q, -q, q, q};
        float umin = 1.0e9f, umax = -1.0e9f, vmin = 1.0e9f, vmax = -1.0e9f;
        for (int k = 0; k < 4; ++k) {
            const float u = (sx[k] - x) * s + (sz[k] - z) * co, v = (sx[k] - x) * co - (sz[k] - z) * s;
            if (u < umin) umin = u;
            if (u > umax) umax = u;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
        g_in = fmaxf(g_in, fmaxf(fmaxf(umin - f, -r - umax), fmaxf(vmin - w, -w - vmax)));
    }
    if (g_in < gap) gap = g_in;
    const float rs = c->obs_half_cm;
    const float reach = IT_HYP(f > r ? f : r, w);    /* ningun punto del chasis esta mas lejos que esto del eje trasero */
    for (int j = 0; j < count; ++j) {
        const float dx = obs[j].x_cm - x, dz = obs[j].z_cm - z;
        /* Un obstaculo mas lejos que gap + alcance + radio no puede bajar el minimo: se salta sin cambiar el resultado. */
        const float lim = gap + reach + rs;
        if (lim > 0.0f && dx * dx + dz * dz > lim * lim) continue;
        const float u = dx * s + dz * co, v = dx * co - dz * s;
        const float du = fmaxf(fmaxf(-r - u, u - f), 0.0f), dv = fmaxf(fabsf(v) - w, 0.0f);
        const float dd = IT_HYP(du, dv) - rs;
        if (dd < gap) gap = dd;
    }
    return gap;
}

/* Corredor de punto (centro del eje trasero): robusto y barato. La huella real se penaliza aparte, en el coste. */
static bool free_at(const it_plan_t *p, float x, float z)
{
    const it_config_t *c = &p->cfg;
    const float rho = fmaxf(fabsf(x), fabsf(z)), m = c->half_width_cm + c->wall_margin_cm;
    if (rho > c->outer_half_cm - m || rho < c->inner_half_cm + m) return false;
    const float rs = c->obs_half_cm + c->half_width_cm + c->obs_margin_cm;
    for (int j = 0; j < p->obstacles; ++j) {
        const it_obstacle_t *g = &p->obs[j];
        if (hypotf(x - g->x_cm, z - g->z_cm) < rs) return false;
        const int side = g->color == 1 ? c->side_red : g->color == 2 ? c->side_green : 0;
        if (side == 0) continue;
        const float rx = p->obs_rx[j], rz = p->obs_rz[j];     /* derecha del avance en el letrero */
        const float fx = -(float)side * rx, fz = -(float)side * rz, tx = -rz, tz = rx;
        const float px = x - g->x_cm, pz = z - g->z_cm;
        const float lam = px * fx + pz * fz, ell = px * tx + pz * tz;
        if (lam <= c->curtain_cm && lam >= 0.0f && ell >= -(rs + c->approach_cm) && ell <= rs) return false;
    }
    return true;
}

/* ------------------------------------------------------------ perfil de velocidad y tiempo */

/* Evaluacion del coste de una trayectoria (desplazamientos d por estacion).
 *
 * Cada magnitud de una estacion depende solo de las vecinas: la posicion de la propia, h y el rumbo del tramo de la siguiente, la curvatura cruda de
 * las dos de al lado, la suavizada de las dos siguientes, y el limite de velocidad de las tres. Una "joroba" del optimizador mueve ~20 de 146
 * estaciones: recalcularlas todas costaba 1,5 ms en el P4 (36 us en un PC). `eval_inc` rehace solo la ventana afectada (+-3 estaciones) con LAS
 * MISMAS funciones por estacion que la evaluacion completa (mismos operandos, mismo orden de suma): el resultado es identico bit a bit (se comprueba con
 * IT_VERIFY_INCREMENTAL). Solo el barrido adelante-atras de velocidad y las sumas recorren todo el anillo (barato: 4 raices por estacion).
 *
 * Estado en cache = el de la ultima trayectoria evaluada o aceptada: p->{x,z,h,heading_deg,gap,kappa,kappa_s,room,vcap} y los estaticos g_*. */
static float g_ks[IT_MAX_N];     /* curvatura cruda (derecha positiva) */
static float g_seg[IT_MAX_N];    /* rumbo del tramo i -> i+1 */
static float g_dl[IT_MAX_N];     /* angulo de volante equivalente a la curvatura suavizada */
static float g_viol[IT_MAX_N];   /* exceso de curvatura sobre lo que el volante puede dar, por estacion */
static float g_vlim[IT_MAX_N];   /* limite de velocidad propio de la estacion (antes del barrido adelante-atras) */
static const it_plan_t *g_owner;   /* plan al que pertenece la cache de arriba */

typedef struct { float a_lim, a_full, cap_acc, cap_dec, kmax, rate; } prof_const_t;

static prof_const_t prof_const(const it_config_t *c)
{
    prof_const_t k;
    const float mu_e = c->mu * c->grip, hL = c->cg_height_cm / c->wheelbase_cm;
    k.a_lim = mu_e * G_CM * c->safety;
    k.a_full = mu_e * G_CM;
    k.cap_acc = fminf(c->a_motor_cm_s2, mu_e * G_CM * c->rear_weight / (1.0f - mu_e * hL));
    k.cap_dec = mu_e * G_CM * c->rear_weight / (1.0f + mu_e * hL);
    /* El plan no usa todo el volante ni toda la velocidad del servo: el resto es reserva para que el seguidor corrija. */
    k.kmax = tanf(c->steer_use * c->steer_max_deg * DEG2RAD) / c->wheelbase_cm;
    k.rate = c->rate_use * c->steer_rate_dps * DEG2RAD;
    return k;
}

static inline int wrap_i(int i, int n) { i %= n; return i < 0 ? i + n : i; }

static void st_xz(it_plan_t *p, const float *d, int i)
{
    p->x[i] = p->cx[i] + d[i] * p->nx[i];
    p->z[i] = p->cz[i] + d[i] * p->nz[i];
}
/* Longitud y rumbo del tramo i -> i+1. */
static void st_h_seg(it_plan_t *p, int i)
{
    const int j = (i + 1) % p->n;
    p->h[i] = IT_HYP(p->x[j] - p->x[i], p->z[j] - p->z[i]);
    if (p->h[i] < 0.01f) p->h[i] = 0.01f;
    g_seg[i] = it_atan2f(p->x[j] - p->x[i], p->z[j] - p->z[i]);
}
/* Curvatura cruda y rumbo de la estacion (circulo por tres puntos). */
static void st_ks_hdg(it_plan_t *p, int i)
{
    const int n = p->n, a = (i + n - 1) % n, b = (i + 1) % n;
    const float ux = p->x[i] - p->x[a], uz = p->z[i] - p->z[a], vx = p->x[b] - p->x[i], vz = p->z[b] - p->z[i];
    const float wx = p->x[b] - p->x[a], wz = p->z[b] - p->z[a];
    const float la = IT_HYP(ux, uz), lb = IT_HYP(vx, vz), lc = IT_HYP(wx, wz);
    const float cross = ux * vz - uz * vx;      /* > 0: giro a la izquierda (x este, z norte) */
    g_ks[i] = (la > 0.01f && lb > 0.01f && lc > 0.01f) ? -2.0f * cross / (la * lb * lc) : 0.0f;   /* derecha positiva */
    p->heading_deg[i] = it_atan2f(wx, wz) * RAD2DEG;
}
/* Holgura de la huella del chasis (con su rumbo) en la estacion: la que de verdad limita la velocidad cerca de una pared. */
static void st_gap(it_plan_t *p, int i)
{
    p->gap[i] = footprint_gap(&p->cfg, p->x[i], p->z[i], p->heading_deg[i] * DEG2RAD, p->obs, p->obstacles) - p->cfg.wall_margin_cm;
}
/* Curvatura suavizada: una sola pasada [1 2 1] / 4 (dos escondian las ondulaciones: el optimizador las aprovechaba) y el 80 % de la cruda como piso,
 * de modo que una trayectoria ondulada cuesta velocidad. */
static void st_kt(it_plan_t *p, int i)
{
    const int n = p->n;
    float kt = 0.25f * g_ks[(i + n - 1) % n] + 0.5f * g_ks[i] + 0.25f * g_ks[(i + 1) % n];
    const float raw = 0.8f * fabsf(g_ks[i]);
    kt = fabsf(kt) >= raw ? kt : (kt < 0.0f ? -raw : raw);
    p->kappa_s[i] = kt;
    p->kappa[i] = fabsf(kt);
    g_dl[i] = it_atanf(p->cfg.wheelbase_cm * kt);
}
/* Curvatura SIN suavizar a partir del giro del rumbo entre tramos: la suavizada esconde un salto lateral entre estaciones vecinas (el optimizador lo
 * aprovecharia). Esta es la que acota lo que el volante puede hacer. */
static void st_viol(it_plan_t *p, const prof_const_t *pc, int i)
{
    const int n = p->n, a = (i + n - 1) % n;
    float dth = g_seg[i] - g_seg[a];
    while (dth > PI_F) dth -= 2.0f * PI_F;
    while (dth < -PI_F) dth += 2.0f * PI_F;
    const float kr = fabsf(dth) / (0.5f * (p->h[a] + p->h[i]));
    g_viol[i] = kr > 1.05f * pc->kmax ? (kr - 1.05f * pc->kmax) * 0.5f * (p->h[a] + p->h[i]) : 0.0f;
}
static void st_vlim(it_plan_t *p, const prof_const_t *pc, const float *d, int i)
{
    const it_config_t *c = &p->cfg;
    const int n = p->n, a = (i + n - 1) % n, b = (i + 1) % n;
    float v = c->v_max_cm_s;
    const float k = p->kappa[i];
    if (k > 1.0e-4f) { const float vk = sqrtf(pc->a_lim / k); if (vk < v) v = vk; }
    const float dds = fabsf(g_dl[b] - g_dl[a]) / (p->h[a] + p->h[i]);
    if (dds > 1.0e-5f) { const float vr = pc->rate / dds; if (vr < v) v = vr; }
    /* Holgura: a mas velocidad hace falta mas margen (error de pose + error de seguimiento, que crece con v^2; el morro barre
     * hacia fuera en curva). La velocidad maxima de la estacion es la que deja el margen dentro de la holgura que la
     * trayectoria tiene: un paso estrecho se cruza despacio, y una trayectoria con holgura permite ir rapido. */
    const float rl = d[i] - p->lo0[i], rr = p->hi0[i] - d[i];
    p->room[i] = rl < rr ? rl : rr;
    float allowed = (rl < rr ? rl : rr);
    /* La holgura de la HUELLA (en cualquier direccion) no sufre el error de seguimiento lateral como el pasillo del eje: con el mismo margen el
     * control optimo da 1,4 s mas de vuelta (7,77 frente a 6,41 s en la pista de prueba). Se le aplica solo la fraccion IT_GAP_ERR_SCALE. */
    const float gap_allowed = c->pose_err_cm + (p->gap[i] - c->pose_err_cm - 0.3f) * IT_GAP_ERR_SCALE_SQ;
    if (gap_allowed + 0.3f < allowed) allowed = gap_allowed + 0.3f;
    allowed -= 0.3f;
    float vc;
    const float spare = allowed - c->pose_err_cm - c->track_floor_cm;
    if (spare <= 0.0f) vc = c->v_min_cm_s;
    else {
        const float ratio = spare / fmaxf(c->track_err_cm, 0.01f);
        vc = 100.0f * (c->track_exp == 2.0f ? sqrtf(ratio) : powf(ratio, 1.0f / c->track_exp));
    }
    p->vcap[i] = vc;
    if (vc < v) v = vc;
    if (v < c->v_min_cm_s) v = c->v_min_cm_s;
    g_vlim[i] = v;
}

/* Barrido adelante-atras de velocidad (aceleracion y frenada limitadas por la friccion), tiempo de vuelta y penalizaciones.
 *
 * Es lo unico que recorre todo el anillo en cada evaluacion incremental, asi que domina el coste. Se hace en el dominio de v^2 (no hace falta v*v ni dos
 * raices por estacion) y la raiz del circulo de friccion solo se calcula cuando la curvatura la hace falta: si a_full^2 - ay^2 >= cap^2 la aceleracion
 * es simplemente cap (una recta). Mismo resultado que antes salvo el redondeo del ultimo bit. */
static float g_k2[IT_MAX_N];     /* curvatura suavizada al cuadrado (la usa el barrido de velocidad) */
static float g_v2[IT_MAX_N];

static float finish_eval(it_plan_t *p, const prof_const_t *pc)
{
    const int n = p->n;
    float viol = 0.0f, clip = 0.0f;
    for (int i = 0; i < n; ++i) viol += g_viol[i];
    const float af2 = pc->a_full * pc->a_full, thr_acc = af2 - pc->cap_acc * pc->cap_acc, thr_dec = af2 - pc->cap_dec * pc->cap_dec;
    for (int i = 0; i < n; ++i) {
        g_v2[i] = g_vlim[i] * g_vlim[i];
        g_k2[i] = p->kappa[i] * p->kappa[i];
        if (p->gap[i] < 0.0f) clip -= p->gap[i];
    }
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 0; i < n; ++i) {
            const int j = (i + 1 == n) ? 0 : i + 1;
            const float ay2 = g_v2[i] * g_v2[i] * g_k2[i];            /* (v^2 k)^2 */
            const float a = (ay2 <= thr_acc) ? pc->cap_acc : fminf(pc->cap_acc, sqrtf(fmaxf(0.0f, af2 - ay2)));
            const float vm2 = g_v2[i] + 2.0f * a * p->h[i];
            if (g_v2[j] > vm2) g_v2[j] = vm2;
        }
        for (int k = n - 1; k >= 0; --k) {
            const int i = k, j = (k + 1 == n) ? 0 : k + 1;
            const float ay2 = g_v2[j] * g_v2[j] * g_k2[i];
            const float a = (ay2 <= thr_dec) ? pc->cap_dec : fminf(pc->cap_dec, sqrtf(fmaxf(0.0f, af2 - ay2)));
            const float vm2 = g_v2[j] + 2.0f * a * p->h[i];
            if (g_v2[i] > vm2) g_v2[i] = vm2;
        }
    }
    for (int i = 0; i < n; ++i) p->v[i] = sqrtf(g_v2[i]);
    float t = 0.0f;
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        t += p->h[i] / (0.5f * (p->v[i] + p->v[j]));
    }
    p->violation = viol;
    p->time_s = t;
    p->clip = clip;
    /* Penalizaciones: el volante que no llega a la curvatura cuesta 2000 s por cm de exceso; cada cm de penetracion de la huella, 4000 s. */
    return t + 2000.0f * viol + 4000.0f * clip;
}

/* Estaciones desde `from`, `count` seguidas (en el anillo). */
#define IT_FOR_WINDOW(i, from, count) for (int _k = 0, i = wrap_i((from), p->n); _k < (count); ++_k, i = (i + 1) % p->n)

static void eval_stages(it_plan_t *p, const prof_const_t *pc, const float *d, int from, int count)
{
    IT_FOR_WINDOW(i, from, count) st_xz(p, d, i);
    IT_FOR_WINDOW(i, from, count) st_h_seg(p, i);
    IT_FOR_WINDOW(i, from, count) st_ks_hdg(p, i);
    IT_FOR_WINDOW(i, from, count) st_gap(p, i);
    IT_FOR_WINDOW(i, from, count) st_kt(p, i);
    IT_FOR_WINDOW(i, from, count) st_viol(p, pc, i);
    IT_FOR_WINDOW(i, from, count) st_vlim(p, pc, d, i);
}

/* Evaluacion completa: deja en cache la trayectoria d y rellena s y la longitud. */
static float eval(it_plan_t *p, const float *d)
{
    const int n = p->n;
    const prof_const_t pc = prof_const(&p->cfg);
    p->evals++;
    eval_stages(p, &pc, d, 0, n);
    g_owner = p;
    float acc = 0.0f;
    for (int i = 0; i < n; ++i) { p->s[i] = acc; acc += p->h[i]; }
    p->length_cm = acc;
    return finish_eval(p, &pc);
}

/* Evaluacion incremental: d difiere del ultimo estado en cache solo en las estaciones [first, first+count). Se puede deshacer una vez (eval_undo) si
 * el optimizador rechaza la prueba. */
#define IT_UNDO_MAX 128
static struct {
    bool valid;
    int from, count;
    float x[IT_UNDO_MAX], z[IT_UNDO_MAX], h[IT_UNDO_MAX], hd[IT_UNDO_MAX], gap[IT_UNDO_MAX], kappa[IT_UNDO_MAX], kappa_s[IT_UNDO_MAX], room[IT_UNDO_MAX],
          vcap[IT_UNDO_MAX], ks[IT_UNDO_MAX], seg[IT_UNDO_MAX], dl[IT_UNDO_MAX], viol[IT_UNDO_MAX], vlim[IT_UNDO_MAX];
} g_undo;

#ifdef IT_VERIFY_INCREMENTAL
float it_verify_max_err;   /* mayor diferencia (coste) entre la evaluacion incremental y la completa */
unsigned it_verify_count;
#endif

static float eval_inc(it_plan_t *p, const float *d, int first, int count)
{
    const int n = p->n;
    const int from = first - 3, win = count + 6;       /* +-3: la ventana de dependencias de la cadena de magnitudes */
    if (g_owner != p || win >= n || win > IT_UNDO_MAX) { g_undo.valid = false; return eval(p, d); }
    g_undo.valid = true; g_undo.from = wrap_i(from, n); g_undo.count = win;
    {
        int i = g_undo.from;
        for (int k = 0; k < win; ++k, i = (i + 1) % n) {
            g_undo.x[k] = p->x[i]; g_undo.z[k] = p->z[i]; g_undo.h[k] = p->h[i]; g_undo.hd[k] = p->heading_deg[i]; g_undo.gap[k] = p->gap[i];
            g_undo.kappa[k] = p->kappa[i]; g_undo.kappa_s[k] = p->kappa_s[i]; g_undo.room[k] = p->room[i]; g_undo.vcap[k] = p->vcap[i];
            g_undo.ks[k] = g_ks[i]; g_undo.seg[k] = g_seg[i]; g_undo.dl[k] = g_dl[i]; g_undo.viol[k] = g_viol[i]; g_undo.vlim[k] = g_vlim[i];
        }
    }
    const prof_const_t pc = prof_const(&p->cfg);
    p->evals++;
    eval_stages(p, &pc, d, from, win);
    const float cost = finish_eval(p, &pc);
#ifdef IT_VERIFY_INCREMENTAL
    {
        const int ev = p->evals;
        const float full = eval(p, d);       /* la completa sobre el mismo estado: si la cache era correcta, no cambia nada */
        p->evals = ev;
        const float err = fabsf(full - cost);
        if (err > it_verify_max_err) it_verify_max_err = err;
        ++it_verify_count;
    }
#endif
    return cost;
}

static void eval_undo(it_plan_t *p);
/* La prueba se rechaza: la cache vuelve a ser la de d (ultima aceptada). Si la prueba se evaluo entera (ventana grande), se reevalua d. */
static void eval_reject(it_plan_t *p, const float *d)
{
    if (g_undo.valid) { eval_undo(p); return; }
    eval(p, d);
}

static void eval_undo(it_plan_t *p)
{
    if (!g_undo.valid) return;
    const int n = p->n;
    int i = g_undo.from;
    for (int k = 0; k < g_undo.count; ++k, i = (i + 1) % n) {
        p->x[i] = g_undo.x[k]; p->z[i] = g_undo.z[k]; p->h[i] = g_undo.h[k]; p->heading_deg[i] = g_undo.hd[k]; p->gap[i] = g_undo.gap[k];
        p->kappa[i] = g_undo.kappa[k]; p->kappa_s[i] = g_undo.kappa_s[k]; p->room[i] = g_undo.room[k]; p->vcap[i] = g_undo.vcap[k];
        g_ks[i] = g_undo.ks[k]; g_seg[i] = g_undo.seg[k]; g_dl[i] = g_undo.dl[k]; g_viol[i] = g_undo.viol[k]; g_vlim[i] = g_undo.vlim[k];
    }
    g_undo.valid = false;
}

/* Mejor trayectoria de longitud + mix * curvatura con cotas lo <= d <= hi: minimos cuadrados con cotas resueltos por conjunto
 * activo y gradiente conjugado sin matriz (la relajacion simple no converge: el operador de curvatura esta muy mal condicionado). */
static void apply_h(const it_plan_t *p, float mix, const float *delta, const bool *freev, float *out)
{
    const int n = p->n;
    static float ux[IT_MAX_N], uz[IT_MAX_N];
    for (int i = 0; i < n; ++i) { const float dd = freev[i] ? delta[i] : 0.0f; ux[i] = dd * p->nx[i]; uz[i] = dd * p->nz[i]; }
    for (int i = 0; i < n; ++i) {
        if (!freev[i]) { out[i] = 0.0f; continue; }
        const int a = (i + n - 1) % n, a2 = (i + n - 2) % n, b = (i + 1) % n, b2 = (i + 2) % n;
        const float lx = ux[a] - 2.0f * ux[i] + ux[b], lz = uz[a] - 2.0f * uz[i] + uz[b];
        const float bx = ux[a2] - 4.0f * ux[a] + 6.0f * ux[i] - 4.0f * ux[b] + ux[b2];
        const float bz = uz[a2] - 4.0f * uz[a] + 6.0f * uz[i] - 4.0f * uz[b] + uz[b2];
        out[i] = (-lx + mix * bx) * p->nx[i] + (-lz + mix * bz) * p->nz[i];
    }
}

static void gradient(const it_plan_t *p, float mix, const float *d, float *g)
{
    const int n = p->n;
    static float px[IT_MAX_N], pz[IT_MAX_N];
    for (int i = 0; i < n; ++i) { px[i] = p->cx[i] + d[i] * p->nx[i]; pz[i] = p->cz[i] + d[i] * p->nz[i]; }
    for (int i = 0; i < n; ++i) {
        const int a = (i + n - 1) % n, a2 = (i + n - 2) % n, b = (i + 1) % n, b2 = (i + 2) % n;
        const float lx = px[a] - 2.0f * px[i] + px[b], lz = pz[a] - 2.0f * pz[i] + pz[b];
        const float bx = px[a2] - 4.0f * px[a] + 6.0f * px[i] - 4.0f * px[b] + px[b2];
        const float bz = pz[a2] - 4.0f * pz[a] + 6.0f * pz[i] - 4.0f * pz[b] + pz[b2];
        g[i] = (-lx + mix * bx) * p->nx[i] + (-lz + mix * bz) * p->nz[i];
    }
}

static void relax(it_plan_t *p, float mix, float *d)
{
    const int n = p->n;
    static float g[IT_MAX_N], r[IT_MAX_N], q[IT_MAX_N], hq[IT_MAX_N], dl[IT_MAX_N];
    static bool fr[IT_MAX_N];
    for (int i = 0; i < n; ++i) d[i] = fminf(fmaxf(d[i], p->lo[i]), p->hi[i]);
    for (int outer = 0; outer < 60; ++outer) {
        gradient(p, mix, d, g);
        int nfree = 0;
        for (int i = 0; i < n; ++i) {
            const bool at_lo = d[i] <= p->lo[i] + 1.0e-4f && g[i] > 0.0f, at_hi = d[i] >= p->hi[i] - 1.0e-4f && g[i] < 0.0f;
            fr[i] = !(at_lo || at_hi) && p->hi[i] - p->lo[i] > 1.0e-4f;
            if (fr[i]) ++nfree;
        }
        if (nfree == 0) break;
        /* CG sobre las libres: H dl = -g. */
        float rr = 0.0f;
        for (int i = 0; i < n; ++i) { dl[i] = 0.0f; r[i] = fr[i] ? -g[i] : 0.0f; q[i] = r[i]; rr += r[i] * r[i]; }
        const float rr0 = rr;
        for (int it = 0; it < p->cfg.relax_iterations && rr > 1.0e-10f * rr0 && rr > 1.0e-12f; ++it) {
            apply_h(p, mix, q, fr, hq);
            float qhq = 0.0f;
            for (int i = 0; i < n; ++i) qhq += q[i] * hq[i];
            if (qhq <= 1.0e-20f) break;
            const float alpha = rr / qhq;
            float rr2 = 0.0f;
            for (int i = 0; i < n; ++i) { dl[i] += alpha * q[i]; r[i] -= alpha * hq[i]; rr2 += r[i] * r[i]; }
            const float beta = rr2 / rr;
            rr = rr2;
            for (int i = 0; i < n; ++i) q[i] = r[i] + beta * q[i];
        }
        /* Paso completo y proyeccion sobre las cotas (las que se tocan pasan a activas en la vuelta siguiente). */
        float moved = 0.0f;
        for (int i = 0; i < n; ++i) {
            if (!fr[i]) continue;
            const float nd = fminf(fmaxf(d[i] + dl[i], p->lo[i]), p->hi[i]);
            if (fabsf(nd - d[i]) > moved) moved = fabsf(nd - d[i]);
            d[i] = nd;
        }
        if (moved < 1.0e-3f) break;
    }
}


/* Corredor libre de cada estacion (de punto). Falso si alguna estacion no tiene paso. */
static bool compute_corridor(it_plan_t *p)
{
    const it_config_t *c = &p->cfg;
    const int n = p->n;
    /* Tramo libre por estacion; con varios tramos (letrero cruzando la normal) se elige uno por estacion minimizando el salto
     * entre vecinas (programacion dinamica sobre el anillo). */
    const float reach = 0.5f * (c->outer_half_cm - c->inner_half_cm) + 5.0f;
    enum { MAXIV = 6 };
    static float iv_lo[IT_MAX_N][MAXIV], iv_hi[IT_MAX_N][MAXIV], cost[IT_MAX_N][MAXIV];
    static int iv_n[IT_MAX_N];
    static int8_t from[IT_MAX_N][MAXIV], best_pick[IT_MAX_N];
    for (int i = 0; i < n; ++i) {
        int ni = 0;
        float run_lo = 0.0f, last = 0.0f;
        bool in_run = false;
        for (float d = -reach; d <= reach + 0.01f; d += 1.0f) {
            const bool f = free_at(p, p->cx[i] + d * p->nx[i], p->cz[i] + d * p->nz[i]);
            if (f && !in_run) { in_run = true; run_lo = d; }
            if (!f && in_run) { if (ni < MAXIV) { iv_lo[i][ni] = run_lo; iv_hi[i][ni] = last; ++ni; } in_run = false; }
            last = d;
        }
        if (in_run && ni < MAXIV) { iv_lo[i][ni] = run_lo; iv_hi[i][ni] = last; ++ni; }
        if (ni == 0) return false;
        iv_n[i] = ni;
    }
    float best_total = 1.0e18f;
    for (int s0 = 0; s0 < iv_n[0]; ++s0) {
        for (int k = 0; k < MAXIV; ++k) cost[0][k] = 1.0e18f;
        cost[0][s0] = 0.0f;
        for (int i = 1; i < n; ++i) {
            for (int k = 0; k < iv_n[i]; ++k) {
                const float mx = p->cx[i] + 0.5f * (iv_lo[i][k] + iv_hi[i][k]) * p->nx[i];
                const float mz = p->cz[i] + 0.5f * (iv_lo[i][k] + iv_hi[i][k]) * p->nz[i];
                cost[i][k] = 1.0e18f; from[i][k] = 0;
                for (int q = 0; q < iv_n[i - 1]; ++q) {
                    if (cost[i - 1][q] >= 1.0e17f) continue;
                    const float qx = p->cx[i - 1] + 0.5f * (iv_lo[i - 1][q] + iv_hi[i - 1][q]) * p->nx[i - 1];
                    const float qz = p->cz[i - 1] + 0.5f * (iv_lo[i - 1][q] + iv_hi[i - 1][q]) * p->nz[i - 1];
                    const float cst = cost[i - 1][q] + hypotf(mx - qx, mz - qz);
                    if (cst < cost[i][k]) { cost[i][k] = cst; from[i][k] = (int8_t)q; }
                }
            }
        }
        for (int k = 0; k < iv_n[n - 1]; ++k) {
            if (cost[n - 1][k] >= 1.0e17f) continue;
            const float qx = p->cx[n - 1] + 0.5f * (iv_lo[n - 1][k] + iv_hi[n - 1][k]) * p->nx[n - 1];
            const float qz = p->cz[n - 1] + 0.5f * (iv_lo[n - 1][k] + iv_hi[n - 1][k]) * p->nz[n - 1];
            const float mx = p->cx[0] + 0.5f * (iv_lo[0][s0] + iv_hi[0][s0]) * p->nx[0];
            const float mz = p->cz[0] + 0.5f * (iv_lo[0][s0] + iv_hi[0][s0]) * p->nz[0];
            const float total = cost[n - 1][k] + hypotf(mx - qx, mz - qz);
            if (total < best_total) {
                best_total = total;
                int pick = k;
                for (int i = n - 1; i >= 0; --i) { best_pick[i] = (int8_t)pick; pick = from[i][pick]; }
            }
        }
    }
    if (best_total >= 1.0e17f) return false;
    for (int i = 0; i < n; ++i) { p->lo0[i] = iv_lo[i][best_pick[i]]; p->hi0[i] = iv_hi[i][best_pick[i]]; }
    return true;
}

/* ------------------------------------------------------------ construccion */

/* Construccion por rebanadas: `it_plan_begin` prepara el corredor y `it_plan_step` reparte la busqueda en trozos de pocos milisegundos, de modo
 * que el controlador puede llamarla en cada escaneo sin parar el control. Hay un solo trabajo a la vez (estado estatico). */
static struct {
    int stage, q, sw, ai, wi, i, sg, sweeps, rcount, attempt, rand_limit;
    float best_cur;
    uint32_t rng;
    bool improved;
    float best, best_t;
} job;
static float jd[IT_MAX_N], jbest[IT_MAX_N];

bool it_plan_begin(it_plan_t *p, const it_config_t *cfg, int direction, const it_obstacle_t *obs, int count)
{
    if (!p) return false;
    memset(p, 0, sizeof(*p));
    if (cfg) p->cfg = *cfg; else it_config_default(&p->cfg);
    it_config_sanitize(&p->cfg);
    const it_config_t *c = &p->cfg;
    p->direction = direction >= 0 ? 1 : -1;
    if (count < 0) count = 0;
    if (count > IT_MAX_OBS) count = IT_MAX_OBS;
    p->obstacles = count;
    for (int j = 0; j < count; ++j) p->obs[j] = obs[j];

    const float hh = 0.5f * (c->outer_half_cm + c->inner_half_cm), rc = c->corner_radius_cm;
    const float period = 4.0f * (2.0f * (hh - rc) + 0.5f * PI_F * rc);
    int n = (int)(period / c->station_cm + 0.5f);
    if (n > IT_MAX_N) n = IT_MAX_N;
    if (n < 16) n = 16;
    p->n = n;
    const float step = period / (float)n;
    for (int i = 0; i < n; ++i) {
        const float sp = p->direction < 0 ? (float)i * step : period - (float)i * step;
        float tx, tz;
        bool arc;
        center_at(c, sp, &p->cx[i], &p->cz[i], &tx, &tz, &arc);
        if (p->direction > 0) { tx = -tx; tz = -tz; }
        p->nx[i] = tz; p->nz[i] = -tx;
    }
    for (int j = 0; j < count; ++j) {
        int best = 0;
        float bd = 1.0e9f;
        for (int i = 0; i < n; ++i) {
            const float dd = hypotf(p->cx[i] - obs[j].x_cm, p->cz[i] - obs[j].z_cm);
            if (dd < bd) { bd = dd; best = i; }
        }
        p->obs_rx[j] = p->nx[best]; p->obs_rz[j] = p->nz[best];
    }
    if (!compute_corridor(p)) { p->valid = false; return false; }

    /* Referencia: la linea central acotada (para saber lo que se gana). */
    for (int i = 0; i < n; ++i) { p->lo[i] = p->lo0[i]; p->hi[i] = p->hi0[i]; p->d[i] = fminf(fmaxf(0.0f, p->lo0[i]), p->hi0[i]); }
    eval(p, p->d);
    p->time_center_s = p->time_s;
    memset(&job, 0, sizeof(job));
    job.sweeps = c->sweeps * c->passes;
    job.best_t = 1.0e18f;
    job.sg = -1;
    job.rng = 12345u;
    job.rand_limit = IT_RAND_EVALS;
    p->building = true;
    return true;
}

/* Avanza el trabajo como mucho `budget` evaluaciones (cada una cuesta ~30 us en un PC). Devuelve true al terminar (p->valid dice si hay
 * plan). Cada arranque de la etapa 0 cuenta como 40 evaluaciones (un relax). */
bool it_plan_step(it_plan_t *p, int budget)
{
    if (!p || !p->building) return true;
    const it_config_t *c = &p->cfg;
    const int n = p->n;
    static const float MIX[6] = {0.0f, 0.3f, 1.0f, 3.0f, 10.0f, 30.0f};
    static const int W[IT_NW] = IT_WIDTHS;
    static const float A[IT_NA] = IT_AMPS;
    static float trial[IT_MAX_N];
    while (budget > 0 && job.stage == 0) {
        /* Varios arranques (de la ruta mas corta a la de curvatura minima): se parte del que da la vuelta mas rapida. */
        for (int i = 0; i < n; ++i) jd[i] = fminf(fmaxf(0.5f * (p->lo[i] + p->hi[i]), p->lo[i]), p->hi[i]);
        if (!((IT_START_MASK >> job.q) & 1)) { budget -= 1; if (++job.q >= 6) { memcpy(jd, jbest, sizeof(float) * (size_t)n); job.best = eval(p, jd); job.stage = 1; } continue; }
        relax(p, MIX[job.q], jd);
        float t = eval(p, jd);
#ifdef IT_ONLY_MIX
        if (job.q != IT_ONLY_MIX) t = 1.0e17f;
#endif
        if (t < job.best_t) { job.best_t = t; memcpy(jbest, jd, sizeof(float) * (size_t)n); }
        budget -= 40;
        if (++job.q >= 6) { memcpy(jd, jbest, sizeof(float) * (size_t)n); job.best = eval(p, jd); job.stage = 1; }
    }
    /* Busqueda local: jorobas suaves (coseno) de varias anchuras y amplitudes sobre d, aceptando solo lo que baja el tiempo. */
    while (budget > 0 && job.stage == 1) {
        const int w = W[job.wi];
        memcpy(trial, jd, sizeof(float) * (size_t)n);
        for (int k = -w; k <= w; ++k) {
            const int j = ((job.i + k) % n + n) % n;
            const float wt = 0.5f * (1.0f + cosf(PI_F * (float)k / (float)(w + 1)));
            float v = trial[j] + (float)job.sg * A[job.ai] * wt;
            if (v < p->lo[j]) v = p->lo[j];
            if (v > p->hi[j]) v = p->hi[j];
            trial[j] = v;
        }
        const float t = eval_inc(p, trial, job.i - w, 2 * w + 1);
        --budget;
        if (t < job.best - 1.0e-4f) { job.best = t; memcpy(jd, trial, sizeof(float) * (size_t)n); job.improved = true; }
        else eval_reject(p, jd);
        job.sg += 2;
        if (job.sg > 1) {
            job.sg = -1;
            job.i += W[job.wi] / 2 > 1 ? W[job.wi] / 2 : 1;
            if (job.i >= n) {
                job.i = 0;
                if (++job.wi >= IT_NW) {
                    job.wi = 0;
                    if (++job.ai >= IT_NA) {
                        job.ai = 0;
                        ++job.sw;
                        if (!job.improved || job.sw >= job.sweeps) { job.stage = 5; job.best_cur = job.best; memcpy(jbest, jd, sizeof(float) * (size_t)n); }
                        job.improved = false;
                    }
                }
            }
        }
    }
    /* Etapa 5: busqueda aleatoria con jorobas de ancho y amplitud al azar (la local determinista se atasca en un minimo: no cruza de un
     * lado al otro del corredor ni mueve un tramo largo a la vez). Solo acepta lo que baja el tiempo. */
    while (budget > 0 && job.stage == 5) {
        if (job.rcount >= job.rand_limit) { if (job.rand_limit > 0) memcpy(jd, jbest, sizeof(float) * (size_t)n); job.stage = 2; break; }
        job.rng = job.rng * 1664525u + 1013904223u; const uint32_t r1 = job.rng >> 8;
        job.rng = job.rng * 1664525u + 1013904223u; const uint32_t r2 = job.rng >> 8;
        job.rng = job.rng * 1664525u + 1013904223u; const uint32_t r3 = job.rng >> 8;
        const int ci = (int)(r1 % (uint32_t)n);
        const int w = 2 + (int)(r2 % 26u);
        const float u = (float)(r3 & 0xFFFFu) / 65535.0f;
        const float amp = (0.3f * powf(30.0f, u)) * ((r3 >> 16) & 1u ? 1.0f : -1.0f);   /* 0,3 .. 9 cm */
        memcpy(trial, jd, sizeof(float) * (size_t)n);
        for (int k = -w; k <= w; ++k) {
            const int j = ((ci + k) % n + n) % n;
            const float wt = 0.5f * (1.0f + cosf(PI_F * (float)k / (float)(w + 1)));
            float v = trial[j] + amp * wt;
            if (v < p->lo[j]) v = p->lo[j];
            if (v > p->hi[j]) v = p->hi[j];
            trial[j] = v;
        }
        const float t = eval_inc(p, trial, ci - w, 2 * w + 1);
        --budget; ++job.rcount;
        /* Recocido: acepta tambien subidas pequenas (temperatura que baja de 0,05 a 0,0005 s) y guarda la mejor; solo aceptar lo que mejora deja la busqueda
         * en el primer valle (con el mapa real del robot: 9,2 s frente a 8,0 s). */
        const float frac = (float)job.rcount / (float)(job.rand_limit > 0 ? job.rand_limit : 1);
        const float temp = IT_SA_T0 * powf(IT_SA_T1 / IT_SA_T0, frac);
        job.rng = job.rng * 1664525u + 1013904223u;
        const float u01 = (float)(job.rng >> 8) / 16777216.0f;
        if (t < job.best_cur || (IT_SA_T0 > 0.0f && u01 < expf(-(t - job.best_cur) / temp))) {
            job.best_cur = t; memcpy(jd, trial, sizeof(float) * (size_t)n);
            if (t < job.best - 1.0e-4f) { job.best = t; memcpy(jbest, trial, sizeof(float) * (size_t)n); }
        } else {
            eval_reject(p, jd);
        }
    }
    if (job.stage != 2) return false;
    eval(p, jd);
    for (int i = 0; i < n; ++i) {
        p->d[i] = jd[i];
        const float vr = p->v[i] / 100.0f;
        p->margin[i] = c->pose_err_cm + c->track_floor_cm + c->track_err_cm * powf(vr, c->track_exp);
    }
    /* Valida solo si el volante llega a la curvatura, la linea es continua (ningun salto mayor que 2 estaciones) y la huella del
     * chasis, con el rumbo de la trayectoria final, no toca nada. */
    float jump = 0.0f;
    for (int i = 0; i < n; ++i) if (p->h[i] > jump) jump = p->h[i];
    /* El volante no llega a la curvatura que pide la pista (pasos en S seguidos): se reintenta con mas volante antes de rendirse. El plan
     * normal reserva el 28 % del volante para que el seguidor corrija; sin plan no hay IT, y con el volante casi a tope si (despacio). */
    if (p->violation >= 0.02f && job.attempt < IT_STEER_RETRIES && p->cfg.steer_use < 0.999f) {
        const int attempt = job.attempt + 1;
        p->cfg.steer_use = fminf(1.0f, p->cfg.steer_use + 0.14f);
        for (int i = 0; i < n; ++i) { p->lo[i] = p->lo0[i]; p->hi[i] = p->hi0[i]; }
        memset(&job, 0, sizeof(job));
        job.attempt = attempt;
        job.rand_limit = IT_RAND_EVALS;
        job.sweeps = c->sweeps * c->passes;
        job.best_t = 1.0e18f;
        job.sg = -1;
        job.rng = 12345u;
        return false;
    }
    p->why = 0;
    if (p->violation >= 0.02f) p->why |= 1u;
    if (jump >= 2.0f * c->station_cm * 1.5f + 3.0f) p->why |= 2u;
    if (!it_path_clear(p, NULL)) p->why |= 4u;
    p->room_min = 1.0e9f;
    for (int i = 0; i < n; ++i) if (p->room[i] < p->room_min) p->room_min = p->room[i];
    p->valid = p->why == 0;
    p->building = false;
    return true;
}

/* Refinado: continua la busqueda desde un plan ya valido (el que el coche esta usando) con jorobas aleatorias (etapa 5), repartido en rebanadas como
 * it_plan_begin. El plan en uso no se toca: `work` recibe el resultado y el llamador lo cambia si `time_s` baja. La busqueda local determinista se queda
 * en un minimo (con el mapa real del robot: 9,2 s frente a 8,0 s que el mismo modelo permite); asi el IT arranca con el primer plan y mejora mientras conduce. */
bool it_plan_refine_begin(it_plan_t *work, const it_plan_t *base, int evals)
{
    if (!work || !base || !base->valid || evals <= 0) return false;
    memcpy(work, base, sizeof(*work));
    const int n = work->n;
    for (int i = 0; i < n; ++i) { jd[i] = base->d[i]; work->lo[i] = work->lo0[i]; work->hi[i] = work->hi0[i]; }
    memset(&job, 0, sizeof(job));
    job.stage = 5;
    job.attempt = IT_STEER_RETRIES;
    job.rng = 987654321u;
    job.rand_limit = evals;
    job.best = eval(work, jd);
    job.best_cur = job.best;
    memcpy(jbest, jd, sizeof(float) * (size_t)n);
    work->valid = false;
    work->building = true;
    return true;
}

bool it_plan_build(it_plan_t *p, const it_config_t *cfg, int direction, const it_obstacle_t *obs, int count)
{
    if (!it_plan_begin(p, cfg, direction, obs, count)) return false;
    while (!it_plan_step(p, 1 << 20)) {}
    return p->valid;
}

bool it_path_clear(const it_plan_t *p, float *min_gap)
{
    if (!p || p->n < 3) return false;
    float gap = 1.0e9f;
    for (int i = 0; i < p->n; ++i) {
        const float g = footprint_gap(&p->cfg, p->x[i], p->z[i], p->heading_deg[i] * DEG2RAD, p->obs, p->obstacles);
        if (g < gap) gap = g;
    }
    if (min_gap) *min_gap = gap;
    return gap >= p->cfg.wall_margin_cm - 0.05f;
}

/* ------------------------------------------------------------ consulta y seguimiento */

int it_locate(const it_plan_t *p, float x, float z, int hint, float window_cm, float *lateral_cm, float *dist_cm)
{
    if (!p || !p->valid) return -1;
    const int n = p->n;
    int best = -1;
    float bd = 1.0e18f;
    if (hint >= 0 && hint < n && window_cm > 0.0f) {
        const int span = (int)(window_cm / (p->length_cm / (float)n)) + 1;
        for (int k = -span; k <= span; ++k) {
            const int i = ((hint + k) % n + n) % n;
            const float d2 = (x - p->x[i]) * (x - p->x[i]) + (z - p->z[i]) * (z - p->z[i]);
            if (d2 < bd) { bd = d2; best = i; }
        }
    } else {
        for (int i = 0; i < n; ++i) {
            const float d2 = (x - p->x[i]) * (x - p->x[i]) + (z - p->z[i]) * (z - p->z[i]);
            if (d2 < bd) { bd = d2; best = i; }
        }
    }
    if (best >= 0) {
        if (dist_cm) *dist_cm = sqrtf(bd);
        if (lateral_cm) {
            const float hd = p->heading_deg[best] * DEG2RAD;
            *lateral_cm = (x - p->x[best]) * cosf(hd) + (z - p->z[best]) * (-sinf(hd));
        }
    }
    return best;
}

static int advance(const it_plan_t *p, int idx, float ahead_cm, float *rest)
{
    const int n = p->n;
    float left = ahead_cm > 0.0f ? ahead_cm : 0.0f;
    int i = idx % n;
    for (int guard = 0; guard < 2 * n && left >= p->h[i]; ++guard) { left -= p->h[i]; i = (i + 1) % n; }
    if (rest) *rest = left;
    return i;
}

float it_speed_at(const it_plan_t *p, int idx, float ahead_cm)
{
    if (!p || !p->valid || idx < 0) return 0.0f;
    return p->v[advance(p, idx, ahead_cm, NULL)];
}

void it_tracker_reset(it_tracker_t *t)
{
    if (t) { memset(t, 0, sizeof(*t)); t->idx = -1; }
}

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

bool it_track(it_tracker_t *t, const it_plan_t *p, float x, float z, float yaw_deg, float v, float *steer_deg, float *speed_cm_s)
{
    if (!t || !p || !p->valid) return false;
    const it_config_t *c = &p->cfg;
    float lat = 0.0f, dist = 0.0f;
    const int idx = it_locate(p, x, z, t->idx, 80.0f, &lat, &dist);
    if (idx < 0) return false;
    t->idx = idx; t->e_cm = lat; t->room_cm = p->room[idx];
    t->psi_err_deg = wrap180(yaw_deg - p->heading_deg[idx]);
    t->on_path = dist < 40.0f && fabsf(t->psi_err_deg) < 70.0f;
    if (!t->on_path) return false;
    /* Persecucion pura con adelanto por el retardo del servo: el punto al que se apunta esta v * lag mas alla. */
    const float vv = v > 0.0f ? v : 0.0f;
    float ld = 8.0f + 0.12f * vv;
    if (ld < 12.0f) ld = 12.0f;
    if (ld > 36.0f) ld = 36.0f;
    float rest;
    const int j = advance(p, idx, ld + vv * c->lag_s, &rest);
    const int k = (j + 1) % p->n;
    const float u = rest / p->h[j];
    const float tx = p->x[j] + u * (p->x[k] - p->x[j]), tz = p->z[j] + u * (p->z[k] - p->z[j]);
    const float yr = yaw_deg * DEG2RAD, dx = tx - x, dz = tz - z;
    const float fwd = dx * sinf(yr) + dz * cosf(yr), rgt = dx * cosf(yr) - dz * sinf(yr);
    const float ldist = hypotf(dx, dz) > 1.0f ? hypotf(dx, dz) : 1.0f;
    const float alpha = atan2f(rgt, fwd);
    float steer = atanf(2.0f * c->wheelbase_cm * sinf(alpha) / ldist) * RAD2DEG;
    const float smax = 0.97f * c->steer_max_deg;
    if (c->ctrl_mode == 1) {
        /* Anticipacion: la curvatura que el plan tendra cuando la orden haga efecto (retardo del servo), mas realimentacion del error
         * de rumbo y del lateral. La persecucion pura solo reacciona al error y con 0,1 s de retardo llega tarde en las esquinas. */
        float r2;
        const int jf = advance(p, idx, vv * c->lag_s, &r2);
        const float ff = atanf(c->wheelbase_cm * p->kappa_s[jf]) * RAD2DEG;
        steer = ff - c->k_psi * t->psi_err_deg - atanf(c->k_e * lat / (vv + 20.0f)) * RAD2DEG;
    }
    if (steer > smax) steer = smax;
    if (steer < -smax) steer = -smax;
    /* Velocidad del perfil unos instantes por delante; se recorta si el error lateral se come la holgura. */
    float vp = it_speed_at(p, idx, 5.0f + vv * (c->lag_s + 0.15f));
    const float room = p->room[idx] > 0.5f ? p->room[idx] : 0.5f, ae = fabsf(lat);
    float f = 1.0f;
    if (ae > 0.5f * room) { f = 1.0f - (ae - 0.5f * room) / (0.5f * room + 0.5f); if (f < 0.35f) f = 0.35f; }
    if (fabsf(t->psi_err_deg) > 25.0f) f *= 0.6f;
    const float vc = vp * f > c->v_min_cm_s ? vp * f : c->v_min_cm_s;
    t->steer_deg = steer; t->v_plan_cm_s = vp; t->v_cmd_cm_s = vc;
    if (steer_deg) *steer_deg = steer;
    if (speed_cm_s) *speed_cm_s = vc;
    return true;
}

float it_adapt_grip(const it_config_t *c, const it_lap_stats_t *s, float grip_max)
{
    float g = c->grip;
    const bool bad = s->contacts > 0 || s->min_room_cm < 0.0f || s->traction_events > 20;
    if (bad) g -= 0.15f;
    else if (s->min_room_cm > 3.0f && s->max_slip_deg < 5.0f && s->traction_events < 3) g += 0.05f;
    if (g < 0.3f) g = 0.3f;
    if (g > grip_max) g = grip_max;
    return g;
}

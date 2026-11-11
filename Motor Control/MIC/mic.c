#include "mic.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define CM 0.01f   /* cm -> m */
#define MIC_OVERSPEED_PLUG_MS 0.06f   /* exceso de velocidad (m/s) sobre un perfil que baja a partir del cual se frena activo */

static float sgnf(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* Perfil base del JGB37-520 de 12 V con la reductora 10:1. `rpm_out` son las rpm en vacio del eje de salida a 12 V:
 * 600 es el motor que hay hoy; 1000 es el definitivo (aun no llega). Lo demas es comun y sigue siendo SUPUESTO. */
void mic_cfg_jgb37_520(mic_cfg_t *c, float rpm_out)
{
    memset(c, 0, sizeof *c);
    c->wheel_diam_cm = 6.6f;
    c->gear_ratio = 10.0f;               /* reductora; la rueda va 1:1 al eje de salida (confirmar) */
    c->counts_per_motor_rev = 44.0f;     /* 11 pulsos por canal x 4 flancos; 440 cuentas por vuelta de rueda */
    c->v_nl_volts = 12.0f;
    c->rpm_nl_out = rpm_out;
    c->i_noload_a = 0.15f;
    c->r_ohm = 2.6f;                     /* SUPUESTO: medir con el rotor bloqueado a baja tension */
    c->eta = 0.75f;
    c->mass_kg = 1.2f;                   /* SUPUESTO: pesar el coche montado */
    c->j_rotor_kgm2 = 2.0e-6f;
    c->f_coulomb_n = 0.6f;               /* SUPUESTO: reductora + escobillas + rodadura */
    c->b_viscous_ns_m = 0.5f;
    c->a_grip_ms2 = 3.0f;
    c->vbat_nominal = 11.1f;             /* 3S */
    c->vdrop_volts = 2.0f;               /* L298N (Kconfig WRO_MOTOR_DROP_MV) */
    c->i_max_a = 2.0f;                   /* L298N: 2 A por canal */
    c->can_coast = false;                /* ENA puenteado */
    c->kv_gain = 25.0f;
    c->ki_gain = 80.0f;
    c->obs_bw_hz = 12.0f;
    c->plan_decel_frac = 0.65f;
    c->plan_accel_frac = 0.9f;
    c->v_creep_cm_s = 4.0f;
    c->pos_tol_cm = 0.25f;
    c->v_stop_cm_s = 1.0f;
    /* Techo de velocidad: la de vacio a la tension disponible (3S menos la caida del L298N) con ~10 % de margen. */
    c->v_max_cm_s = 0.9f * rpm_out * (c->wheel_diam_cm * PI_F) / 60.0f * (c->vbat_nominal - c->vdrop_volts) / c->v_nl_volts;
}

void mic_cfg_jgb37_520_600rpm(mic_cfg_t *c) { mic_cfg_jgb37_520(c, 600.0f); }
void mic_cfg_jgb37_520_1000rpm(mic_cfg_t *c) { mic_cfg_jgb37_520(c, 1000.0f); }

void mic_init(mic_t *m, const mic_cfg_t *cfg)
{
    memset(m, 0, sizeof *m);
    m->cfg = *cfg;
    const float r = 0.5f * cfg->wheel_diam_cm * CM;
    const float w_nl_motor = cfg->rpm_nl_out * cfg->gear_ratio * 2.0f * PI_F / 60.0f;
    m->ke_motor = (cfg->v_nl_volts - cfg->i_noload_a * cfg->r_ohm) / w_nl_motor;   /* V s/rad == N m/A */
    m->ke_lin = m->ke_motor * cfg->gear_ratio / r;                                  /* V por (m/s) */
    m->kf_lin = m->ke_motor * cfg->gear_ratio * cfg->eta / r;                       /* N por A */
    const float refl = cfg->gear_ratio / r;
    m->m_eff = cfg->mass_kg + cfg->j_rotor_kgm2 * refl * refl;
    m->cm_per_count = PI_F * cfg->wheel_diam_cm / (cfg->counts_per_motor_rev * cfg->gear_ratio);
    m->v_avail_nom = cfg->vbat_nominal - cfg->vdrop_volts;
    m->mode = MIC_MODE_IDLE;
}

/* ------------------------------------------------------------------ modelo */

static float fric(const mic_t *m, float v)
{
    return m->cfg.f_coulomb_n * tanhf(v / 0.01f) + m->cfg.b_viscous_ns_m * v;
}

/* Aceleracion que produce una tension Vm aplicada al motor con la velocidad v. */
static float accel_model(const mic_t *m, float vm, float v)
{
    const float i = (vm - m->ke_lin * v) / m->cfg.r_ohm;
    const float f = m->kf_lin * i;
    if (fabsf(v) < 0.003f && fabsf(f) <= m->cfg.f_coulomb_n) return 0.0f;   /* friccion estatica */
    return (f - fric(m, v)) / m->m_eff;
}

static float v_avail(const mic_t *m, float vbat)
{
    const float vb = vbat > 1.0f ? vbat : m->cfg.vbat_nominal;
    return fmaxf(vb - m->cfg.vdrop_volts, 1.0f);
}

/* Capacidades en SI, velocidad v >= 0 en el sentido de la marcha. */
static float cap_accel(const mic_t *m, float v)
{
    const float i = fminf(m->cfg.i_max_a, (m->v_avail_nom - m->ke_lin * v) / m->cfg.r_ohm);
    const float a = (m->kf_lin * i - fric(m, v)) / m->m_eff;
    return clampf(a, 0.0f, m->cfg.a_grip_ms2);
}

static float cap_decel(const mic_t *m, float v, mic_decel_t mode)
{
    const float i = mode == MIC_DECEL_AGGRESSIVE ? m->cfg.i_max_a : m->ke_lin * v / m->cfg.r_ohm;
    const float a = (m->kf_lin * fminf(i, m->cfg.i_max_a) + fric(m, v)) / m->m_eff;
    return clampf(a, 0.05f, m->cfg.a_grip_ms2);
}

float mic_max_accel_cm_s2(const mic_t *m, float v) { return cap_accel(m, fabsf(v) * CM) / CM; }
float mic_max_decel_cm_s2(const mic_t *m, float v, mic_decel_t d) { return cap_decel(m, fabsf(v) * CM, d) / CM; }

float mic_stop_distance_cm(const mic_t *m, float v_cm_s, mic_decel_t decel)
{
    float v = fabsf(v_cm_s) * CM, d = 0.0f;
    const float dv = fmaxf(v / 40.0f, 1e-3f);
    while (v > 1e-3f) {
        const float a = cap_decel(m, v, decel);
        d += v * dv / a;
        v -= dv;
    }
    return d / CM;
}

/* ------------------------------------------------------------------ ordenes */

void mic_request_speed(mic_t *m, float v_cm_s, mic_decel_t decel)
{
    /* Una orden de velocidad que llega cada barrido (10 Hz) no debe reiniciar la referencia ni la integral. */
    const bool continuing = m->mode == MIC_MODE_SPEED && !m->done_flag;
    m->mode = MIC_MODE_SPEED;
    m->decel = decel;
    m->v_target = clampf(v_cm_s, -m->cfg.v_max_cm_s, m->cfg.v_max_cm_s) * CM;
    if (!continuing) m->fresh = true;
    m->done_flag = false;
    m->emergency_latched = false;
}

void mic_request_stop(mic_t *m, mic_decel_t decel) { mic_request_speed(m, 0.0f, decel); }

void mic_route_begin(mic_t *m) { m->nseg = 0; }

bool mic_route_add(mic_t *m, float length_cm, float v_max_cm_s, float v_exit_max_cm_s)
{
    if (m->nseg >= MIC_ROUTE_MAX_SEGMENTS || !(length_cm > 0.0f) || !(v_max_cm_s > 0.0f)) return false;
    mic_segment_t *s = &m->seg[m->nseg++];
    s->length_m = length_cm * CM;
    s->v_max = fminf(v_max_cm_s, m->cfg.v_max_cm_s) * CM;
    s->v_exit_max = v_exit_max_cm_s > 0.0f ? fminf(v_exit_max_cm_s, v_max_cm_s) * CM : s->v_max;
    return true;
}

bool mic_route_commit(mic_t *m, float v_end_cm_s, mic_decel_t decel)
{
    if (m->nseg <= 0) return false;
    float total = 0.0f;
    for (int i = 0; i < m->nseg; ++i) total += m->seg[i].length_m;
    const int n = (int)clampf(total / 0.005f + 2.0f, 3.0f, (float)MIC_PROFILE_POINTS);
    const float ds = total / (float)(n - 1);
    float lim[MIC_PROFILE_POINTS];
    for (int i = 0; i < n; ++i) lim[i] = m->cfg.v_max_cm_s * CM;
    float start = 0.0f;
    for (int j = 0; j < m->nseg; ++j) {
        const float end = start + m->seg[j].length_m;
        for (int i = 0; i < n; ++i) {
            const float s = (float)i * ds;
            if (s >= start - 1e-6f && s <= end + 1e-6f) lim[i] = fminf(lim[i], m->seg[j].v_max);
        }
        const int ie = (int)lroundf(end / ds);
        if (ie >= 0 && ie < n) lim[ie] = fminf(lim[ie], m->seg[j].v_exit_max);
        start = end;
    }
    const float v_end = fmaxf(v_end_cm_s, 0.0f) * CM;
    lim[n - 1] = fminf(lim[n - 1], v_end);
    /* paso hacia atras: frenar lo mas tarde posible con la desaceleracion que el motor da a cada velocidad */
    float b[MIC_PROFILE_POINTS];
    b[n - 1] = lim[n - 1];
    for (int i = n - 2; i >= 0; --i) {
        const float a = m->cfg.plan_decel_frac * cap_decel(m, b[i + 1], decel);
        b[i] = fminf(lim[i], sqrtf(b[i + 1] * b[i + 1] + 2.0f * a * ds));
    }
    /* paso hacia delante: la aceleracion real disponible, desde la velocidad actual */
    const float v0 = fminf(fabsf(m->v), b[0]);
    m->prof[0] = v0;
    for (int i = 0; i < n - 1; ++i) {
        const float a = m->cfg.plan_accel_frac * cap_accel(m, m->prof[i]);
        m->prof[i + 1] = fminf(b[i + 1], sqrtf(m->prof[i] * m->prof[i] + 2.0f * a * ds));
    }
    m->nprof = n;
    m->route_len = total;
    m->route_ds = ds;
    m->route_v_end = v_end;
    m->route_s0 = m->s;
    m->decel = decel;
    m->mode = MIC_MODE_ROUTE;
    m->fresh = true;
    m->done_flag = false;
    m->emergency_latched = false;
    return true;
}

void mic_request_move(mic_t *m, float dist_cm, float v_cruise_cm_s, float v_end_cm_s, mic_decel_t decel)
{
    /* Sentido atras: se planea en el sentido de la marcha y el signo se aplica a la salida. */
    mic_route_begin(m);
    mic_route_add(m, fabsf(dist_cm), fmaxf(v_cruise_cm_s, m->cfg.v_creep_cm_s), 0.0f);
    mic_route_commit(m, v_end_cm_s, decel);
    m->v_target = dist_cm < 0.0f ? -1.0f : 1.0f;   /* en modo ruta guarda el sentido */
}

/* ------------------------------------------------------------------ ciclo */

static void hold_out(mic_t *m, mic_out_t *out)
{
    m->integ = 0.0f;
    m->vm_last = 0.0f;
    out->duty = 0.0f;
    out->brake_short = true;
    out->phase = MIC_PHASE_HOLD;
}

void mic_update(mic_t *m, const mic_in_t *in, float dt, mic_out_t *out)
{
    const mic_cfg_t *k = &m->cfg;
    memset(out, 0, sizeof *out);
    if (!(dt > 0.0f)) { hold_out(m, out); return; }

    /* ---------------- observador: predice con el modelo, corrige con el encoder ---------------- */
    const float s_meas = (float)in->counts * m->cm_per_count * CM;
    if (!m->obs_init) { m->s = s_meas; m->v = 0.0f; m->obs_init = true; }
    {
        /* observador de tercer orden: posicion, velocidad y aceleracion NO modelada (carga, friccion real) */
        const float a = accel_model(m, m->vm_last, m->v) + m->d_est;
        float v_new = m->v + a * dt;
        if (fabsf(m->v) < 0.003f && fabsf(m->vm_last * 0.0f) < 1.0f && accel_model(m, m->vm_last, m->v) == 0.0f) v_new = 0.0f;
        else if (m->v != 0.0f && sgnf(v_new) != sgnf(m->v) && fabsf(m->vm_last) < 0.5f * k->vbat_nominal) v_new = 0.0f;
        m->s += 0.5f * (m->v + v_new) * dt;
        m->v = v_new;
        const float w = 2.0f * PI_F * k->obs_bw_hz;
        const float innov = s_meas - m->s;
        m->s += 3.0f * w * dt * innov;
        m->v += 3.0f * w * w * dt * innov;
        m->d_est = clampf(m->d_est + w * w * w * dt * innov, -4.0f, 4.0f);
    }
    const float vav = v_avail(m, in->vbat);
    out->pos_cm = m->s / CM;
    out->v_cm_s = m->v / CM;
    out->i_est_a = (m->vm_last - m->ke_lin * m->v) / k->r_ohm;
    out->disturbance_ms2 = m->d_est;

    if (m->mode == MIC_MODE_IDLE) { hold_out(m, out); out->done = true; return; }

    /* ---------------- referencia ---------------- */
    float dirf = 1.0f;
    float v_ref = 0.0f, a_ref = 0.0f, remaining = 0.0f;
    bool stopping = false, route_done = false;
    const float tol = k->pos_tol_cm * CM;

    if (m->mode == MIC_MODE_SPEED) {
        dirf = m->v_target != 0.0f ? sgnf(m->v_target) : (m->v != 0.0f ? sgnf(m->v) : 1.0f);
        const float vd = dirf * m->v;
        if (m->fresh) { m->v_ref = vd; m->integ = 0.0f; m->fresh = false; }
        const float goal = dirf * m->v_target;
        float step;
        if (goal > m->v_ref) step = m->cfg.plan_accel_frac * cap_accel(m, fmaxf(m->v_ref, 0.0f)) * dt;
        else step = -k->plan_decel_frac * cap_decel(m, fmaxf(m->v_ref, 0.0f), m->decel) * dt;
        const float prev = m->v_ref;
        m->v_ref = clampf(goal, prev - fabsf(step), prev + fabsf(step));
        v_ref = m->v_ref;
        a_ref = (v_ref - prev) / dt;
        stopping = fabsf(goal) < 0.003f;
        if (stopping && fabsf(m->v) <= k->v_stop_cm_s * CM && v_ref <= k->v_stop_cm_s * CM) {
            hold_out(m, out); out->done = true; return;
        }
    } else {
        dirf = m->v_target < 0.0f ? -1.0f : 1.0f;
        const float s_rel = dirf * (m->s - m->route_s0);
        const float vd = dirf * m->v;
        if (m->fresh) { m->integ = 0.0f; m->fresh = false; }
        remaining = m->route_len - s_rel;
        const float pos = clampf(s_rel / m->route_ds, 0.0f, (float)(m->nprof - 1) - 1e-3f);
        const int i0 = (int)pos;
        const float fr = pos - (float)i0;
        const float v0 = m->prof[i0], v1 = m->prof[i0 + 1];
        v_ref = v0 + fr * (v1 - v0);
        a_ref = (v1 * v1 - v0 * v0) / (2.0f * m->route_ds);
        if (s_rel <= 0.0f && v_ref < 1e-3f) a_ref = fmaxf(a_ref, 0.5f);
        const bool end_stop = m->route_v_end < 0.003f;
        if (remaining <= tol) {
            if (end_stop) {
                stopping = true;
                v_ref = 0.0f; a_ref = 0.0f;
                if (fabsf(vd) <= k->v_stop_cm_s * CM) { hold_out(m, out); out->done = true; out->remaining_cm = remaining / CM; return; }
            } else {
                route_done = true;
            }
        } else if (end_stop && v_ref < k->v_creep_cm_s * CM) {
            v_ref = k->v_creep_cm_s * CM;       /* aproximacion final: no se atasca en la friccion estatica */
            a_ref = 0.0f;
        }
        m->v_ref = v_ref;
        out->remaining_cm = remaining / CM;
        if (route_done) {                         /* la ruta termina en movimiento: seguir con esa velocidad */
            mic_request_speed(m, dirf * m->route_v_end / CM, MIC_DECEL_PROGRESSIVE);
            m->v_ref = vd; m->fresh = false; out->done = true;
            v_ref = vd; a_ref = 0.0f;
        }
    }

    /* ---------------- ley de control: dinamica inversa + correccion ---------------- */
    const float vd = dirf * m->v;
    const float ev = v_ref - vd;
    float a_cmd = a_ref + k->kv_gain * ev + k->ki_gain * m->integ;
    const float sgn_motion = fabsf(vd) > 0.004f ? sgnf(vd) : sgnf(v_ref > 0.0f ? v_ref : a_cmd);
    float f_need = m->m_eff * (a_cmd - dirf * m->d_est) + k->f_coulomb_n * sgn_motion + k->b_viscous_ns_m * vd;
    float vm = k->r_ohm * (f_need / m->kf_lin) + m->ke_lin * vd;

    /* ¿se llega al punto con la desaceleracion progresiva? si no, emergencia: frenado activo */
    bool plug_allowed = m->decel == MIC_DECEL_AGGRESSIVE;
    if (m->mode == MIC_MODE_ROUTE && m->route_v_end < 0.003f && vd > 0.02f) {
        const float d_need = vd * vd / (2.0f * cap_decel(m, vd, MIC_DECEL_PROGRESSIVE)) ;
        if (!plug_allowed && d_need > remaining + 0.004f && remaining > 0.0f) { plug_allowed = true; m->emergency_latched = true; }
    }
    /* Va mas rapido que el perfil y el perfil baja: el modelo se queda corto (masa, friccion, bateria).
     * Se llega al limite de velocidad aunque haga falta tension inversa un instante; queda registrado. */
    if (!plug_allowed && a_ref < -0.2f && (vd - v_ref) > MIC_OVERSPEED_PLUG_MS) { plug_allowed = true; m->emergency_latched = true; }
    out->emergency = m->emergency_latched;

    /* limites: corriente del puente y tension disponible */
    float vm_lo = m->ke_lin * vd - k->r_ohm * k->i_max_a, vm_hi = m->ke_lin * vd + k->r_ohm * k->i_max_a;
    vm_lo = fmaxf(vm_lo, -vav); vm_hi = fminf(vm_hi, vav);
    float vm_d = clampf(vm, vm_lo, vm_hi);
    /* sin tension inversa: la tension no cambia de signo respecto a la marcha */
    if (!plug_allowed) {
        if (vd > k->v_stop_cm_s * CM) vm_d = fmaxf(vm_d, 0.0f);
        else if (vd < -k->v_stop_cm_s * CM) vm_d = fminf(vm_d, 0.0f);
    }
    const bool saturated = (vm_d != vm) && sgnf(ev) == sgnf(vm - vm_d);
    if (!saturated) m->integ = clampf(m->integ + ev * dt, -3.0f / fmaxf(k->ki_gain, 1.0f), 3.0f / fmaxf(k->ki_gain, 1.0f));
    else m->integ *= 0.98f;

    const float vm_out = dirf * vm_d;
    m->vm_last = vm_out;
    out->duty = clampf(vm_out / vav, -1.0f, 1.0f);
    out->brake_short = fabsf(out->duty) < 1e-3f;
    out->v_ref_cm_s = v_ref / CM;
    const float i_est = (vm_out - m->ke_lin * m->v) / k->r_ohm;
    const float i_dir = dirf * i_est;
    if (vm_d < -1e-3f && vd > k->v_stop_cm_s * CM) out->phase = MIC_PHASE_PLUG;
    else if (i_dir < -0.15f && vd > k->v_stop_cm_s * CM) out->phase = MIC_PHASE_BRAKE;
    else if (i_dir < 0.15f && vd > k->v_stop_cm_s * CM) out->phase = MIC_PHASE_COAST;
    else out->phase = MIC_PHASE_DRIVE;
    out->done = out->done || route_done;
}

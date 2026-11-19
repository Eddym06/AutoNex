#include "odometry.h"

#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
#define PI_F 3.14159265358979f

static float wrap_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void odom_config_default(odom_config_t *cfg)
{
    /* Valores de partida (datos del equipo, sin confirmar): rueda de 5,5 cm y 11,5 cm entre los centros de las
     * ruedas motrices. El encoder entrega el angulo en grados de rueda. Se confirman con
     * odom_calibrate_* sobre el chasis real. */
    cfg->wheel_diameter_cm = 5.5f;
    cfg->track_width_cm = 11.5f;
    cfg->left_scale = 1.0f;
    cfg->right_scale = 1.0f;
    cfg->left_sign = 1;
    cfg->right_sign = 1;
    cfg->slip_yaw_dps = 40.0f;
    cfg->slip_min_dt_s = 0.01f;
    cfg->max_gap_s = 0.5f;
    cfg->max_speed_cm_s = 300.0f;     /* una muestra que implique mas es un error de lectura: la trayectoria inteligente llega a 150 cm/s, con 150 se descartaba distancia real */
}

static bool clamp_f(float *v, float lo, float hi)
{
    if (!(*v >= lo)) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}

bool odom_config_sanitize(odom_config_t *cfg)
{
    bool ch = false;
    ch |= clamp_f(&cfg->wheel_diameter_cm, 1.0f, 30.0f);
    ch |= clamp_f(&cfg->track_width_cm, 3.0f, 60.0f);
    ch |= clamp_f(&cfg->left_scale, 0.5f, 2.0f);
    ch |= clamp_f(&cfg->right_scale, 0.5f, 2.0f);
    if (cfg->left_sign != 1 && cfg->left_sign != -1) { cfg->left_sign = cfg->left_sign < 0 ? -1 : 1; ch = true; }
    if (cfg->right_sign != 1 && cfg->right_sign != -1) { cfg->right_sign = cfg->right_sign < 0 ? -1 : 1; ch = true; }
    ch |= clamp_f(&cfg->slip_yaw_dps, 5.0f, 720.0f);
    ch |= clamp_f(&cfg->slip_min_dt_s, 0.001f, 0.5f);
    ch |= clamp_f(&cfg->max_gap_s, 0.05f, 5.0f);
    ch |= clamp_f(&cfg->max_speed_cm_s, 20.0f, 1000.0f);
    return ch;
}

void odom_init(odom_t *o, const odom_config_t *cfg)
{
    memset(o, 0, sizeof(*o));
    if (cfg) o->cfg = *cfg; else odom_config_default(&o->cfg);
    odom_config_sanitize(&o->cfg);
}

void odom_reset_pose(odom_t *o, float x_cm, float y_cm, float heading_deg)
{
    o->x_cm = x_cm;
    o->y_cm = y_cm;
    o->heading_deg = heading_deg;
}

float odom_wheel_to_cm(const odom_t *o, float deg, bool left)
{
    const float scale = left ? o->cfg.left_scale : o->cfg.right_scale;
    const int sign = left ? o->cfg.left_sign : o->cfg.right_sign;
    return (float)sign * deg * (PI_F * o->cfg.wheel_diameter_cm / 360.0f) * scale;
}

float odom_update(odom_t *o, uint64_t t_us, float left_deg, float right_deg, float yaw_deg, bool yaw_valid)
{
    if (!isfinite(left_deg) || !isfinite(right_deg)) { ++o->rejected; return 0.0f; }
    if (!o->have_prev || t_us <= o->t_prev_us || (float)(t_us - o->t_prev_us) * 1.0e-6f > o->cfg.max_gap_s) {
        /* Primera muestra, reloj atrasado o un hueco largo: se toma como nueva referencia sin integrar. */
        o->have_prev = true;
        o->t_prev_us = t_us;
        o->left_prev_deg = left_deg;
        o->right_prev_deg = right_deg;
        o->yaw_prev_deg = yaw_deg;
        o->yaw_prev_valid = yaw_valid;
        if (yaw_valid) o->heading_deg = yaw_deg;
        o->slipping = false;
        return 0.0f;
    }
    const float dt = (float)(t_us - o->t_prev_us) * 1.0e-6f;
    const float dl = odom_wheel_to_cm(o, left_deg - o->left_prev_deg, true);
    const float dr = odom_wheel_to_cm(o, right_deg - o->right_prev_deg, false);
    o->t_prev_us = t_us;
    o->left_prev_deg = left_deg;
    o->right_prev_deg = right_deg;
    ++o->samples;
    if (fabsf(0.5f * (dl + dr)) / dt > o->cfg.max_speed_cm_s || fabsf(dl) / dt > 2.0f * o->cfg.max_speed_cm_s ||
        fabsf(dr) / dt > 2.0f * o->cfg.max_speed_cm_s) {
        ++o->rejected;           /* lectura imposible (salto del contador): no se integra */
        o->yaw_prev_deg = yaw_deg;
        o->yaw_prev_valid = yaw_valid;
        return 0.0f;
    }

    const float track = o->cfg.track_width_cm;
    const float dth_enc = (dl - dr) / track * RAD2DEG;                 /* giro segun encoders (grados, horario) */
    const bool use_gyro = yaw_valid && o->yaw_prev_valid;
    const float dth_gyro = use_gyro ? wrap_deg(yaw_deg - o->yaw_prev_deg) : dth_enc;
    float ds = 0.5f * (dl + dr);
    o->slipping = false;
    if (use_gyro && dt >= o->cfg.slip_min_dt_s && fabsf(dth_enc - dth_gyro) / dt > o->cfg.slip_yaw_dps) {
        /* Una rueda gira mas de lo que el robot se mueve. Con el giro real (giroscopio) cada rueda propone un
         * avance del eje; la que patina sobreestima, asi que se toma el menor. */
        const float half = 0.5f * track * dth_gyro * DEG2RAD;
        const float from_left = dl - half, from_right = dr + half;
        const float pick = fabsf(from_left) < fabsf(from_right) ? from_left : from_right;
        o->slip_cm += fabsf(ds - pick);
        ++o->slip_count;
        o->slipping = true;
        ds = pick;
    }
    const float h0 = use_gyro ? o->yaw_prev_deg : o->heading_deg;
    const float h1 = use_gyro ? yaw_deg : o->heading_deg + dth_enc;
    const float mid = (h0 + 0.5f * wrap_deg(h1 - h0)) * DEG2RAD;
    o->x_cm += ds * sinf(mid);
    o->y_cm += ds * cosf(mid);
    o->heading_deg = wrap_deg(h1);
    o->path_cm += fabsf(ds);
    const float a = 1.0f - expf(-dt / 0.05f);
    o->v_cm_s += a * (ds / dt - o->v_cm_s);
    o->omega_dps += a * (dth_gyro / dt - o->omega_dps);
    o->yaw_prev_deg = yaw_deg;
    o->yaw_prev_valid = yaw_valid;
    return ds;
}

float odom_update_axle(odom_t *o, uint64_t t_us, float axle_deg, float yaw_deg, bool yaw_valid)
{
    if (!o || !isfinite(axle_deg) || (yaw_valid && !isfinite(yaw_deg))) return 0.0f;
    if (!o->have_prev || t_us <= o->t_prev_us || (float)(t_us - o->t_prev_us) * 1e-6f > o->cfg.max_gap_s) {
        o->have_prev = true;
        o->t_prev_us = t_us;
        o->left_prev_deg = axle_deg;
        o->yaw_prev_deg = yaw_deg;
        o->yaw_prev_valid = yaw_valid;
        if (yaw_valid) o->heading_deg = yaw_deg;
        o->slipping = false;
        return 0.0f;
    }
    const float dt = (float)(t_us - o->t_prev_us) * 1e-6f;
    const float ds = odom_wheel_to_cm(o, axle_deg - o->left_prev_deg, true);
    o->t_prev_us = t_us;
    o->left_prev_deg = axle_deg;
    ++o->samples;
    if (fabsf(ds) / dt > o->cfg.max_speed_cm_s) {
        ++o->rejected;
        o->yaw_prev_deg = yaw_deg;
        o->yaw_prev_valid = yaw_valid;
        return 0.0f;
    }
    const bool gyro = yaw_valid && o->yaw_prev_valid;
    const float h0 = gyro ? o->yaw_prev_deg : o->heading_deg;
    const float h1 = yaw_valid ? yaw_deg : o->heading_deg;
    const float dth = gyro ? wrap_deg(h1 - h0) : 0.0f;
    const float mid = (h0 + 0.5f * dth) * DEG2RAD;
    o->x_cm += ds * sinf(mid);
    o->y_cm += ds * cosf(mid);
    o->heading_deg = wrap_deg(h1);
    o->path_cm += fabsf(ds);
    const float a = 1.0f - expf(-dt / 0.05f);
    o->v_cm_s += a * (ds / dt - o->v_cm_s);
    o->omega_dps += a * (dth / dt - o->omega_dps);
    o->slipping = false;
    o->yaw_prev_deg = yaw_deg;
    o->yaw_prev_valid = yaw_valid;
    return ds;
}

bool odom_calibrate_distance(odom_config_t *cfg, float enc_cm, float true_cm)
{
    if (!(fabsf(enc_cm) > 20.0f) || !(fabsf(true_cm) > 20.0f) || (enc_cm > 0.0f) != (true_cm > 0.0f)) return false;
    const float k = true_cm / enc_cm;
    if (!(k > 0.5f && k < 2.0f)) return false;
    cfg->left_scale *= k;
    cfg->right_scale *= k;
    odom_config_sanitize(cfg);
    return true;
}

bool odom_calibrate_track(odom_config_t *cfg, float enc_diff_cm, float gyro_deg)
{
    if (!(fabsf(gyro_deg) >= 90.0f) || (enc_diff_cm > 0.0f) != (gyro_deg > 0.0f)) return false;
    const float track = enc_diff_cm / (gyro_deg * DEG2RAD);
    if (!(track >= 3.0f && track <= 60.0f)) return false;
    cfg->track_width_cm = track;
    return true;
}

bool odom_calibrate_balance(odom_config_t *cfg, float left_cm, float right_cm, float gyro_deg)
{
    const float sum = left_cm + right_cm;
    if (!(sum > 50.0f)) return false;
    const float delta = ((left_cm - right_cm) - gyro_deg * DEG2RAD * cfg->track_width_cm) / sum;
    if (!(fabsf(delta) < 0.2f)) return false;
    cfg->left_scale *= 1.0f - delta;
    cfg->right_scale *= 1.0f + delta;
    odom_config_sanitize(cfg);
    return true;
}

#include "map_pose.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define MATCH_FAR_CM 20.0f        /* un par solo cuenta si los puntos estan a menos de esto */
#define MATCH_NEAR_CM 8.0f        /* y en las ultimas iteraciones, a menos de esto */
#define MIN_MATCHES 30
#define MAX_SHIFT_CM 15.0f        /* movimiento maximo creible entre dos barridos (1,5 m/s a 10 Hz) */
#define DEAD_ZONE_CM 0.4f         /* correcciones menores se ignoran: sin esto el ruido hace derivar al robot quieto */
#define MIN_RANGE_CM 10.0f
#define MAX_RANGE_CM 500.0f

static float wrap180(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* Normales de los puntos de referencia: perpendicular a la recta entre los vecinos del barrido (si estan cerca). */
static void compute_normals(map_pose_t *m)
{
    for (int j = 0; j < m->prev_count; ++j) {
        m->prev_ne[j] = m->prev_nn[j] = 0.0f;
        if (j == 0 || j == m->prev_count - 1) continue;
        const float te = m->prev_e[j + 1] - m->prev_e[j - 1], tn = m->prev_n[j + 1] - m->prev_n[j - 1];
        const float l = sqrtf(te * te + tn * tn);
        const float a = hypotf(m->prev_e[j] - m->prev_e[j - 1], m->prev_n[j] - m->prev_n[j - 1]);
        const float b = hypotf(m->prev_e[j + 1] - m->prev_e[j], m->prev_n[j + 1] - m->prev_n[j]);
        if (l < 1e-3f || a > 15.0f || b > 15.0f) continue;      /* borde de un objeto: no es una linea */
        m->prev_ne[j] = -tn / l;
        m->prev_nn[j] = te / l;
    }
}

void map_pose_reset(map_pose_t *m)
{
    const uint64_t t = m->last_gyro_us;
    memset(m, 0, sizeof *m);
    m->last_gyro_us = t;
}

void map_pose_gyro(map_pose_t *m, float dps, uint64_t t_us)
{
    if (m->last_gyro_us && t_us > m->last_gyro_us) {
        const float dt = (float)(t_us - m->last_gyro_us) * 1e-6f;
        if (dt < 0.5f) {                                                      /* un hueco largo no se integra */
            m->heading_deg = wrap180(m->heading_deg + (dps - m->bias_dps) * dt);
            m->ref_dps_sum += dps * dt;
            m->ref_time_s += dt;
            m->rot_time_s += dt;
        }
    }
    m->last_gyro_us = t_us;
}

/* Puntos del barrido en el marco del mundo (este, norte) para una pose dada. */
static int to_world(const local_lidar_point_t *pts, size_t count, float heading_deg, float e0, float n0,
                    float *oe, float *on)
{
    const float th = heading_deg * PI_F / 180.0f, s = sinf(th), c = cosf(th);
    const size_t stride = count > MAP_POSE_MAX_POINTS ? (count + MAP_POSE_MAX_POINTS - 1) / MAP_POSE_MAX_POINTS : 1;
    int k = 0;
    for (size_t i = 0; i < count && k < MAP_POSE_MAX_POINTS; i += stride) {
        if (!pts[i].valid || pts[i].distance_cm < MIN_RANGE_CM || pts[i].distance_cm > MAX_RANGE_CM) continue;
        const float a = pts[i].angle_deg * PI_F / 180.0f;
        const float xr = pts[i].distance_cm * sinf(a);       /* a la derecha del robot */
        const float yr = pts[i].distance_cm * cosf(a);       /* al frente */
        oe[k] = e0 + yr * s + xr * c;
        on[k] = n0 + yr * c - xr * s;
        ++k;
    }
    return k;
}

/* Perfil de distancias: distancia media por cada grado (angulo desde el frente, 0..359), -1 si no hay punto. */
static void make_profile(const local_lidar_point_t *p, size_t n, float prof[MAP_POSE_BINS])
{
    static float sum[MAP_POSE_BINS];          /* estaticos: la pila de la tarea no aguanta arreglos de este tamano */
    static int cnt[MAP_POSE_BINS];
    memset(sum, 0, sizeof sum);
    memset(cnt, 0, sizeof cnt);
    for (size_t i = 0; i < n; ++i) {
        if (!p[i].valid || p[i].distance_cm < MIN_RANGE_CM || p[i].distance_cm > MAX_RANGE_CM) continue;
        int b = (int)floorf(p[i].angle_deg + 360.0f) % MAP_POSE_BINS;
        sum[b] += p[i].distance_cm;
        ++cnt[b];
    }
    for (int b = 0; b < MAP_POSE_BINS; ++b) prof[b] = cnt[b] ? sum[b] / (float)cnt[b] : -1.0f;
}

/* Diferencia media entre now[a] y ref[a + shift] en los grados donde ambos tienen dato. */
static float profile_mad(const float now[MAP_POSE_BINS], const float ref[MAP_POSE_BINS], int shift, int *count)
{
    float s = 0.0f;
    int n = 0;
    for (int a = 0; a < MAP_POSE_BINS; ++a) {
        const float x = now[a], y = ref[((a + shift) % MAP_POSE_BINS + MAP_POSE_BINS) % MAP_POSE_BINS];
        if (x < 0.0f || y < 0.0f) continue;
        s += fabsf(x - y);
        ++n;
    }
    *count = n;
    return n ? s / (float)n : 1e9f;
}

static void gyro_calibration_check(map_pose_t *m, const local_lidar_point_t *pts, size_t count)
{
    static float prof[MAP_POSE_BINS];
    make_profile(pts, count, prof);
    int n;

    /* 1) Reposo: el perfil no cambia durante >= 1,5 s => la lectura media del giroscopio es sesgo. */
    if (!m->ref_valid) {
        memcpy(m->ref_prof, prof, sizeof prof); m->ref_valid = 1; m->ref_dps_sum = 0.0f; m->ref_time_s = 0.0f;
    } else {
        const float mad = profile_mad(prof, m->ref_prof, 0, &n);
        if (n < 120 || mad > 2.5f) {                          /* se movio: empezar de nuevo */
            memcpy(m->ref_prof, prof, sizeof prof); m->ref_dps_sum = 0.0f; m->ref_time_s = 0.0f;
        } else if (m->ref_time_s >= 1.5f) {
            const float raw_mean = m->ref_dps_sum / m->ref_time_s;
            if (mad < 1.2f && fabsf(raw_mean) < 3.0f) {
                m->bias_dps += 0.5f * (raw_mean - m->bias_dps);
                ++m->bias_updates;
            }
            memcpy(m->ref_prof, prof, sizeof prof); m->ref_dps_sum = 0.0f; m->ref_time_s = 0.0f;
        }
    }

    /* 2) Giro: cuanto giro el LiDAR (desplazamiento del perfil) frente a lo que integro el giroscopio. */
    if (!m->rot_valid) {
        memcpy(m->rot_prof, prof, sizeof prof); m->rot_valid = 1; m->rot_heading_deg = m->heading_deg; m->rot_time_s = 0.0f;
        return;
    }
    const float dh = wrap180(m->heading_deg - m->rot_heading_deg);
    if (fabsf(dh) >= 3.0f) {
        int best_s = 0, best_n = 0, n0 = 0;
        float best = 1e9f;
        const float mad0 = profile_mad(prof, m->rot_prof, 0, &n0);
        for (int s = -15; s <= 15; ++s) {
            const float v = profile_mad(prof, m->rot_prof, s, &n);
            if (n >= 150 && v < best) { best = v; best_s = s; best_n = n; }
        }
        /* un giro horario hace que lo fijo aparezca a la izquierda: now[a] = ref[a + s] con s = giro horario */
        if (best_n && abs(best_s) >= 2 && best < 0.6f * mad0 && best < 8.0f) {
            if ((best_s > 0) == (dh > 0.0f)) { ++m->votes_ok; m->votes_flip = 0; }
            else { ++m->votes_flip; m->votes_ok = 0; }
            if (m->votes_flip >= 3) { m->flip_request = 1; m->votes_flip = 0; }
        }
        memcpy(m->rot_prof, prof, sizeof prof); m->rot_heading_deg = m->heading_deg; m->rot_time_s = 0.0f;
    } else if (m->rot_time_s > 2.0f) {
        memcpy(m->rot_prof, prof, sizeof prof); m->rot_heading_deg = m->heading_deg; m->rot_time_s = 0.0f;
    }
}

void map_pose_scan(map_pose_t *m, const local_lidar_point_t *points, size_t count, float odom_forward_cm)
{
    const float th = m->heading_deg * PI_F / 180.0f;
    float ge = m->e_cm + odom_forward_cm * sinf(th), gn = m->n_cm + odom_forward_cm * cosf(th);   /* estimacion inicial */
    static float ce[MAP_POSE_MAX_POINTS], cn[MAP_POSE_MAX_POINTS];     /* estaticos: ver make_profile */
    const int n = to_world(points, count, m->heading_deg, ge, gn, ce, cn);
    m->last_matches = 0;
    m->last_shift_cm = 0.0f;
    int keep_reference = 0;       /* quieto: se conserva el barrido de referencia y el movimiento lento se va acumulando */

    if (m->have_prev && n >= MIN_MATCHES && m->prev_count >= MIN_MATCHES) {
        float te = 0.0f, tn = 0.0f;
        int matches = 0;
        for (int it = 0; it < 12; ++it) {
            const float gate = it < 5 ? MATCH_FAR_CM : MATCH_NEAR_CM, gate2 = gate * gate;
            float aee = 0.0f, aen = 0.0f, ann = 0.0f, be = 0.0f, bn = 0.0f;
            matches = 0;
            for (int i = 0; i < n; ++i) {
                const float qe = ce[i] + te, qn = cn[i] + tn;
                float best = gate2;
                int bj = -1;
                for (int j = 0; j < m->prev_count; ++j) {
                    const float de = m->prev_e[j] - qe, dn = m->prev_n[j] - qn, d2 = de * de + dn * dn;
                    if (d2 < best) { best = d2; bj = j; }
                }
                if (bj < 0 || (m->prev_ne[bj] == 0.0f && m->prev_nn[bj] == 0.0f)) continue;
                const float ne = m->prev_ne[bj], nn = m->prev_nn[bj];
                const float r = ne * (m->prev_e[bj] - qe) + nn * (m->prev_n[bj] - qn);   /* distancia a la recta */
                aee += ne * ne; aen += ne * nn; ann += nn * nn;
                be += ne * r; bn += nn * r;
                ++matches;
            }
            if (matches < MIN_MATCHES) { matches = 0; break; }
            const float lam = 0.02f * (float)matches;      /* regularizacion: un pasillo no fija el avance a lo largo de el */
            aee += lam; ann += lam;
            const float det = aee * ann - aen * aen;
            const float se = (ann * be - aen * bn) / det, sn = (aee * bn - aen * be) / det;
            te += se;
            tn += sn;
            if (se * se + sn * sn < 0.0025f) break;          /* convergio (< 0,5 mm) */
        }
        const float shift = sqrtf(te * te + tn * tn);
        if (matches >= MIN_MATCHES && shift <= MAX_SHIFT_CM) {
            m->last_matches = matches;
            m->last_shift_cm = shift;
            if (shift >= DEAD_ZONE_CM) { ge += te; gn += tn; }
            else keep_reference = 1;
        }                                                    /* si no alinea, la referencia pasa a ser este barrido */
    }
    m->e_cm = ge;
    m->n_cm = gn;
    gyro_calibration_check(m, points, count);
    if (keep_reference) return;
    m->prev_count = to_world(points, count, m->heading_deg, ge, gn, m->prev_e, m->prev_n);
    compute_normals(m);
    m->have_prev = m->prev_count >= MIN_MATCHES;
}

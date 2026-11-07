#include "park_localize.h"

#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define TAU_CM 1.5f          /* distancia maxima de un punto al modelo para contar */
#define MAX_RANGE_CM 100.0f

static float g_x[LOCAL_MAP_MAX_POINTS], g_f[LOCAL_MAP_MAX_POINTS];
static int g_n;

typedef struct { float score; int wall, rear, front; } eval_t;

static float lot_len(const park_params_t *p)
{
    return p->lot_length_cm > 0.0f ? p->lot_length_cm : 1.5f * p->robot_length_cm;
}

static void load_points(const park_loc_t *loc, const local_lidar_point_t *pts, size_t count)
{
    g_n = 0;
    for (size_t i = 0; i < count && g_n < LOCAL_MAP_MAX_POINTS; ++i) {
        if (!pts[i].valid || !(pts[i].distance_cm > 0.0f) || pts[i].distance_cm > MAX_RANGE_CM) continue;
        const float a = pts[i].angle_deg * DEG2RAD;
        g_x[g_n] = (float)loc->mirror * pts[i].distance_cm * sinf(a);
        g_f[g_n] = loc->lidar_forward_cm + pts[i].distance_cm * cosf(a);
        ++g_n;
    }
}

static float rect_dist(float u, float w, float x0, float x1, float y0, float y1)
{
    const float dx = fmaxf(fmaxf(x0 - u, 0.0f), u - x1), dy = fmaxf(fmaxf(y0 - w, 0.0f), w - y1);
    if (dx > 0.0f || dy > 0.0f) return hypotf(dx, dy);
    return fminf(fminf(u - x0, x1 - u), fminf(w - y0, y1 - w));
}

static eval_t evaluate(const park_params_t *p, float u, float w, float th_deg)
{
    eval_t e = {0.0f, 0, 0, 0};
    const float th = th_deg * DEG2RAD, c = cosf(th), s = sinf(th);
    const float half = lot_len(p) * 0.5f, t = p->limiter_thickness_cm, W = p->lot_width_cm;
    for (int i = 0; i < g_n; ++i) {
        const float pu = u + g_f[i] * c + g_x[i] * s;
        const float pw = w + g_f[i] * s - g_x[i] * c;
        if (pw < -2.5f || pw > 70.0f) continue;
        const float d_wall = fabsf(pw);
        const float d_rear = rect_dist(pu, pw, -half - t, -half, 0.0f, W);
        const float d_front = rect_dist(pu, pw, half, half + t, 0.0f, W);
        float d = d_wall;
        int kind = 0;
        if (d_rear < d) { d = d_rear; kind = 1; }
        if (d_front < d) { d = d_front; kind = 2; }
        if (d >= TAU_CM) continue;
        const float r = d / TAU_CM;
        e.score += 1.0f - r * r;
        if (kind == 0) ++e.wall; else if (kind == 1) ++e.rear; else ++e.front;
    }
    return e;
}

static float penalty(float du, float dw, float dth)
{
    return 0.02f * du * du + 0.05f * dw * dw + 0.01f * dth * dth;
}

static void best_around(const park_params_t *p, const park_pose_t *center, const park_pose_t *prior,
                        float su, float sw, float sth, int nu, int nw, int nth, park_fix_t *fix)
{
    float best = -1e9f;
    for (int a = -nu; a <= nu; ++a)
        for (int b = -nw; b <= nw; ++b)
            for (int d = -nth; d <= nth; ++d) {
                park_pose_t q = {center->u_cm + (float)a * su, center->w_cm + (float)b * sw,
                                 center->theta_deg + (float)d * sth};
                const eval_t e = evaluate(p, q.u_cm, q.w_cm, q.theta_deg);
                const float score = e.score - penalty(q.u_cm - prior->u_cm, q.w_cm - prior->w_cm,
                                                      q.theta_deg - prior->theta_deg);
                if (score > best) {
                    best = score;
                    fix->pose = q; fix->score = e.score;
                    fix->wall_hits = e.wall; fix->rear_hits = e.rear; fix->front_hits = e.front;
                }
            }
}

bool park_localize_track(const park_loc_t *loc, const local_lidar_point_t *points, size_t count,
                         const park_pose_t *prior, park_fix_t *fix)
{
    if (!loc || !loc->params || !points || !prior || !fix) return false;
    memset(fix, 0, sizeof(*fix));
    load_points(loc, points, count);
    if (g_n < 20) return false;
    best_around(loc->params, prior, prior, 2.0f, 2.0f, 5.0f, 3, 2, 2, fix);
    park_pose_t coarse = fix->pose;
    best_around(loc->params, &coarse, prior, 0.75f, 0.75f, 1.5f, 2, 2, 2, fix);
    fix->valid = fix->score >= 14.0f && fix->wall_hits >= 8;
    return fix->valid;
}

bool park_localize_acquire(const park_loc_t *loc, const local_lidar_point_t *points, size_t count,
                           float u_max_cm, park_fix_t *fix)
{
    if (!loc || !loc->params || !points || !fix) return false;
    memset(fix, 0, sizeof(*fix));
    load_points(loc, points, count);
    if (g_n < 30) return false;
    /* Etapa 1: la pared. Rumbo y distancia que dejan mas puntos a menos de 0,8 cm de una recta. */
    int best_n = 0;
    float best_w = 0.0f, best_th = 0.0f;
    for (float th_deg = -30.0f; th_deg <= 30.0f; th_deg += 3.0f) {
        const float th = th_deg * DEG2RAD, c = cosf(th), s = sinf(th);
        int hist[56];
        memset(hist, 0, sizeof(hist));
        for (int i = 0; i < g_n; ++i) {
            const float pw = g_f[i] * s - g_x[i] * c;       /* w - w0: la distancia de la recta al eje */
            const int bin = (int)floorf(-pw + 0.5f);
            if (bin >= 1 && bin < 52) { ++hist[bin - 1]; ++hist[bin]; ++hist[bin + 1]; }
        }
        for (int b = 2; b < 52; ++b)
            if (hist[b] > best_n) { best_n = hist[b]; best_w = (float)b; best_th = th_deg; }
    }
    if (best_n < 15) return false;
    /* Etapa 2: la posicion a lo largo de la pared, con los topes. */
    park_pose_t prior = {-25.0f, best_w, best_th};
    float best = -1e9f;
    park_fix_t cand;
    memset(&cand, 0, sizeof(cand));
    for (float u = -45.0f; u <= u_max_cm; u += 1.5f)
        for (int b = -2; b <= 2; ++b)
            for (int d = -2; d <= 2; ++d) {
                park_pose_t q = {u, best_w + (float)b * 0.75f, best_th + (float)d * 1.5f};
                const eval_t e = evaluate(loc->params, q.u_cm, q.w_cm, q.theta_deg);
                /* Se llega por detras: el primer tope que se ve es el trasero. Sin esta condicion un solo
                 * tope se puede tomar por el delantero y el cajon queda 27 cm mas lejos de lo que esta. */
                const int limiter = e.rear + e.front;
                if (limiter < 6 || (e.rear < 5 && e.front < 5) || (e.rear < e.front && e.rear < 5)) continue;
                const float score = e.score + 0.5f * (float)limiter;
                if (score > best) {
                    best = score;
                    cand.pose = q; cand.score = e.score;
                    cand.wall_hits = e.wall; cand.rear_hits = e.rear; cand.front_hits = e.front;
                }
            }
    if (best < 0.0f) return false;
    *fix = cand;
    best_around(loc->params, &cand.pose, &cand.pose, 0.5f, 0.5f, 1.0f, 3, 2, 2, fix);
    /* Un tope visto a menos de ~1 m deja unos 15 puntos; con menos es coincidencia con otra cosa. */
    const bool behind = fix->rear_hits >= 8 && fix->rear_hits >= fix->front_hits;   /* solo se ve el trasero (o mas) */
    const bool both = fix->rear_hits >= 6 && fix->front_hits >= 6;                  /* se ven los dos: sin ambiguedad */
    fix->valid = fix->wall_hits >= 10 && (behind || both) && (fix->rear_hits + fix->front_hits) >= 10 &&
                 fix->score >= 20.0f;
    (void)prior;
    return fix->valid;
}

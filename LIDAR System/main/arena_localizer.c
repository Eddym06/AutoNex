#include "arena_localizer.h"

#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
#define MAX_PTS 400

typedef struct {
    bool vertical;           /* true: la pared es x = pos (normal en x); false: z = pos */
    float pos;
    float lo, hi;            /* extension a lo largo de la pared */
} wall_t;

static float wrap_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

void arena_config_default(arena_config_t *c)
{
    c->outer_half_cm = 150.0f;
    c->inner_half_cm = 50.0f;
    c->tol_cm = 6.0f;
    c->gain = 0.6f;
    c->min_support = 12;
    c->max_shift_cm = 30.0f;
    c->max_rot_deg = 8.0f;
    c->min_range_cm = 8.0f;
    c->max_range_cm = 300.0f;
    c->init_min_score = 0.45f;
}

static bool clampf_(float *v, float lo, float hi)
{
    if (!(*v >= lo)) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}

bool arena_config_sanitize(arena_config_t *c)
{
    bool ch = false;
    ch |= clampf_(&c->outer_half_cm, 60.0f, 1000.0f);
    ch |= clampf_(&c->inner_half_cm, 10.0f, c->outer_half_cm - 30.0f);
    ch |= clampf_(&c->tol_cm, 1.0f, 20.0f);
    ch |= clampf_(&c->gain, 0.05f, 1.0f);
    if (c->min_support < 3) { c->min_support = 3; ch = true; }
    ch |= clampf_(&c->max_shift_cm, 2.0f, 100.0f);
    ch |= clampf_(&c->max_rot_deg, 1.0f, 30.0f);
    ch |= clampf_(&c->min_range_cm, 0.0f, 50.0f);
    ch |= clampf_(&c->max_range_cm, 50.0f, 1000.0f);
    ch |= clampf_(&c->init_min_score, 0.1f, 0.95f);
    return ch;
}

void arena_init(arena_loc_t *a, const arena_config_t *cfg)
{
    memset(a, 0, sizeof(*a));
    if (cfg) a->cfg = *cfg; else arena_config_default(&a->cfg);
    arena_config_sanitize(&a->cfg);
}

static void build_walls(const arena_config_t *c, wall_t w[8])
{
    const float O = c->outer_half_cm, I = c->inner_half_cm;
    w[0] = (wall_t){true, O, -O, O};   w[1] = (wall_t){true, -O, -O, O};
    w[2] = (wall_t){false, O, -O, O};  w[3] = (wall_t){false, -O, -O, O};
    w[4] = (wall_t){true, I, -I, I};   w[5] = (wall_t){true, -I, -I, I};
    w[6] = (wall_t){false, I, -I, I};  w[7] = (wall_t){false, -I, -I, I};
}

void arena_to_world(const arena_pose_t *p, float qx, float qf, float *wx, float *wz)
{
    const float th = p->theta_deg * DEG2RAD, s = sinf(th), c = cosf(th);
    *wx = p->x_cm + qx * c + qf * s;
    *wz = p->z_cm - qx * s + qf * c;
}

void arena_to_robot(const arena_pose_t *p, float wx, float wz, float *qx, float *qf)
{
    const float th = p->theta_deg * DEG2RAD, s = sinf(th), c = cosf(th);
    const float dx = wx - p->x_cm, dz = wz - p->z_cm;
    *qx = dx * c - dz * s;
    *qf = dx * s + dz * c;
}

/* La pared mas cercana a un punto del campo: devuelve la distancia con signo (normal - posicion) y su indice. */
static int nearest_wall(const wall_t w[8], float px, float pz, float tol, float *resid)
{
    int best = -1;
    float best_abs = tol;
    for (int i = 0; i < 8; ++i) {
        const float across = w[i].vertical ? px : pz, along = w[i].vertical ? pz : px;
        if (along < w[i].lo - tol || along > w[i].hi + tol) continue;
        const float r = across - w[i].pos;
        if (fabsf(r) < best_abs) { best_abs = fabsf(r); best = i; *resid = r; }
    }
    return best;
}

/* Puntos validos del barrido en el marco del robot (eje de las ruedas). */
static size_t robot_points(const arena_config_t *c, const arena_point_t *pts, size_t n, float lidar_forward_cm,
                           float *qx, float *qf)
{
    size_t m = 0;
    for (size_t i = 0; i < n && m < MAX_PTS; ++i) {
        if (!pts[i].valid || !(pts[i].distance_cm >= c->min_range_cm) || pts[i].distance_cm > c->max_range_cm) continue;
        const float a = pts[i].angle_deg * DEG2RAD;
        qx[m] = pts[i].distance_cm * sinf(a);
        qf[m] = pts[i].distance_cm * cosf(a) + lidar_forward_cm;
        ++m;
    }
    return m;
}

/* Un paso de Gauss-Newton robusto. Devuelve cuantos puntos casaron; rellena delta (dx, dz, dtheta en rad), el
 * respaldo por eje y el residuo cuadratico medio. */
static int gn_step(const arena_config_t *c, const wall_t w[8], const arena_pose_t *p, const float *qx, const float *qf,
                   size_t m, float tol, float delta[3], float sup[3], float *rms)
{
    float H[3][3] = {{0}}, g[3] = {0};
    const float th = p->theta_deg * DEG2RAD, s = sinf(th), co = cosf(th);
    int inl = 0;
    float sq = 0.0f, wsum = 0.0f;
    sup[0] = sup[1] = sup[2] = 0.0f;
    for (size_t i = 0; i < m; ++i) {
        float px, pz;
        arena_to_world(p, qx[i], qf[i], &px, &pz);
        float r = 0.0f;
        const int wi = nearest_wall(w, px, pz, tol, &r);
        if (wi < 0) continue;
        ++inl;
        const float ar = fabsf(r), half = 0.5f * tol;
        const float wt = ar <= half ? 1.0f : half / ar;          /* Huber */
        const float dpx = qf[i] * co - qx[i] * s, dpz = -qx[i] * co - qf[i] * s;
        const float nx = w[wi].vertical ? 1.0f : 0.0f, nz = w[wi].vertical ? 0.0f : 1.0f;
        const float J[3] = {nx, nz, nx * dpx + nz * dpz};
        for (int a = 0; a < 3; ++a) {
            g[a] += wt * J[a] * r;
            for (int b = 0; b < 3; ++b) H[a][b] += wt * J[a] * J[b];
        }
        sup[0] += wt * nx * nx; sup[1] += wt * nz * nz;
        sup[2] += wt * (fabsf(J[2]) > 1.0f ? 1.0f : 0.0f);
        sq += wt * r * r; wsum += wt;
    }
    *rms = wsum > 0.0f ? sqrtf(sq / wsum) : 0.0f;
    delta[0] = delta[1] = delta[2] = 0.0f;
    if (inl < 3) return inl;
    /* Solo los parametros con respaldo suficiente entran en el sistema. */
    int idx[3], k = 0;
    const float need = (float)c->min_support;
    if (sup[0] >= need) idx[k++] = 0;
    if (sup[1] >= need) idx[k++] = 1;
    if (sup[2] >= need) idx[k++] = 2;
    if (k == 0) return inl;
    float A[3][4];
    for (int a = 0; a < k; ++a) {
        for (int b = 0; b < k; ++b) A[a][b] = H[idx[a]][idx[b]];
        A[a][a] += 1.0e-3f + (idx[a] == 2 ? 50.0f : 0.5f);       /* amortiguacion: la rotacion se mueve poco */
        A[a][3] = -g[idx[a]];
    }
    for (int col = 0; col < k; ++col) {
        int piv = col;
        for (int r2 = col + 1; r2 < k; ++r2) if (fabsf(A[r2][col]) > fabsf(A[piv][col])) piv = r2;
        if (fabsf(A[piv][col]) < 1.0e-9f) return inl;
        if (piv != col) for (int cc = 0; cc < 4; ++cc) { const float t = A[col][cc]; A[col][cc] = A[piv][cc]; A[piv][cc] = t; }
        for (int r2 = col + 1; r2 < k; ++r2) {
            const float f = A[r2][col] / A[col][col];
            for (int cc = col; cc < 4; ++cc) A[r2][cc] -= f * A[col][cc];
        }
    }
    float sol[3] = {0};
    for (int a = k - 1; a >= 0; --a) {
        float v = A[a][3];
        for (int b = a + 1; b < k; ++b) v -= A[a][b] * sol[b];
        sol[a] = v / A[a][a];
    }
    for (int a = 0; a < k; ++a) delta[idx[a]] = sol[a];
    return inl;
}

static int score_pose(const arena_config_t *c, const wall_t w[8], const arena_pose_t *p, const float *qx, const float *qf, size_t m)
{
    int ok = 0;
    for (size_t i = 0; i < m; ++i) {
        float px, pz, r;
        arena_to_world(p, qx[i], qf[i], &px, &pz);
        if (nearest_wall(w, px, pz, 0.5f * c->tol_cm, &r) >= 0) ++ok;
    }
    return ok;
}

bool arena_acquire(arena_loc_t *a, const arena_point_t *pts, size_t n, float lidar_forward_cm)
{
    static float qx[MAX_PTS], qf[MAX_PTS], sx[MAX_PTS / 3 + 1], sf[MAX_PTS / 3 + 1];
    wall_t w[8];
    build_walls(&a->cfg, w);
    const size_t m = robot_points(&a->cfg, pts, n, lidar_forward_cm, qx, qf);
    if (m < 30) return false;
    /* Para la busqueda gruesa basta una de cada tres. */
    size_t ms = 0;
    for (size_t i = 0; i < m; i += 3) { sx[ms] = qx[i]; sf[ms] = qf[i]; ++ms; }
    const float O = a->cfg.outer_half_cm, I = a->cfg.inner_half_cm;
    const float z_lo = -(O - 8.0f), z_hi = -(I + 8.0f);
    arena_pose_t best = {0, 0, 0};
    int best_score = -1;
    for (int dirn = 0; dirn < 2; ++dirn) {
        const float base = dirn == 0 ? 90.0f : -90.0f;
        for (float dth = -8.0f; dth <= 8.1f; dth += 2.0f)
            for (float x = -(O - 10.0f); x <= O - 10.0f; x += 5.0f)
                for (float z = z_lo; z <= z_hi; z += 4.0f) {
                    const arena_pose_t p = {x, z, base + dth};
                    const int sc = score_pose(&a->cfg, w, &p, sx, sf, ms);
                    if (sc > best_score) { best_score = sc; best = p; }
                }
    }
    if (best_score < 0 || (float)best_score < a->cfg.init_min_score * (float)ms) return false;
    /* Refinado con todos los puntos. */
    arena_pose_t p = best;
    for (int it = 0; it < 8; ++it) {
        float d[3], sup[3], rms;
        gn_step(&a->cfg, w, &p, qx, qf, m, a->cfg.tol_cm, d, sup, &rms);
        p.x_cm += d[0]; p.z_cm += d[1]; p.theta_deg += d[2] * RAD2DEG;
    }
    const int final_score = score_pose(&a->cfg, w, &p, qx, qf, m);
    if ((float)final_score < a->cfg.init_min_score * (float)m) return false;
    p.theta_deg = wrap_deg(p.theta_deg);
    a->pose = p;
    a->valid = true;
    a->inliers = final_score;
    a->used = (int)m;
    return true;
}

void arena_predict(arena_loc_t *a, float ds_cm, float theta_deg)
{
    if (!a->valid) return;
    const float mid = (a->pose.theta_deg + 0.5f * wrap_deg(theta_deg - a->pose.theta_deg)) * DEG2RAD;
    a->pose.x_cm += ds_cm * sinf(mid);
    a->pose.z_cm += ds_cm * cosf(mid);
    a->pose.theta_deg = wrap_deg(theta_deg);
}

bool arena_update(arena_loc_t *a, const arena_point_t *pts, size_t n, float lidar_forward_cm)
{
    static float qx[MAX_PTS], qf[MAX_PTS];
    if (!a->valid) return false;
    wall_t w[8];
    build_walls(&a->cfg, w);
    const size_t m = robot_points(&a->cfg, pts, n, lidar_forward_cm, qx, qf);
    a->used = (int)m;
    if (m < 20) { a->last_dx = a->last_dz = a->last_dtheta_deg = 0.0f; return false; }
    arena_pose_t p = a->pose;
    float tx = 0.0f, tz = 0.0f, tr = 0.0f, sup[3] = {0}, rms = 0.0f;
    int inl = 0;
    for (int it = 0; it < 3; ++it) {
        float d[3];
        /* La primera pasada con tolerancia amplia: si la pose previa se desvio varios cm, la pared del fondo (la
         * unica que fija el eje longitudinal) queda fuera de la tolerancia normal y no aportaria nada. */
        inl = gn_step(&a->cfg, w, &p, qx, qf, m, it == 0 ? 2.5f * a->cfg.tol_cm : a->cfg.tol_cm, d, sup, &rms);
        p.x_cm += d[0]; p.z_cm += d[1]; p.theta_deg += d[2] * RAD2DEG;
        tx += d[0]; tz += d[1]; tr += d[2] * RAD2DEG;
        if (fabsf(d[0]) + fabsf(d[1]) + fabsf(d[2]) * 50.0f < 0.05f) break;
    }
    a->inliers = inl;
    a->sup_x = sup[0]; a->sup_z = sup[1]; a->sup_rot = sup[2];
    a->rms_cm = rms;
    if (hypotf(tx, tz) > a->cfg.max_shift_cm || fabsf(tr) > a->cfg.max_rot_deg ||
        (float)inl < 0.25f * (float)m) {
        ++a->rejected;
        a->last_dx = a->last_dz = a->last_dtheta_deg = 0.0f;
        return false;
    }
    a->last_dx = a->cfg.gain * tx;
    a->last_dz = a->cfg.gain * tz;
    a->last_dtheta_deg = a->cfg.gain * tr;
    a->pose.x_cm += a->last_dx;
    a->pose.z_cm += a->last_dz;
    a->pose.theta_deg = wrap_deg(a->pose.theta_deg + a->last_dtheta_deg);
    ++a->updates;
    return true;
}

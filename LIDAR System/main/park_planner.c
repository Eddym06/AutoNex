#include "park_planner.h"

#include <math.h>
#include <string.h>

/* Las rejillas de busqueda ocupan ~900 KB: en el ESP32 van a la PSRAM. */
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define PARK_BIG_BSS EXT_RAM_BSS_ATTR
#else
#define PARK_BIG_BSS
#endif

#define NU PARK_NU
#define NW PARK_NW
#define DEG2RAD 0.017453292519943295f
#define INF16 0xFFFFu
#define F_FREE 0x01u
#define F_ROTP 0x02u   /* se puede girar de este rumbo al siguiente (k+1) sin chocar */
#define F_NEAR 0x04u   /* libre, pero con menos de soft_margin de holgura */
#define NEAR_PENALTY 10
#define MAX_COMP 3     /* vectores primitivos con componentes <= 3: 32 rumbos */

static park_params_t g_p;
static park_goal_t g_goal;
static bool g_ready;
static int g_last_i = -1, g_last_j = -1, g_last_k = -1;   /* ultima celda decidida (ver to_state) */

static int g_nk;
static int g_dx[PARK_MAX_HEADINGS], g_dy[PARK_MAX_HEADINGS];
static float g_deg[PARK_MAX_HEADINGS];
static int g_cost_fwd[PARK_MAX_HEADINGS], g_cost_back[PARK_MAX_HEADINGS], g_cost_rot[PARK_MAX_HEADINGS];

static PARK_BIG_BSS uint16_t g_cost[PARK_MAX_HEADINGS][NW][NU];
static PARK_BIG_BSS uint8_t g_flags[PARK_MAX_HEADINGS][NW][NU];
static PARK_BIG_BSS uint32_t g_queue[PARK_MAX_HEADINGS * NW * NU];

static float lot_len(const park_params_t *p)
{
    return p->lot_length_cm > 0.0f ? p->lot_length_cm : 1.5f * p->robot_length_cm;
}

void park_params_default(park_params_t *p, float width, float length, float rear)
{
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->robot_width_cm = width;
    p->robot_length_cm = length;
    p->rear_overhang_cm = rear;
    p->lot_length_cm = 0.0f;
    p->lot_width_cm = 20.0f;
    p->limiter_thickness_cm = 2.0f;
    p->margin_cm = 1.5f;
    p->soft_margin_cm = 3.5f;
    p->goal_theta_tol_deg = 6.0f;
    p->goal_slack_cm = 1.5f;
    p->exit_clear_cm = 1.0f;
}

/* ---------------------------------------------------------------- geometria */

void park_body_corners(const park_params_t *p, const park_pose_t *pose, float out[4][2])
{
    const float th = pose->theta_deg * DEG2RAD, c = cosf(th), s = sinf(th);
    const float fs[4][2] = {{-p->rear_overhang_cm, -p->robot_width_cm * 0.5f},
                            {p->robot_length_cm - p->rear_overhang_cm, -p->robot_width_cm * 0.5f},
                            {p->robot_length_cm - p->rear_overhang_cm, p->robot_width_cm * 0.5f},
                            {-p->rear_overhang_cm, p->robot_width_cm * 0.5f}};
    for (int n = 0; n < 4; ++n) {
        out[n][0] = pose->u_cm + fs[n][0] * c - fs[n][1] * s;
        out[n][1] = pose->w_cm + fs[n][0] * s + fs[n][1] * c;
    }
}

/* Cuerpo convexo (4 puntos) contra rectangulo alineado, con holgura (SAT). */
static bool hits_rect(float poly[4][2], float x0, float x1, float y0, float y1, float margin)
{
    x0 -= margin; x1 += margin; y0 -= margin; y1 += margin;
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    for (int n = 0; n < 4; ++n) {
        if (poly[n][0] < minx) minx = poly[n][0];
        if (poly[n][0] > maxx) maxx = poly[n][0];
        if (poly[n][1] < miny) miny = poly[n][1];
        if (poly[n][1] > maxy) maxy = poly[n][1];
    }
    if (maxx < x0 || minx > x1 || maxy < y0 || miny > y1) return false;
    const float bx[2] = {x0, x1}, by[2] = {y0, y1};
    for (int e = 0; e < 2; ++e) {
        float nx = -(poly[e + 1][1] - poly[e][1]), ny = poly[e + 1][0] - poly[e][0];
        const float len = hypotf(nx, ny);
        nx /= len; ny /= len;
        float pmin = 1e9f, pmax = -1e9f, rmin = 1e9f, rmax = -1e9f;
        for (int q = 0; q < 4; ++q) {
            const float d = poly[q][0] * nx + poly[q][1] * ny;
            if (d < pmin) pmin = d;
            if (d > pmax) pmax = d;
        }
        for (int a = 0; a < 2; ++a) for (int b = 0; b < 2; ++b) {
            const float d = bx[a] * nx + by[b] * ny;
            if (d < rmin) rmin = d;
            if (d > rmax) rmax = d;
        }
        if (pmax < rmin || pmin > rmax) return false;
    }
    return true;
}

static bool pose_collides(const park_params_t *p, float u, float w, float theta_deg)
{
    const park_pose_t pose = {u, w, theta_deg};
    float c[4][2];
    park_body_corners(p, &pose, c);
    const float half = lot_len(p) * 0.5f, t = p->limiter_thickness_cm;
    return hits_rect(c, -400.0f, 400.0f, -100.0f, 0.0f, p->margin_cm) ||            /* pared */
           hits_rect(c, -half - t, -half, 0.0f, p->lot_width_cm, p->margin_cm) ||   /* tope trasero */
           hits_rect(c, half, half + t, 0.0f, p->lot_width_cm, p->margin_cm);       /* tope delantero */
}

bool park_pose_free(const park_params_t *p, const park_pose_t *pose)
{
    return p && pose && !pose_collides(p, pose->u_cm, pose->w_cm, pose->theta_deg);
}

static float rect_boundary_distance(float u, float w, float x0, float x1, float y0, float y1)
{
    const float dx = fmaxf(fmaxf(x0 - u, 0.0f), u - x1), dy = fmaxf(fmaxf(y0 - w, 0.0f), w - y1);
    if (dx > 0.0f || dy > 0.0f) return hypotf(dx, dy);
    return fminf(fminf(u - x0, x1 - u), fminf(w - y0, y1 - w));   /* dentro: a la arista mas cercana */
}

float park_model_distance(const park_params_t *p, float u, float w)
{
    const float half = lot_len(p) * 0.5f, t = p->limiter_thickness_cm;
    float d = fabsf(w);
    const float d1 = rect_boundary_distance(u, w, -half - t, -half, 0.0f, p->lot_width_cm);
    const float d2 = rect_boundary_distance(u, w, half, half + t, 0.0f, p->lot_width_cm);
    if (d1 < d) d = d1;
    if (d2 < d) d = d2;
    return d;
}

/* ------------------------------------------------------------------ rumbos */

static void build_headings(void)
{
    int dx[PARK_MAX_HEADINGS], dy[PARK_MAX_HEADINGS];
    float ang[PARK_MAX_HEADINGS];
    int n = 0;
    for (int x = -MAX_COMP; x <= MAX_COMP; ++x) {
        for (int y = -MAX_COMP; y <= MAX_COMP; ++y) {
            if (x == 0 && y == 0) continue;
            int a = x < 0 ? -x : x, b = y < 0 ? -y : y;
            while (b) { const int t = a % b; a = b; b = t; }
            if (a != 1) continue;                      /* solo vectores primitivos */
            if (n < PARK_MAX_HEADINGS) { dx[n] = x; dy[n] = y; ang[n] = atan2f((float)y, (float)x) / DEG2RAD; ++n; }
        }
    }
    /* Ordenados por angulo, de -180 a 180; el rumbo 0 queda en su sitio. */
    for (int i = 1; i < n; ++i) {
        const int xi = dx[i], yi = dy[i];
        const float ai = ang[i];
        int j = i - 1;
        while (j >= 0 && ang[j] > ai) { dx[j + 1] = dx[j]; dy[j + 1] = dy[j]; ang[j + 1] = ang[j]; --j; }
        dx[j + 1] = xi; dy[j + 1] = yi; ang[j + 1] = ai;
    }
    g_nk = n;
    for (int k = 0; k < n; ++k) {
        g_dx[k] = dx[k]; g_dy[k] = dy[k]; g_deg[k] = ang[k];
        const float len = hypotf((float)dx[k], (float)dy[k]) * PARK_CELL_CM;
        g_cost_fwd[k] = (int)lroundf(4.0f * len);
        g_cost_back[k] = (int)lroundf(4.6f * len);
        float d = ang[(k + 1) % n] - ang[k];
        if (d < 0.0f) d += 360.0f;
        g_cost_rot[k] = 4 + (int)lroundf(d / 4.0f);       /* de k a k+1 */
    }
}

static int nearest_heading(float theta_deg)
{
    int best = 0;
    float best_d = 1e9f;
    for (int k = 0; k < g_nk; ++k) {
        float d = fabsf(theta_deg - g_deg[k]);
        while (d > 360.0f) d -= 360.0f;
        if (d > 180.0f) d = 360.0f - d;
        if (d < best_d) { best_d = d; best = k; }
    }
    return best;
}

size_t park_heading_count(void) { return (size_t)g_nk; }
float park_heading_deg(size_t k) { return k < (size_t)g_nk ? g_deg[k] : 0.0f; }
void park_heading_step(size_t k, float *du_cm, float *dw_cm)
{
    if (k >= (size_t)g_nk) { *du_cm = 0.0f; *dw_cm = 0.0f; return; }
    *du_cm = (float)g_dx[k] * PARK_CELL_CM;
    *dw_cm = (float)g_dy[k] * PARK_CELL_CM;
}
size_t park_heading_nearest(float theta_deg) { return (size_t)nearest_heading(theta_deg); }

/* ------------------------------------------------------------------- campo */

static inline size_t sidx(int k, int j, int i) { return ((size_t)k * NW + (size_t)j) * NU + (size_t)i; }
static inline float cell_u(int i) { return PARK_U0_CM + ((float)i + 0.5f) * PARK_CELL_CM; }
static inline float cell_w(int j) { return ((float)j + 0.5f) * PARK_CELL_CM; }

static bool body_inside_lot(const park_params_t *p, float u, float w, float th)
{
    const park_pose_t pose = {u, w, th};
    float c[4][2];
    park_body_corners(p, &pose, c);
    const float half = lot_len(p) * 0.5f - p->goal_slack_cm;
    for (int n = 0; n < 4; ++n)
        if (c[n][0] < -half || c[n][0] > half || c[n][1] < p->goal_slack_cm ||
            c[n][1] > p->lot_width_cm - p->goal_slack_cm) return false;
    return true;
}

static bool body_clear_of_lot(const park_params_t *p, float u, float w, float th)
{
    const park_pose_t pose = {u, w, th};
    float c[4][2];
    park_body_corners(p, &pose, c);
    for (int n = 0; n < 4; ++n)
        if (c[n][1] < p->lot_width_cm + p->exit_clear_cm) return false;
    return true;
}

bool park_field_build(const park_params_t *p, park_goal_t goal)
{
    g_ready = false;
    g_last_k = -1;
    if (!p || !(p->robot_width_cm > 2.0f) || !(p->robot_length_cm > 4.0f) ||
        !(p->rear_overhang_cm >= 0.0f) || !(p->margin_cm >= 0.0f) || !(p->lot_width_cm > p->robot_width_cm))
        return false;
    g_p = *p;
    g_goal = goal;
    build_headings();
    memset(g_cost, 0xff, sizeof(g_cost));
    for (int k = 0; k < g_nk; ++k)
        for (int j = 0; j < NW; ++j)
            for (int i = 0; i < NU; ++i) {
                uint8_t f = 0;
                const float u = cell_u(i), w = cell_w(j), th = g_deg[k];
                if (!pose_collides(&g_p, u, w, th)) {
                    f |= F_FREE;
                    if (g_p.soft_margin_cm > g_p.margin_cm) {
                        park_params_t soft = g_p;
                        soft.margin_cm = g_p.soft_margin_cm;
                        if (pose_collides(&soft, u, w, th)) f |= F_NEAR;
                    }
                    /* Girar al rumbo siguiente: extremos libres y tambien el rumbo intermedio. */
                    const int k2 = (k + 1) % g_nk;
                    float d = g_deg[k2] - g_deg[k];
                    if (d < 0.0f) d += 360.0f;
                    if (!pose_collides(&g_p, u, w, g_deg[k] + 0.5f * d) &&
                        !pose_collides(&g_p, u, w, g_deg[k] + 0.25f * d) &&
                        !pose_collides(&g_p, u, w, g_deg[k] + 0.75f * d))
                        f |= F_ROTP;
                }
                g_flags[k][j][i] = f;
            }
    /* Meta. */
    size_t head = 0, tail = 0, goals = 0;
    for (int k = 0; k < g_nk; ++k) {
        const float th = g_deg[k];
        const bool heading_ok = goal == PARK_GOAL_IN ? fabsf(th) <= p->goal_theta_tol_deg
                                                     : fabsf(th) <= 20.0f;
        if (!heading_ok) continue;
        for (int j = 0; j < NW; ++j)
            for (int i = 0; i < NU; ++i) {
                if (!(g_flags[k][j][i] & F_FREE)) continue;
                const bool ok = goal == PARK_GOAL_IN ? body_inside_lot(&g_p, cell_u(i), cell_w(j), th)
                                                     : body_clear_of_lot(&g_p, cell_u(i), cell_w(j), th);
                if (!ok) continue;
                g_cost[k][j][i] = 0;
                g_queue[tail++] = (uint32_t)sidx(k, j, i);
                ++goals;
            }
    }
    if (goals == 0) return false;
    /* SPFA inverso con cola circular: relaja los predecesores de cada pose. */
    const size_t cap = sizeof(g_queue) / sizeof(g_queue[0]);
    size_t count = tail;
    tail %= cap;
    while (count) {
        const uint32_t id = g_queue[head];
        head = (head + 1) % cap;
        --count;
        const int k = (int)(id / ((uint32_t)NW * NU));
        const int j = (int)((id % ((uint32_t)NW * NU)) / NU);
        const int i = (int)(id % NU);
        const int c = g_cost[k][j][i];
        for (int m = 0; m < 4; ++m) {
            int pi = i, pj = j, pk = k, w;
            if (m == 0) { pi = i - g_dx[k]; pj = j - g_dy[k]; w = g_cost_fwd[k]; }          /* avanzo desde p y llego aqui */
            else if (m == 1) { pi = i + g_dx[k]; pj = j + g_dy[k]; w = g_cost_back[k]; }    /* retrocedo desde p */
            else if (m == 2) { pk = (k + g_nk - 1) % g_nk; w = g_cost_rot[pk]; }            /* giro de k-1 a k */
            else { pk = (k + 1) % g_nk; w = g_cost_rot[k]; }                                  /* giro de k+1 a k */
            if (pi < 0 || pj < 0 || pi >= NU || pj >= NW) continue;
            const uint8_t pf = g_flags[pk][pj][pi];
            if (!(pf & F_FREE)) continue;
            if (m == 2 && !(pf & F_ROTP)) continue;                     /* p=k-1 gira a k: usa su bit */
            if (m == 3 && !(g_flags[k][j][i] & F_ROTP)) continue;       /* p=k+1 gira a k: giro inverso de k->k+1 */
            if (pf & F_NEAR) w += NEAR_PENALTY;
            const int nc = c + w;
            if (nc < (int)g_cost[pk][pj][pi] && nc < (int)INF16) {
                g_cost[pk][pj][pi] = (uint16_t)nc;
                if (count < cap) { g_queue[tail] = (uint32_t)sidx(pk, pj, pi); tail = (tail + 1) % cap; ++count; }
            }
        }
    }
    g_ready = true;
    return true;
}

bool park_field_ready(void) { return g_ready; }
park_goal_t park_field_goal(void) { return g_goal; }
const park_params_t *park_field_params(void) { return &g_p; }

/* La pose medida tiembla unas decimas de centimetro; si cruza un borde de celda por poco la
 * decision cambia y el robot alterna dos ordenes sin avanzar. Se conserva la celda anterior
 * mientras la pose siga a menos de 0,8 celdas de su centro (y el rumbo a menos de 0,65 de
 * su separacion con los rumbos vecinos). */
static bool to_state(const park_pose_t *pose, int *k, int *j, int *i)
{
    int ii = (int)floorf((pose->u_cm - PARK_U0_CM) / PARK_CELL_CM);
    int jj = (int)floorf(pose->w_cm / PARK_CELL_CM);
    if (ii < 0 || jj < 0 || ii >= NU || jj >= NW) return false;
    int kk = nearest_heading(pose->theta_deg);
    if (g_last_k >= 0) {
        if (fabsf(pose->u_cm - cell_u(g_last_i)) <= 0.8f * PARK_CELL_CM) ii = g_last_i;
        if (fabsf(pose->w_cm - cell_w(g_last_j)) <= 0.8f * PARK_CELL_CM) jj = g_last_j;
        float d = fabsf(pose->theta_deg - g_deg[g_last_k]);
        while (d > 360.0f) d -= 360.0f;
        if (d > 180.0f) d = 360.0f - d;
        const int kn = (g_last_k + 1) % g_nk, kp = (g_last_k + g_nk - 1) % g_nk;
        float gap = fmaxf(fabsf(g_deg[kn] - g_deg[g_last_k]), fabsf(g_deg[g_last_k] - g_deg[kp]));
        if (gap > 180.0f) gap = 360.0f - gap;
        if (d <= 0.65f * 0.5f * gap) kk = g_last_k;
    }
    g_last_i = ii; g_last_j = jj; g_last_k = kk;
    *i = ii; *j = jj; *k = kk;
    return true;
}

float park_field_cost(const park_pose_t *pose)
{
    int k, j, i;
    if (!g_ready || !to_state(pose, &k, &j, &i)) return -1.0f;
    const uint16_t c = g_cost[k][j][i];
    return c == INF16 ? -1.0f : (float)c * 0.25f;
}

bool park_field_free(const park_pose_t *pose)
{
    int k, j, i;
    if (!g_ready || !to_state(pose, &k, &j, &i)) return false;
    return (g_flags[k][j][i] & F_FREE) != 0;
}

park_decision_t park_field_decide(const park_pose_t *pose, park_move_t previous)
{
    park_decision_t out = {PARK_MOVE_NONE, -1.0f, 0.0f, false};
    int k, j, i;
    if (!g_ready || !to_state(pose, &k, &j, &i)) return out;
    /* Si la celda no tiene camino (o esta en holgura), mira las vecinas: la medida tiene error. */
    if (g_cost[k][j][i] == INF16) {
        int bk = -1, bj = 0, bi = 0, best = INF16;
        for (int dj = -3; dj <= 3; ++dj)
            for (int di = -3; di <= 3; ++di) {
                const int nj = j + dj, ni = i + di;
                if (nj < 0 || ni < 0 || nj >= NW || ni >= NU) continue;
                for (int dk = -1; dk <= 1; ++dk) {
                    const int nk = (k + dk + g_nk) % g_nk;
                    if (g_cost[nk][nj][ni] < best) { best = g_cost[nk][nj][ni]; bk = nk; bj = nj; bi = ni; }
                }
            }
        if (bk < 0) return out;
        k = bk; j = bj; i = bi;
    }
    const int here = g_cost[k][j][i];
    out.cost = (float)here * 0.25f;
    if (here == 0) {
        /* La celda es meta. La meta de este planeador es quedar EN PARALELO (theta 0) o, al
         * salir, con el morro hacia el carril: se corrige contra rumbo 0 y no contra el bin
         * de discretizacion mas cercano. Con la pose medida temblando medio grado, el bin
         * oscila entre dos vecinos y el robot gira alternativamente a un lado y al otro sin
         * asentarse nunca (con el retardo del motor, el parqueo no terminaba). */
        float err = -pose->theta_deg;
        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        out.target_theta_deg = 0.0f;
        if (fabsf(err) > 4.0f) { out.move = err > 0.0f ? PARK_MOVE_ROT_POS : PARK_MOVE_ROT_NEG; return out; }
        out.at_goal = true;
        return out;
    }
    int best_total = INF16 * 2, totals[5] = {INF16 * 2, INF16 * 2, INF16 * 2, INF16 * 2, INF16 * 2};
    park_move_t best_move = PARK_MOVE_NONE;
    float thetas[5] = {0, g_deg[k], g_deg[k], g_deg[k], g_deg[k]};
    for (int m = 1; m <= 4; ++m) {
        int ni = i, nj = j, nk = k, w;
        if (m == PARK_MOVE_FORWARD) { ni = i + g_dx[k]; nj = j + g_dy[k]; w = g_cost_fwd[k]; }
        else if (m == PARK_MOVE_BACK) { ni = i - g_dx[k]; nj = j - g_dy[k]; w = g_cost_back[k]; }
        else if (m == PARK_MOVE_ROT_POS) {
            if (!(g_flags[k][j][i] & F_ROTP)) continue;
            nk = (k + 1) % g_nk; w = g_cost_rot[k]; thetas[m] = g_deg[nk];
        } else {
            nk = (k + g_nk - 1) % g_nk;
            if (!(g_flags[nk][j][i] & F_ROTP)) continue;
            w = g_cost_rot[nk]; thetas[m] = g_deg[nk];
        }
        if (g_flags[k][j][i] & F_NEAR) w += NEAR_PENALTY;
        if (ni < 0 || nj < 0 || ni >= NU || nj >= NW) continue;
        if (!(g_flags[nk][nj][ni] & F_FREE)) continue;
        const int nc = g_cost[nk][nj][ni];
        if (nc == INF16) continue;
        totals[m] = nc + w;
        if (totals[m] < best_total) { best_total = totals[m]; best_move = (park_move_t)m; }
    }
    if (best_move == PARK_MOVE_NONE) return out;
    /* Histeresis: seguir con el movimiento anterior si apenas pierde. */
    if (previous >= PARK_MOVE_FORWARD && previous <= PARK_MOVE_ROT_NEG && totals[previous] <= best_total + 3)
        best_move = previous;
    out.move = best_move;
    out.target_theta_deg = thetas[best_move];
    return out;
}

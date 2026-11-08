#include "park_ackermann.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define BIG_DATA EXT_RAM_BSS_ATTR
#else
#define BIG_DATA
#endif

/* Área alrededor del cajón: resolución 1 cm y 10 grados. */
#define NX 121
#define NY 71
#define NH 36
#define N (NX * NY * NH)
#define MAX_NODES 24000
#define HEAP_CAP (MAX_NODES * 4)
#define PI_F 3.14159265358979323846f
#define RAD (PI_F / 180.0f)
#define NEVER 1.0e20f

typedef struct { float u, w, theta, cost; int32_t parent; int8_t dir, steer; } node_t;
typedef struct { float score; int32_t id; } heap_item_t;
static BIG_DATA node_t nodes[MAX_NODES];
static BIG_DATA int32_t cells[N];
static BIG_DATA heap_item_t heap[HEAP_CAP];
static int heap_n, node_n;

static float wrap(float a) { while (a >= 180) a -= 360; while (a < -180) a += 360; return a; }
static int index_of(float u, float w, float theta)
{
    int x = (int)lroundf(u + 60.0f), y = (int)lroundf(w);
    int h = (int)lroundf((wrap(theta) + 180.0f) / 10.0f) % NH;
    if (x < 0 || x >= NX || y < 0 || y >= NY) return -1;
    return (y * NX + x) * NH + h;
}
static bool push(int id, float score)
{
    if (heap_n >= HEAP_CAP) return false;
    int i = heap_n++;
    while (i > 0) { int p = (i - 1) / 2; if (heap[p].score <= score) break; heap[i] = heap[p]; i = p; }
    heap[i] = (heap_item_t){score, id}; return true;
}
static heap_item_t pop(void)
{
    heap_item_t out = heap[0], last = heap[--heap_n];
    int i = 0;
    while (2 * i + 1 < heap_n) {
        int c = 2 * i + 1;
        if (c + 1 < heap_n && heap[c + 1].score < heap[c].score) ++c;
        if (heap[c].score >= last.score) break;
        heap[i] = heap[c]; i = c;
    }
    if (heap_n) heap[i] = last;
    return out;
}
static bool goal_reached(const park_params_t *p, const park_pose_t *s, park_goal_t goal)
{
    if (fabsf(wrap(s->theta_deg)) > 10.0f) return false;
    if (goal == PARK_GOAL_OUT) return s->w_cm >= p->lot_width_cm + p->robot_width_cm * 0.5f + 2.0f;
    const float len = p->lot_length_cm > 0 ? p->lot_length_cm : 1.5f * p->robot_length_cm;
    float c[4][2]; park_body_corners(p, s, c);
    for (int i = 0; i < 4; ++i)
        if (fabsf(c[i][0]) > len * 0.5f - 0.5f || c[i][1] < 0.5f || c[i][1] > p->lot_width_cm - 0.5f) return false;
    return true;
}
static float heuristic(const park_params_t *p, const park_pose_t *s, park_goal_t goal)
{
    const float target_w = goal == PARK_GOAL_OUT ? p->lot_width_cm + p->robot_width_cm * 0.5f + 2.0f : p->lot_width_cm * 0.5f;
    const float target_u = goal == PARK_GOAL_OUT ? s->u_cm : -(p->robot_length_cm * 0.5f - p->rear_overhang_cm);
    return 2.5f * hypotf(s->u_cm - target_u, s->w_cm - target_w) + fabsf(wrap(s->theta_deg)) * 0.12f;
}
static bool travel(const park_params_t *p, const park_ackermann_config_t *cfg, const park_pose_t *in,
                   int dir, int steer, park_pose_t *out)
{
    *out = *in;
    const float ds = dir * cfg->step_cm / 6.0f;
    const float curvature = tanf(steer * cfg->steer_max_deg * RAD) / cfg->wheelbase_cm;
    for (int i = 0; i < 6; ++i) {
        const float dtheta = ds * curvature, mid = out->theta_deg * RAD + dtheta * 0.5f;
        out->u_cm += ds * cosf(mid); out->w_cm += ds * sinf(mid);
        out->theta_deg = wrap(out->theta_deg + dtheta / RAD);
        if (!park_pose_free(p, out)) return false;
    }
    return true;
}

bool park_ackermann_arc_free(const park_params_t *p, const park_ackermann_config_t *cfg,
                            const park_pose_t *start, int direction, int steering)
{
    if (!p || !cfg || !start || !(cfg->wheelbase_cm > 0) || !(cfg->step_cm > 0) ||
        !(cfg->steer_max_deg > 0 && cfg->steer_max_deg < 80) ||
        (direction != -1 && direction != 1) || steering < -1 || steering > 1 || !park_pose_free(p, start)) return false;
    park_pose_t end;
    return travel(p, cfg, start, direction, steering, &end);
}

park_ackermann_result_t park_ackermann_plan(const park_params_t *p, const park_ackermann_config_t *cfg,
                                            const park_pose_t *start, park_goal_t goal)
{
    park_ackermann_result_t result = {0};
    if (!p || !cfg || !start || !isfinite(start->u_cm) || !isfinite(start->w_cm) ||
        !isfinite(start->theta_deg) || !(cfg->wheelbase_cm > 0) || !(cfg->step_cm >= 2) ||
        !(cfg->step_cm <= 10) || !(cfg->steer_max_deg > 0 && cfg->steer_max_deg < 80) || !park_pose_free(p, start)) return result;
    if (goal_reached(p, start, goal)) { result.found = result.at_goal = true; return result; }
    const int start_cell = index_of(start->u_cm, start->w_cm, start->theta_deg);
    if (start_cell < 0) return result;
    memset(cells, 0xff, sizeof cells);
    heap_n = 0; node_n = 1;
    const int start_id = 0;
    cells[start_cell] = start_id;
    nodes[start_id] = (node_t){start->u_cm, start->w_cm, wrap(start->theta_deg), 0, -1, (int8_t)cfg->previous_direction, 0};
    push(start_id, heuristic(p, start, goal));
    int end = -1;
    while (heap_n && result.expanded < MAX_NODES) {
        heap_item_t item = pop(); node_t cur = nodes[item.id];
        park_pose_t pose = {cur.u, cur.w, cur.theta};
        if (item.score > cur.cost + heuristic(p, &pose, goal) + 0.001f) continue;
        ++result.expanded;
        if (goal_reached(p, &pose, goal)) { end = item.id; break; }
        for (int dir = -1; dir <= 1; dir += 2) for (int steer = -1; steer <= 1; ++steer) {
            park_pose_t next;
            if (!travel(p, cfg, &pose, dir, steer, &next)) continue;
            int cell = index_of(next.u_cm, next.w_cm, next.theta_deg);
            if (cell < 0) continue;
            int id = cells[cell];
            if (id == item.id) continue;
            float cost = cur.cost + cfg->step_cm * (dir < 0 ? 1.1f : 1.0f) + (cur.dir && cur.dir != dir ? 4.0f : 0.0f);
            if (id >= 0 && cost >= nodes[id].cost) continue;
            if (id < 0) {
                if (node_n >= MAX_NODES) continue;
                id = node_n++; cells[cell] = id;
            }
            nodes[id] = (node_t){next.u_cm, next.w_cm, next.theta_deg, cost, item.id, (int8_t)dir, (int8_t)steer};
            if (!push(id, cost + heuristic(p, &next, goal))) return result;
        }
    }
    if (end < 0) return result;
    int child = end;
    while (child != start_id) {
        if (child < 0 || child >= node_n || result.count >= PARK_ACK_MAX_STEPS) return (park_ackermann_result_t){0};
        result.directions[result.count] = nodes[child].dir;
        result.steering[result.count] = nodes[child].steer;
        ++result.count;
        child = nodes[child].parent;
    }
    for (unsigned i = 0; i < result.count / 2; ++i) {
        unsigned j = result.count - 1 - i;
        int8_t d = result.directions[i], s = result.steering[i];
        result.directions[i] = result.directions[j]; result.steering[i] = result.steering[j];
        result.directions[j] = d; result.steering[j] = s;
    }
    result.found = true; result.direction = result.directions[0];
    result.steer_deg = result.steering[0] * cfg->steer_max_deg;
    return result;
}

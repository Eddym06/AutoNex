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
/* Tope de nodos del A*. Se probo a subirlo a 60000 con 40 grados y el resultado no cambio
 * (los mismos casos fallaban), asi que el limite no era la causa; se deja el valor original. */
#define MAX_NODES 24000
#define HEAP_CAP (MAX_NODES * 4)
#define PI_F 3.14159265358979323846f
/* Penalizacion de un cambio de sentido en el plan (ver la nota del coste). */
#define PARK_REVERSE_COST 12.0f
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
            /* Cambiar de sentido cuesta PARK_REVERSE_COST: con 4 (lo que valia) un cambio
             * costaba lo mismo que un arco de 3,5 cm y el plan salia a base de repasadas
             * (10 dentro del cajon). Cada repasada es tiempo y desgaste: se penaliza mas. */
            float cost = cur.cost + cfg->step_cm * (dir < 0 ? 1.1f : 1.0f) +
                         (cur.dir && cur.dir != dir ? PARK_REVERSE_COST : 0.0f);
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

/* --------------------------------------- entrada en una pasada: S de dos arcos
 * Entrar en el cajon como aparca una persona en paralelo: un arco largo de reversa que mete la
 * cola y un arco largo de avance que endereza. Dos movimientos y **un solo cambio de sentido**.
 * El A* de arriba no la encuentra porque encadena arcos de `step_cm` y vuelve a decidir en cada
 * uno: con una meta tan estrecha cada correccion se convierte en una pasada corta mas (medido:
 * 7-8 en la entrada, cuando la salida se hace en 1). Aqui se buscan las dos longitudes con la
 * geometria real (esquinas contra pared y topes) y solo se acepta si termina centrada, dentro y
 * paralela; si no hay ninguna, el llamador sigue con el A*. */
static void arc_step(float wheelbase, float steer_deg, park_pose_t *s, float ds)
{
    const float k = tanf(steer_deg * RAD) / wheelbase;
    const float mid = s->theta_deg * RAD + ds * k * 0.5f;
    s->u_cm += ds * cosf(mid); s->w_cm += ds * sinf(mid);
    s->theta_deg = wrap(s->theta_deg + ds * k / RAD);
}

bool park_ackermann_entry_s(const park_params_t *p, const park_ackermann_config_t *cfg,
                            const park_pose_t *start, park_ackermann_result_t *out)
{
    if (!p || !cfg || !start || !out || !(cfg->wheelbase_cm > 0) || !(cfg->steer_max_deg > 0)) return false;
    const float w_center = p->lot_width_cm * 0.5f;
    const float u_center = -(p->robot_length_cm * 0.5f - p->rear_overhang_cm);
    const float max_d = 2.4f * p->robot_length_cm;          /* no gastar mas que eso */
    const float step = 0.5f;
    for (int srev = 0; srev < 2; ++srev) {
        const float st1 = (srev ? -1.0f : 1.0f) * cfg->steer_max_deg, st2 = -st1;
        park_pose_t a = *start;
        for (float d1 = step; d1 <= max_d; d1 += step) {
            arc_step(cfg->wheelbase_cm, st1, &a, -step);
            if (!park_pose_free(p, &a)) break;              /* mas largo tampoco cabe */
            if (d1 < 2.0f) continue;
            park_pose_t b = a;
            for (float d2 = step; d2 <= max_d; d2 += step) {
                arc_step(cfg->wheelbase_cm, st2, &b, +step);
                if (!park_pose_free(p, &b)) break;
                if (d2 < 2.0f) continue;
                if (fabsf(wrap(b.theta_deg)) > 6.0f) continue;
                if (fabsf(b.u_cm - u_center) > 6.0f) continue;
                if (fabsf(b.w_cm - w_center) > 2.5f) continue;
                if (!goal_reached(p, &b, PARK_GOAL_IN)) continue;
                /* Se devuelve como arcos del ejecutor, sin pasar de PARK_ACK_MAX_STEPS. */
                const unsigned n1 = (unsigned)ceilf(d1 / cfg->step_cm);
                const unsigned n2 = (unsigned)ceilf(d2 / cfg->step_cm);
                if (n1 + n2 == 0 || n1 + n2 > PARK_ACK_MAX_STEPS) return false;
                out->count = 0;
                for (unsigned i = 0; i < n1; ++i) {
                    out->directions[out->count] = -1; out->steering[out->count] = (int8_t)(srev ? -1 : 1);
                    ++out->count;
                }
                for (unsigned i = 0; i < n2; ++i) {
                    out->directions[out->count] = +1; out->steering[out->count] = (int8_t)(srev ? 1 : -1);
                    ++out->count;
                }
                out->found = true; out->at_goal = false;
                out->direction = out->directions[0];
                out->steer_deg = out->steering[0] * cfg->steer_max_deg;
                return true;
            }
        }
    }
    return false;
}


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
#ifndef PARK_GOAL_TH_IN_DEG
#define PARK_GOAL_TH_IN_DEG 4.0f
#endif
#ifndef PARK_CLEAR_PENALTY
#define PARK_CLEAR_PENALTY 1.0f      /* cm de coste por subpaso a menos de margen+extra de un obstaculo */
#endif
#ifndef PARK_CLEAR_EXTRA_CM
#define PARK_CLEAR_EXTRA_CM 1.5f
#endif
/* Entrada: se planean varias variantes (otro precio de la repasada y de la holgura) y se elige la mejor (ver park_ackermann_plan).
 * 1 = un solo A* como antes. Cada variante es un A* completo: en el P4 el tiempo de plan sube casi lineal. */
#ifndef PARK_PLAN_VARIANTS
#define PARK_PLAN_VARIANTS 5
#endif
#ifndef PARK_SCORE_CLEAR_TARGET_CM
#define PARK_SCORE_CLEAR_TARGET_CM 3.0f   /* holgura extra por encima de la de planeo a partir de la cual ya no se premia */
#endif
#ifndef PARK_SCORE_CLEAR_WEIGHT
#define PARK_SCORE_CLEAR_WEIGHT 10.0f     /* cm de coste por cada cm de holgura que falta */
#endif

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
    /* Al entrar, el juez da 'completo' con |sen(rumbo)| x batalla <= 2 cm (6,2 grados para esta batalla).
     * La meta aceptaba hasta 10 grados y el error de rumbo estimado es ~1 grado: 3 de 24 rondas acababan
     * en 6,2-6,6 grados, dentro del cajon pero 'parcial' (medido). */
    if (fabsf(wrap(s->theta_deg)) > (goal == PARK_GOAL_IN ? PARK_GOAL_TH_IN_DEG : 10.0f)) return false;
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
static bool travel_pen(const park_params_t *p, const park_params_t *wide, const park_ackermann_config_t *cfg,
                       const park_pose_t *in, int dir, int steer, park_pose_t *out, int *pen)
{
    *out = *in;
    const float ds = dir * cfg->step_cm / 6.0f;
    const float curvature = tanf(steer * cfg->steer_max_deg * RAD) / cfg->wheelbase_cm;
    int v = 0;
    for (int i = 0; i < 6; ++i) {
        const float dtheta = ds * curvature, mid = out->theta_deg * RAD + dtheta * 0.5f;
        out->u_cm += ds * cosf(mid); out->w_cm += ds * sinf(mid);
        out->theta_deg = wrap(out->theta_deg + dtheta / RAD);
        if (!park_pose_free(p, out)) return false;
        if (wide && !park_pose_free(wide, out)) ++v;
    }
    if (pen) *pen = v;
    return true;
}
static bool travel(const park_params_t *p, const park_ackermann_config_t *cfg, const park_pose_t *in,
                   int dir, int steer, park_pose_t *out)
{
    return travel_pen(p, NULL, cfg, in, dir, steer, out, NULL);
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

typedef struct { float reverse_cost, clear_penalty, clear_extra_cm; } plan_variant_t;

static park_ackermann_result_t plan_core(const park_params_t *p, const park_ackermann_config_t *cfg,
                                         const park_pose_t *start, park_goal_t goal, plan_variant_t var)
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
    /* Coste suave de holgura (solo al entrar): las soluciones del A* con exactamente la holgura de
     * planeo rozan el tope y el error de pose (~0,5-1 cm) las aborta; las que tienen 1-2 cm de mas
     * se ejecutan enteras (medido en 16 rondas). Se prefieren sin prohibir las apretadas. */
    park_params_t wide = *p;
    wide.margin_cm += var.clear_extra_cm;
    const bool use_wide = goal == PARK_GOAL_IN && var.clear_penalty > 0.0f;
    int end = -1;
    while (heap_n && result.expanded < MAX_NODES) {
        heap_item_t item = pop(); node_t cur = nodes[item.id];
        park_pose_t pose = {cur.u, cur.w, cur.theta};
        if (item.score > cur.cost + heuristic(p, &pose, goal) + 0.001f) continue;
        ++result.expanded;
        if (goal_reached(p, &pose, goal)) { end = item.id; break; }
        for (int dir = -1; dir <= 1; dir += 2) for (int steer = -1; steer <= 1; ++steer) {
            park_pose_t next;
            int pen = 0;
            if (!travel_pen(p, use_wide ? &wide : NULL, cfg, &pose, dir, steer, &next, &pen)) continue;
            int cell = index_of(next.u_cm, next.w_cm, next.theta_deg);
            if (cell < 0) continue;
            int id = cells[cell];
            if (id == item.id) continue;
            /* Cambiar de sentido cuesta PARK_REVERSE_COST: con 4 (lo que valia) un cambio
             * costaba lo mismo que un arco de 3,5 cm y el plan salia a base de repasadas
             * (10 dentro del cajon). Cada repasada es tiempo y desgaste: se penaliza mas. */
            float cost = cur.cost + cfg->step_cm * (dir < 0 ? 1.1f : 1.0f) +
                         (cur.dir && cur.dir != dir ? var.reverse_cost : 0.0f) + (float)pen * var.clear_penalty;
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

/* ---------------------------------------------------------------- eleccion entre planes candidatos
 * El A* da UN camino y su forma depende de detalles: poses a 1 cm de distancia daban 1, 2, 5 o 10 repasadas
 * (medido) porque el camino optimo para el coste cambia de familia de golpe. Las repasadas de mas vienen de
 * planes con ~1 cm de holgura que el error de pose aborta a mitad, o de caminos con cambios de sentido que
 * el coste no pagaba lo bastante. Se planean varias variantes (cambio de sentido mas o menos caro, holgura
 * mas o menos premiada) y se elige la de menor puntuacion = longitud + cambios de sentido + holgura que falta. */
static bool plan_evaluate(const park_params_t *p, const park_ackermann_config_t *cfg, const park_pose_t *start,
                          const park_ackermann_result_t *r, float rev_cost, float *score, int *reversals, float *clear_cm)
{
    static const float extras[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
    enum { NE = sizeof extras / sizeof extras[0] };
    park_pose_t pose = *start;
    int rev = 0, last = cfg->previous_direction;
    float length = 0.0f;
    bool ok[NE];
    for (int e = 0; e < NE; ++e) ok[e] = true;
    for (unsigned i = 0; i < r->count; ++i) {
        const int dir = r->directions[i];
        if (last && dir != last) ++rev;
        last = dir;
        length += cfg->step_cm * (dir < 0 ? 1.1f : 1.0f);
        const float ds = dir * cfg->step_cm / 6.0f;
        const float curvature = tanf(r->steering[i] * cfg->steer_max_deg * RAD) / cfg->wheelbase_cm;
        for (int k = 0; k < 6; ++k) {
            const float dtheta = ds * curvature, mid = pose.theta_deg * RAD + dtheta * 0.5f;
            pose.u_cm += ds * cosf(mid); pose.w_cm += ds * sinf(mid);
            pose.theta_deg = wrap(pose.theta_deg + dtheta / RAD);
            for (int e = 0; e < NE; ++e) {
                if (!ok[e]) continue;
                park_params_t q = *p; q.margin_cm += extras[e];
                if (!park_pose_free(&q, &pose)) ok[e] = false;
            }
        }
    }
    float clear = 0.0f;
    for (int e = 0; e < NE; ++e) if (ok[e]) clear = extras[e]; else break;
    *reversals = rev; *clear_cm = clear;
    *score = length + rev_cost * (float)rev + PARK_SCORE_CLEAR_WEIGHT * fmaxf(0.0f, PARK_SCORE_CLEAR_TARGET_CM - clear);
    return true;
}

park_ackermann_result_t park_ackermann_plan(const park_params_t *p, const park_ackermann_config_t *cfg,
                                            const park_pose_t *start, park_goal_t goal)
{
    const plan_variant_t base = {PARK_REVERSE_COST, PARK_CLEAR_PENALTY, PARK_CLEAR_EXTRA_CM};
    park_ackermann_result_t best = plan_core(p, cfg, start, goal, base);
    if (goal != PARK_GOAL_IN || PARK_PLAN_VARIANTS <= 1 || best.at_goal || !cfg) return best;
    static const plan_variant_t alt[] = {
        {PARK_REVERSE_COST, 4.0f, 3.0f},          /* mas holgura (premiada) */
        {2.0f * PARK_REVERSE_COST, 1.0f, 1.5f},   /* menos cambios de sentido */
        {2.0f * PARK_REVERSE_COST, 4.0f, 3.0f},   /* las dos cosas */
        {0.5f * PARK_REVERSE_COST, 2.0f, 2.5f},   /* mas corto en distancia */
    };
    float best_score = 1.0e30f; int best_rev = 0; float best_clear = 0.0f;
    /* Si el A* base no encontro camino (se le acabaron los nodos), cualquier variante que lo encuentre sirve. */
    if (best.found) plan_evaluate(p, cfg, start, &best, PARK_REVERSE_COST, &best_score, &best_rev, &best_clear);
    unsigned total_expanded = best.expanded;
    for (unsigned v = 0; v < sizeof alt / sizeof alt[0] && (int)v + 1 < PARK_PLAN_VARIANTS; ++v) {
        park_ackermann_result_t cand = plan_core(p, cfg, start, goal, alt[v]);
        total_expanded += cand.expanded;
        if (!cand.found || cand.at_goal) continue;
        float sc; int rev; float clr;
        plan_evaluate(p, cfg, start, &cand, PARK_REVERSE_COST, &sc, &rev, &clr);
        if (sc < best_score - 0.01f) { best = cand; best_score = sc; best_rev = rev; best_clear = clr; }
    }
    (void)best_rev; (void)best_clear;
    best.expanded = total_expanded;
    return best;
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

/* Vuelca un tramo de longitud `len` en arcos del ejecutor. */
static bool push_segment(park_ackermann_result_t *out, float len, float step_cm, int dir, int steer)
{
    unsigned n = (unsigned)ceilf(len / step_cm);
    if (n == 0) n = 1;
    if (out->count + n > PARK_ACK_MAX_STEPS) return false;
    for (unsigned i = 0; i < n; ++i) {
        out->directions[out->count] = (int8_t)dir;
        out->steering[out->count] = (int8_t)steer;
        ++out->count;
    }
    return true;
}

/* Maniobra de entrada en una pasada: tres tramos con el volante al maximo.
 *   1) avance (o reversa) para colocarse en la pose de arranque,
 *   2) el arco largo que entra y gira el coche hacia el cajon,
 *   3) el arco que endereza.
 * Es lo que hace una persona al aparcar en paralelo y son DOS cambios de sentido. El A* de arcos
 * cortos no lo encuentra: castiga el rumbo grande en la heuristica y, al retroceder desde el
 * carril, su primer arco engancha el tope trasero a los 6,5 cm, asi que solo sabe entrar casi
 * paralelo a mordiscos (7-8 pasadas medidas).
 * Se buscan las tres longitudes con la geometria real (esquinas contra pared y topes) y se acepta
 * solo si termina centrada, dentro y paralela. Si no hay, el llamador sigue con el A*. */
bool park_ackermann_entry_maneuver(const park_params_t *p, const park_ackermann_config_t *cfg,
                                   const park_pose_t *start, park_ackermann_result_t *out)
{
    if (!p || !cfg || !start || !out || !(cfg->wheelbase_cm > 0) || !(cfg->steer_max_deg > 0)) return false;
    const float w_center = p->lot_width_cm * 0.5f;
    const float u_center = -(p->robot_length_cm * 0.5f - p->rear_overhang_cm);
    const float max_d = 2.0f * p->robot_length_cm;
    const float coarse = 1.0f;                       /* rejilla de busqueda */
    const float s = cfg->steer_max_deg;
    float best_total = 1.0e9f;
    float bd[3] = {0, 0, 0};
    int bdir[3] = {0, 0, 0}, bst[3] = {0, 0, 0};
    bool found = false;
    /* Dos ordenes (avance-reversa-avance y su inverso) por dos signos de volante en el tramo
     * central; el primero y el tercero van con el signo contrario entre si (para enderezar). */
    for (int order = 0; order < 2; ++order) {
        for (int sign = 0; sign < 2; ++sign) {
            const int d1 = order ? -1 : +1, d2 = -d1, d3 = d1;
            const int st1 = sign ? -1 : +1, st2 = -st1, st3 = st1;
            park_pose_t a = *start;
            for (float l1 = coarse; l1 <= max_d; l1 += coarse) {
                arc_step(cfg->wheelbase_cm, st1 * s, &a, d1 * coarse);
                if (!park_pose_free(p, &a)) break;
                park_pose_t b = a;
                for (float l2 = coarse; l2 <= max_d; l2 += coarse) {
                    arc_step(cfg->wheelbase_cm, st2 * s, &b, d2 * coarse);
                    if (!park_pose_free(p, &b)) break;
                    park_pose_t c = b;
                    for (float l3 = coarse; l3 <= max_d; l3 += coarse) {
                        arc_step(cfg->wheelbase_cm, st3 * s, &c, d3 * coarse);
                        if (!park_pose_free(p, &c)) break;
                        if (l1 + l2 + l3 >= best_total) continue;
                        if (fabsf(wrap(c.theta_deg)) > 5.0f) continue;
                        if (fabsf(c.u_cm - u_center) > 6.0f) continue;
                        if (fabsf(c.w_cm - w_center) > 2.5f) continue;
                        if (!goal_reached(p, &c, PARK_GOAL_IN)) continue;
                        best_total = l1 + l2 + l3;
                        bd[0] = l1; bd[1] = l2; bd[2] = l3;
                        bdir[0] = d1; bdir[1] = d2; bdir[2] = d3;
                        bst[0] = st1; bst[1] = st2; bst[2] = st3;
                        found = true;
                    }
                }
            }
        }
    }
    if (!found) return false;
    out->count = 0;
    for (int i = 0; i < 3; ++i)
        if (!push_segment(out, bd[i], cfg->step_cm, bdir[i], bst[i])) return false;
    out->found = true; out->at_goal = false;
    out->direction = out->directions[0];
    out->steer_deg = out->steering[0] * cfg->steer_max_deg;
    return true;
}


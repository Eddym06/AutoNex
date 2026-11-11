#include "local_obstacle_map.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LOCAL_DEG_TO_RAD 0.01745329251994329577f

/* Campo de distancias en medias celdas (1,25 cm). El centinela 255 satura en
 * 318 cm, muy por encima de lo que consulta la reticula (60 cm). */
#define LOCAL_DT_FAR 255
#define LOCAL_DT_UNIT_CM (LOCAL_MAP_CELL_CM * 0.5f)

/* Dos rutas simetricas difieren solo en el ultimo bit de los flotantes: el
 * desempate por giro mas suave usa esta tolerancia para no decidir con ruido. */
#define LOCAL_COST_EPS 1.0e-9f

_Static_assert(LOCAL_LATTICE_DEPTH == 3,
               "route_cost y lattice_action despliegan tres arcos explicitos");

/* ------------------------------------------------------------- configuracion */

void local_planner_config_default(local_planner_config_t *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->chassis_width_cm = LOCAL_CHASSIS_WIDTH_CM;
    c->chassis_length_cm = LOCAL_CHASSIS_LENGTH_CM;
    c->wheelbase_cm = LOCAL_WHEELBASE_CM;
    c->chassis_rear_cm = LOCAL_CHASSIS_REAR_CM;
    c->footprint_shrink_cm = 1.0f;
    c->squeeze_cm = 3.0f;
    c->tight_clear_cm = 0.0f;        /* apagado: costaba ~18 s en una ronda de 7 letreros */
    c->lane_half_cm = 50.0f;
    c->steer_nudge_deg = LOCAL_STEER_NUDGE_DEG;
    c->steer_soft_deg = LOCAL_STEER_SOFT_DEG;
    c->steer_hard_deg = LOCAL_STEER_HARD_DEG;
    c->lidar_forward_cm = LOCAL_LIDAR_FORWARD_CM;
    c->camera_forward_cm = LOCAL_CAMERA_FORWARD_CM;
    c->lidar_min_cm = 5.0f;
    /* Una vuelta de 8 m (circuito de 1 m de carril) tres veces, mas el parqueo,
     * no cabe en los 180 s de la ronda a 10 cm/s; 20 cm/s deja holgura. */
    c->speed_cm_s = 34.0f;            /* velocidad mientras construye el mapa (vuelta 1). 30 y 40 dieron la misma fiabilidad que 23 con el chasis largo26d (24 rondas: 15 frente a 14-15 perfectas, 0 contactos); a 55 ya pierde fiabilidad. Se fija en 34 porque a 35 el plano del equipo (plano_corregido.json) pasa un letrero verde por el lado equivocado y a 33 el eslalon de test_missions roza letreros; 34 pasa ambos (el comportamiento no es monotono con la velocidad: 30, 33 y 36 fallan el eslalon). Un banco anterior con otro perfil habia dado 26 como limite: revalidar con el chasis real */
    c->maneuver_speed_cm_s = 10.0f;
    c->cycle_s = 0.1f;
    c->scan_period_s = 0.1f;
    c->sync_max_age_us = LOCAL_SENSOR_SYNC_MAX_AGE_US;
    /* Pesos: seguridad por encima de mision, y mision por encima de suavidad. */
    c->w_clearance = 4.0f; c->w_lateral = 1.0f; c->w_heading = 3.0f;
    c->w_turn = 0.3f; c->w_progress = 1.0f; c->w_dead_end = 60.0f;
    c->w_change = 0.4f; c->w_camera = 0.5f; c->w_pass_shortfall = 8.0f;
    /* Banda de empate: dentro de ella se mantiene la decision anterior. */
    c->switch_tie = 2.0f;
    c->arc_cm = LOCAL_LATTICE_ARC_CM;
    c->discount = LOCAL_LATTICE_DISCOUNT;
    c->clearance_pref_cm = LOCAL_CLEARANCE_PREF_CM;
    c->lateral_scale_cm = LOCAL_LATERAL_SCALE_CM;
    c->min_commit_cm = LOCAL_MIN_COMMIT_CM;
    c->step_guard_cm = 0.0f;
    c->track_lookahead_cm = 40.0f;
    c->track_deadband_deg = 0.8f;
    c->track_slack = 3.0f;
    c->plan_enabled = true;
    c->plan_horizon_cm = 150.0f;
    c->plan_margin_cm = 2.0f;
    c->plan_margin_extra_cm = 3.0f;
    c->plan_lookahead_cm = 40.0f;
    c->plan_slack = 6.0f;
    c->plan_w_lateral = 1.0f;
    c->plan_w_clearance = 4.0f;
    c->plan_w_slew = 0.3f;
    c->plan_refine = false;
    c->plan_w_path = 15.0f;
    c->plan_w_race = 1.0f;
    c->plan_race_clear_cm = 6.0f;
    c->plan_path_scale_cm = 5.0f;     /* 5 es estable en dos bancos de 100 pistas (99/95 perfectas; con 10: 95/93; con 6: 98/90) */
    c->pass_probe_cm = LOCAL_PASS_PROBE_CM;
    c->min_pass_cm = LOCAL_MIN_PASS_CM;
    c->lane_max_span_cm = LOCAL_LANE_MAX_SPAN_CM;
    c->lane_spread_cm = LOCAL_LANE_SPREAD_CM;
    c->recenter_lane_cm = LOCAL_RECENTER_LANE_CM;
    c->recenter_hard_deg = LOCAL_RECENTER_HARD_DEG;
    c->recenter_hard_cm = LOCAL_RECENTER_HARD_CM;
    c->recenter_slack = LOCAL_RECENTER_SLACK;
    c->recenter_slack_heading = LOCAL_RECENTER_SLACK_HEADING;
    c->recenter_deg_to_cm = LOCAL_RECENTER_DEG_TO_CM;
    c->recenter_clear_cm = LOCAL_RECENTER_CLEAR_CM;
    c->escape_room_cm = LOCAL_ESCAPE_ROOM_CM;
    c->escape_margin_cm = LOCAL_ESCAPE_MARGIN_CM;
    c->escape_turn_deg = LOCAL_ESCAPE_TURN_DEG;
    c->escape_probe_pad_cm = LOCAL_ESCAPE_PROBE_PAD_CM;
    c->escape_back_max_cm = LOCAL_ESCAPE_BACK_MAX_CM;
    c->escape_min_back_cm = 15.0f;
    c->escape_retry_s = 2.0f;
    c->escape_max_attempts = (uint8_t)LOCAL_ESCAPE_MAX_ATTEMPTS;
    c->bridge_max_gap_cm = 12.0f;
    c->bridge_max_jump_cm = 8.0f;
    c->deskew = true;
    c->pass_side_red = 1;
    c->pass_side_green = -1;
    c->enforce_color_side = true;
    c->curtain_len_cm = 80.0f;
    c->curtain_min_forward_cm = 0.0f;
    c->assoc_bearing_tol_deg = 5.0f;
    c->assoc_range_rel = 0.5f;
    c->assoc_range_abs_cm = 12.0f;
    c->cluster_radius_cm = 6.0f;
    c->camera_ghost_fallback = false;
    c->curtain_unconfirmed = false;
}

static bool clampf(float *v, float lo, float hi)
{
    if (!isfinite(*v)) { *v = lo; return true; }
    if (*v < lo) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}

bool local_planner_config_sanitize(local_planner_config_t *c)
{
    if (!c) return false;
    bool ch = false;
    ch |= clampf(&c->chassis_width_cm, 2.0f, 30.0f);
    ch |= clampf(&c->chassis_length_cm, 4.0f, 40.0f);
    ch |= clampf(&c->wheelbase_cm, 3.0f, 40.0f);
    ch |= clampf(&c->chassis_rear_cm, 0.0f, c->chassis_length_cm - 1.0f);
    ch |= clampf(&c->footprint_shrink_cm, 0.0f, c->chassis_width_cm * 0.5f);
    ch |= clampf(&c->squeeze_cm, 0.0f, 6.0f);
    ch |= clampf(&c->tight_clear_cm, 0.0f, 20.0f);
    ch |= clampf(&c->lane_half_cm, 15.0f, 150.0f);
    ch |= clampf(&c->steer_nudge_deg, 0.5f, 20.0f);
    ch |= clampf(&c->steer_soft_deg, c->steer_nudge_deg, 45.0f);
    ch |= clampf(&c->steer_hard_deg, c->steer_soft_deg, 45.0f);
    ch |= clampf(&c->lidar_forward_cm, -20.0f, 40.0f);
    ch |= clampf(&c->camera_forward_cm, -20.0f, 40.0f);
    ch |= clampf(&c->lidar_min_cm, 1.0f, 30.0f);
    ch |= clampf(&c->speed_cm_s, 3.0f, 200.0f);
    ch |= clampf(&c->maneuver_speed_cm_s, 2.0f, 100.0f);
    ch |= clampf(&c->cycle_s, 0.02f, 0.5f);
    ch |= clampf(&c->scan_period_s, 0.02f, 0.5f);
    if (c->sync_max_age_us < 10000u) { c->sync_max_age_us = 10000u; ch = true; }
    if (c->sync_max_age_us > 1000000u) { c->sync_max_age_us = 1000000u; ch = true; }
    ch |= clampf(&c->arc_cm, 10.0f, 40.0f);
    ch |= clampf(&c->discount, 0.5f, 1.0f);
    ch |= clampf(&c->clearance_pref_cm, 1.0f, 40.0f);
    ch |= clampf(&c->lateral_scale_cm, 5.0f, 100.0f);
    ch |= clampf(&c->min_commit_cm, 5.0f, 3.0f * c->arc_cm);
    ch |= clampf(&c->step_guard_cm, 0.0f, 20.0f);
    ch |= clampf(&c->track_lookahead_cm, 10.0f, 150.0f);
    ch |= clampf(&c->track_deadband_deg, 0.0f, 10.0f);
    ch |= clampf(&c->track_slack, 0.0f, 20.0f);
    ch |= clampf(&c->plan_horizon_cm, 30.0f, 180.0f);
    ch |= clampf(&c->plan_margin_cm, 0.0f, 10.0f);
    ch |= clampf(&c->plan_margin_extra_cm, 0.0f, 10.0f);
    ch |= clampf(&c->plan_lookahead_cm, 15.0f, c->plan_horizon_cm);
    ch |= clampf(&c->plan_slack, 0.0f, 50.0f);
    ch |= clampf(&c->plan_w_lateral, 0.0f, 20.0f);
    ch |= clampf(&c->plan_w_clearance, 0.0f, 50.0f);
    ch |= clampf(&c->plan_w_slew, 0.0f, 20.0f);
    ch |= clampf(&c->plan_w_path, 0.0f, 100.0f);
    ch |= clampf(&c->plan_path_scale_cm, 2.0f, 50.0f);
    ch |= clampf(&c->plan_w_race, 0.0f, 20.0f);
    ch |= clampf(&c->plan_race_clear_cm, 0.0f, 20.0f);
    ch |= clampf(&c->pass_probe_cm, 3.0f * c->arc_cm, 300.0f);
    ch |= clampf(&c->min_pass_cm, 5.0f, 100.0f);
    ch |= clampf(&c->escape_back_max_cm, 5.0f, 200.0f);
    ch |= clampf(&c->escape_min_back_cm, 0.0f, c->escape_back_max_cm);
    ch |= clampf(&c->escape_retry_s, 0.2f, 60.0f);
    ch |= clampf(&c->escape_turn_deg, 20.0f, 180.0f);
    ch |= clampf(&c->bridge_max_gap_cm, 0.0f, 30.0f);
    ch |= clampf(&c->bridge_max_jump_cm, 0.0f, 30.0f);
    ch |= clampf(&c->curtain_len_cm, 0.0f, 200.0f);
    ch |= clampf(&c->assoc_bearing_tol_deg, 0.5f, 30.0f);
    ch |= clampf(&c->assoc_range_rel, 0.05f, 2.0f);
    ch |= clampf(&c->cluster_radius_cm, 1.0f, 20.0f);
    if (c->pass_side_red > 1 || c->pass_side_red < -1) { c->pass_side_red = 0; ch = true; }
    if (c->pass_side_green > 1 || c->pass_side_green < -1) { c->pass_side_green = 0; ch = true; }
    if (c->escape_max_attempts < 1u) { c->escape_max_attempts = 1u; ch = true; }
    return ch;
}

/* El planificador es de un solo hilo: la configuracion activa y las curvaturas
 * derivadas se fijan al entrar en update/recommend. */
static const local_planner_config_t *g;
static float k_nudge, k_soft, k_hard;
/* Puntos de muestreo de la huella: uno por celda de 5 cm como maximo. Con tres
 * puntos fijos, un chasis ancho dejaba entre muestras huecos mayores que un
 * pilar y el planificador pasaba "por encima" de el. */
#define FOOT_MAX 9
static int foot_nx, foot_nf;

static local_planner_config_t g_defaults;
static bool g_defaults_ready;

void local_planner_set_default_config(const local_planner_config_t *cfg)
{
    if (cfg) { g_defaults = *cfg; g_defaults_ready = true; local_planner_config_sanitize(&g_defaults); }
    else g_defaults_ready = false;
}

static void use_config(const local_planner_config_t *cfg)
{
    if (!cfg) {
        if (!g_defaults_ready) { local_planner_config_default(&g_defaults); g_defaults_ready = true; }
        cfg = &g_defaults;
    }
    g = cfg;
    foot_nx = (int)ceilf(cfg->chassis_width_cm / LOCAL_MAP_CELL_CM) + 1;
    foot_nf = (int)ceilf(cfg->chassis_length_cm / LOCAL_MAP_CELL_CM) + 1;
    if (foot_nx < 3) foot_nx = 3;
    if (foot_nf < 3) foot_nf = 3;
    if (foot_nx > FOOT_MAX) foot_nx = FOOT_MAX;
    if (foot_nf > FOOT_MAX) foot_nf = FOOT_MAX;
    k_nudge = tanf(cfg->steer_nudge_deg * LOCAL_DEG_TO_RAD) / cfg->wheelbase_cm;
    k_soft = tanf(cfg->steer_soft_deg * LOCAL_DEG_TO_RAD) / cfg->wheelbase_cm;
    k_hard = tanf(cfg->steer_hard_deg * LOCAL_DEG_TO_RAD) / cfg->wheelbase_cm;
}

static float step_cm(void) { return g->maneuver_speed_cm_s * g->cycle_s; }

/* Con 20 cm/s y un ciclo de 100 ms el chasis avanza 2 cm entre decisiones y la
 * siguiente lectura llega con retraso: se verifican dos ciclos por delante. */
static float step_guard_cm(void)
{
    if (g->step_guard_cm > 0.0f) return g->step_guard_cm;
    const float two_cycles = 2.0f * g->speed_cm_s * g->cycle_s;
    return two_cycles > LOCAL_STEP_GUARD_CM ? two_cycles : LOCAL_STEP_GUARD_CM;
}

static int local_min_int(int a, int b) { return a < b ? a : b; }

static bool to_cell(float x_cm, float forward_cm, int *x, int *y);
static float lattice_curvature;   /* curvatura continua de la ultima decision de la retícula */

/* Campo de distancias: distancia de cada celda al obstaculo mas cercano, en
 * medias celdas. Dos pasadas chamfer 1:1,5 y ya esta: a partir de aqui, saber
 * si la huella cabe en un punto es una consulta O(1) en vez de lanzar rayos, y
 * la tabla la comparten todas las rutas candidatas. */
static uint8_t local_dt[LOCAL_MAP_SIZE][LOCAL_MAP_SIZE];

static void build_distance_field(const local_obstacle_map_t *map)
{
    for (int y = 0; y < LOCAL_MAP_SIZE; ++y)
        for (int x = 0; x < LOCAL_MAP_SIZE; ++x)
            local_dt[y][x] = (map->cells[y][x] & LOCAL_CELL_OCCUPIED)
                ? 0u : (uint8_t)LOCAL_DT_FAR;
    for (int y = 0; y < LOCAL_MAP_SIZE; ++y) {
        for (int x = 0; x < LOCAL_MAP_SIZE; ++x) {
            int best = (int)local_dt[y][x];
            if (best == 0) continue;
            if (y > 0) {
                if (x > 0) best = local_min_int(best, (int)local_dt[y - 1][x - 1] + 3);
                best = local_min_int(best, (int)local_dt[y - 1][x] + 2);
                if (x + 1 < LOCAL_MAP_SIZE)
                    best = local_min_int(best, (int)local_dt[y - 1][x + 1] + 3);
            }
            if (x > 0) best = local_min_int(best, (int)local_dt[y][x - 1] + 2);
            local_dt[y][x] = (uint8_t)(best > LOCAL_DT_FAR ? LOCAL_DT_FAR : best);
        }
    }
    for (int y = LOCAL_MAP_SIZE - 1; y >= 0; --y) {
        for (int x = LOCAL_MAP_SIZE - 1; x >= 0; --x) {
            int best = (int)local_dt[y][x];
            if (best == 0) continue;
            if (y + 1 < LOCAL_MAP_SIZE) {
                if (x + 1 < LOCAL_MAP_SIZE)
                    best = local_min_int(best, (int)local_dt[y + 1][x + 1] + 3);
                best = local_min_int(best, (int)local_dt[y + 1][x] + 2);
                if (x > 0) best = local_min_int(best, (int)local_dt[y + 1][x - 1] + 3);
            }
            if (x + 1 < LOCAL_MAP_SIZE)
                best = local_min_int(best, (int)local_dt[y][x + 1] + 2);
            local_dt[y][x] = (uint8_t)(best > LOCAL_DT_FAR ? LOCAL_DT_FAR : best);
        }
    }
}

/* Holgura de la huella en un punto del recorrido, con el chasis girado `theta`.
 * La huella hay que rotarla: una ruta que gira lleva el chasis girado, y medir
 * un rectangulo orientado como el chasis inicial mide un vehiculo que no existe. */
static float footprint_clearance(float x_cm, float forward_cm, float theta)
{
    const float c = cosf(theta), s = sinf(theta);
    float worst = 1.0e9f;
    for (int li = 0; li < foot_nx; ++li) {
        /* Las celdas marcadas son de 2,5 cm y se redondean hacia fuera: una pared a 6,3 cm del
         * borde del chasis se marca a 5 cm. `footprint_shrink_cm` compensa ese redondeo. */
        const float ox = ((float)li / (float)(foot_nx - 1) - 0.5f) * (g->chassis_width_cm - g->footprint_shrink_cm);
        for (int fi = 0; fi < foot_nf; ++fi) {
            const float of = (float)fi / (float)(foot_nf - 1) * g->chassis_length_cm - g->chassis_rear_cm;
            int cx, cy;
            if (!to_cell(x_cm + ox * c + of * s, forward_cm + of * c - ox * s, &cx, &cy))
                return 0.0f;
            const float d = (float)local_dt[cy][cx] * LOCAL_DT_UNIT_CM;
            if (d < worst) worst = d;
        }
    }
    return worst;
}

static float action_curvature(uint8_t action)
{
    switch (action) {
    case LOCAL_ACTION_NUDGE_RIGHT: return k_nudge;
    case LOCAL_ACTION_NUDGE_LEFT: return -k_nudge;
    case LOCAL_ACTION_RIGHT_SOFT: return k_soft;
    case LOCAL_ACTION_RIGHT_HARD: return k_hard;
    case LOCAL_ACTION_LEFT_SOFT: return -k_soft;
    case LOCAL_ACTION_LEFT_HARD: return -k_hard;
    default: return 0.0f;
    }
}

static int action_turn_dir(uint8_t action)
{
    switch (action) {
    case LOCAL_ACTION_NUDGE_RIGHT:
    case LOCAL_ACTION_RIGHT_SOFT:
    case LOCAL_ACTION_RIGHT_HARD:
    case LOCAL_ACTION_PARK_RIGHT:
    case LOCAL_ACTION_RETURN_RIGHT: return 1;
    case LOCAL_ACTION_NUDGE_LEFT:
    case LOCAL_ACTION_LEFT_SOFT:
    case LOCAL_ACTION_LEFT_HARD:
    case LOCAL_ACTION_PARK_LEFT:
    case LOCAL_ACTION_RETURN_LEFT: return -1;
    default: return 0;
    }
}

static float yaw_delta_deg(float now_deg, float then_deg)
{
    float delta = now_deg - then_deg;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

/* Comprueba que el arco de curvatura `k` tenga `distance` cm por delante. */
static bool arc_clear(float k, float distance)
{
    const float sample = 2.0f;
    float theta = 0.0f, x_cm = 0.0f, forward_cm = 0.0f;
    for (float travelled = 0.0f; travelled < distance - 1.0e-6f; travelled += sample) {
        theta += k * sample;
        x_cm += sinf(theta) * sample;
        forward_cm += cosf(theta) * sample;
        if (footprint_clearance(x_cm, forward_cm, theta) <= 0.0f) return false;
    }
    return true;
}

/* Centro del carril medido en una fila, en cm (positivo hacia la derecha del
 * robot). Devuelve false cuando esa fila no es un carril: sin pared a los dos
 * lados la resta mide el mapa, no un pasillo. Las paredes de un carril recto son
 * paralelas a la direccion de avance, asi que no aparecen en una fila lejana:
 * hay que medirlas al lado. */
static bool row_center_cm(const local_obstacle_map_t *map, float forward_cm, float *out_cm)
{
    int cx, row;
    if (!to_cell(0.0f, forward_cm, &cx, &row)) return false;
    (void)cx;
    if (map->cells[row][LOCAL_MAP_CENTER] & LOCAL_CELL_OCCUPIED) return false;
    int left = LOCAL_MAP_CENTER, right = LOCAL_MAP_CENTER;
    while (left > 0 && !(map->cells[row][left - 1] & LOCAL_CELL_OCCUPIED)) --left;
    while (right < LOCAL_MAP_SIZE - 1 &&
           !(map->cells[row][right + 1] & LOCAL_CELL_OCCUPIED)) ++right;
    if (left == 0 || right == LOCAL_MAP_SIZE - 1) return false;
    /* Un lado abierto -cruce, o el recorte de un retorno lejano contra el borde
     * del mapa- deja la fila entera sin pared: la resta daria decenas de cm y el
     * coste lateral arrastraria al robot hacia esa referencia falsa. */
    if ((float)(right - left) * LOCAL_MAP_CELL_CM > g->lane_max_span_cm) return false;
    const float left_cm = (float)(LOCAL_MAP_CENTER - left) * LOCAL_MAP_CELL_CM;
    const float right_cm = (float)(right - LOCAL_MAP_CENTER) * LOCAL_MAP_CELL_CM;
    *out_cm = (right_cm - left_cm) * 0.5f;
    return true;
}

/* Apartamiento lateral del centro del carril a la altura del chasis: mediana de
 * tres filas. Una sola fila se contamina con una celda suelta (un bloque ya
 * rebasado, un objeto de camara) y movia el centro decenas de centimetros. Sin
 * dos filas de acuerdo dentro de lane_spread_cm no hay carril y la referencia
 * lateral es cero: la linea propia. Las filas se ORDENAN antes de tomar la
 * mediana y de medir la dispersion: con el orden de captura, un valor atipico
 * en el medio se devolvia como "mediana". */
static bool lane_measured;   /* corridor_center_cm encontro carril en la ultima llamada */

static float corridor_center_cm(const local_obstacle_map_t *map)
{
    lane_measured = false;
    float values[LOCAL_LANE_ROWS];
    int count = 0;
    for (int k = -1; k <= 1; ++k) {
        const float forward_cm = g->chassis_length_cm * 0.5f - g->chassis_rear_cm + (float)k * 4.0f;
        float value = 0.0f;
        if (forward_cm < 0.0f) continue;
        if (!row_center_cm(map, forward_cm, &value)) continue;
        values[count++] = value;
    }
    if (count == 0) return 0.0f;
    for (int i = 1; i < count; ++i) {
        const float v = values[i];
        int j = i - 1;
        while (j >= 0 && values[j] > v) { values[j + 1] = values[j]; --j; }
        values[j + 1] = v;
    }
    if (values[count - 1] - values[0] > g->lane_spread_cm) return 0.0f;
    lane_measured = true;
    return values[count / 2];
}

/* Espacio libre a un lado, medido en la fila del centro del chasis. */
static float lateral_room(const local_obstacle_map_t *map, int direction)
{
    int cx, row;
    if (!to_cell(0.0f, g->chassis_length_cm * 0.5f - g->chassis_rear_cm, &cx, &row)) return 0.0f;
    (void)cx;
    int col = LOCAL_MAP_CENTER;
    float room = 0.0f;
    while (col + direction >= 0 && col + direction < LOCAL_MAP_SIZE &&
           !(map->cells[row][col] & LOCAL_CELL_OCCUPIED)) {
        col += direction;
        room += LOCAL_MAP_CELL_CM;
    }
    return room;
}

/* Holgura de la huella por DETRAS del eje trasero: los mismos tres puntos
 * delanteros con el avance cambiado de signo, y por eso cubren celdas
 * contiguas. Llamar a la version delantera con un avance negativo no medía la
 * parte de atras. */
static float rear_clearance(void)
{
    float worst = 1.0e9f;
    for (int li = 0; li < foot_nx; ++li) {
        const float ox = ((float)li / (float)(foot_nx - 1) - 0.5f) * g->chassis_width_cm;
        for (int fi = 0; fi < foot_nf; ++fi) {
            const float behind_cm = (float)fi / (float)(foot_nf - 1) * g->chassis_length_cm;
            int cx, cy;
            if (!to_cell(ox, -behind_cm, &cx, &cy)) return 0.0f;
            const float d = (float)local_dt[cy][cx] * LOCAL_DT_UNIT_CM;
            if (d < worst) worst = d;
        }
    }
    return worst;
}

static bool rear_clear(void)
{
    return rear_clearance() > 0.0f;
}

/* La camara y la IMU llegan de forma asincrona: pueden ser unos milisegundos
 * mas recientes que el escaneo (el escaneo termina antes de que el P4 lo
 * procese) o mas viejas. Lo que cuenta es cuanto se separan, no el orden. */
static bool fresh(uint64_t now_us, uint64_t sample_us)
{
    const uint64_t gap = now_us >= sample_us ? now_us - sample_us : sample_us - now_us;
    return gap <= (uint64_t)g->sync_max_age_us;
}

static bool to_cell(float x_cm, float forward_cm, int *x, int *y)
{
    int cx = (int)lroundf((float)LOCAL_MAP_CENTER + x_cm / LOCAL_MAP_CELL_CM);
    int cy = (int)lroundf((float)LOCAL_MAP_CENTER - forward_cm / LOCAL_MAP_CELL_CM);
    if (cx == LOCAL_MAP_SIZE) cx = LOCAL_MAP_SIZE - 1;
    if (cy == LOCAL_MAP_SIZE) cy = LOCAL_MAP_SIZE - 1;
    if (cx < 0 || cx >= LOCAL_MAP_SIZE || cy < 0 || cy >= LOCAL_MAP_SIZE) return false;
    *x = cx;
    *y = cy;
    return true;
}

static void mark_occupied(local_obstacle_map_t *map, int x, int y, uint8_t source)
{
    /* Se marca la celda del retorno, nada mas. Dilatar una celda alrededor
     * sumaba ~5 cm al margen que ya impone el campo de distancias. El margen lo
     * aplica ahora la huella al consultar el campo, que es donde se puede
     * calibrar. */
    if (x >= 0 && x < LOCAL_MAP_SIZE && y >= 0 && y < LOCAL_MAP_SIZE)
        map->cells[y][x] = (uint8_t)((map->cells[y][x] & ~LOCAL_CELL_FREE) |
                                     LOCAL_CELL_OCCUPIED | source);
}

static void ray(local_obstacle_map_t *map, int x0, int y0, int x1, int y1, bool hit)
{
    const int start_x = x0, start_y = y0;
    const int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        const bool end = x0 == x1 && y0 == y1;
        if (end) {
            if (hit) mark_occupied(map, x0, y0, 0);
            else map->cells[y0][x0] |= LOCAL_CELL_FREE;
            break;
        }
        if (!(x0 == start_x && y0 == start_y) &&
            !(map->cells[y0][x0] & LOCAL_CELL_OCCUPIED)) map->cells[y0][x0] |= LOCAL_CELL_FREE;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x0 += sx; }
        if (twice <= dx) { error += dx; y0 += sy; }
    }
}

/* Une dos retornos vecinos con celdas ocupadas. A 2,5 m el LiDAR separa los
 * puntos de un muro oblicuo mas de una celda, y el planificador veia un hueco
 * donde hay pared: una salida que no existe. */
static void bridge(local_obstacle_map_t *map, int x0, int y0, int x1, int y1)
{
    const int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        mark_occupied(map, x0, y0, 0);
        if (x0 == x1 && y0 == y1) break;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x0 += sx; }
        if (twice <= dx) { error += dx; y0 += sy; }
    }
}

typedef struct { float x_cm, forward_cm; } hit_t;
static hit_t hits[LOCAL_MAP_MAX_POINTS];
static size_t hit_count;

typedef struct {
    bool hit;
    float x_cm, forward_cm, distance_cm;
} scan_sample_t;

/* Proyecta un punto, marca su rayo y devuelve el retorno si lo hubo. */
static scan_sample_t add_lidar_point(local_obstacle_map_t *map, const local_lidar_point_t *point,
                                     float angle_deg)
{
    scan_sample_t out = {false, 0.0f, 0.0f, 0.0f};
    if (!isfinite(angle_deg)) return out;
    const float a = angle_deg * LOCAL_DEG_TO_RAD;
    if (!point->valid) {
        /* Zona ciega: el retorno esta mas cerca que el minimo del sensor, y
         * seguramente pegado al morro. Ignorarlo dejaba trazar rutas a traves
         * de ello. */
        if (point->blind) {
            int bx, by;
            if (to_cell(sinf(a) * g->lidar_min_cm,
                        g->lidar_forward_cm + cosf(a) * g->lidar_min_cm, &bx, &by))
                mark_occupied(map, bx, by, 0);
        }
        return out;
    }
    if (!isfinite(point->distance_cm) || point->distance_cm <= 0.0f) return out;
    ++map->lidar_points;
    float x_cm = sinf(a) * point->distance_cm;
    float forward_cm = g->lidar_forward_cm + cosf(a) * point->distance_cm;
    const float extent = (float)LOCAL_MAP_CENTER * LOCAL_MAP_CELL_CM;
    const float axis_distance = fmaxf(fabsf(x_cm), fabsf(forward_cm));
    const bool hit = axis_distance <= extent;
    if (!hit) {
        const float scale = extent / axis_distance;
        x_cm *= scale;
        forward_cm *= scale;
    }
    int x, y;
    if (to_cell(x_cm, forward_cm, &x, &y)) {
        int start_x, start_y;
        if (!to_cell(0.0f, g->lidar_forward_cm, &start_x, &start_y)) return out;
        ray(map, start_x, start_y, x, y, hit);
        if (hit) {
            ++map->lidar_hits;
            out.hit = true;
            out.x_cm = x_cm;
            out.forward_cm = forward_cm;
            out.distance_cm = point->distance_cm;
            if (hit_count < LOCAL_MAP_MAX_POINTS) hits[hit_count++] = (hit_t){x_cm, forward_cm};
        }
    }
    return out;
}

/* Une un objeto de color de la camara con el retorno LiDAR que lo respalda. La
 * camara estima el alcance con un error de decenas de por ciento, el LiDAR con
 * un centimetro: la camara dice QUE es, el LiDAR dice DONDE esta. El primer
 * retorno dentro de la ventana angular y de alcance es el pilar (el LiDAR ve la
 * cara frontal). Sin retorno, la deteccion queda como "no confirmada". */
static void associate_camera(local_obstacle_map_t *map, const local_camera_object_t *object)
{
    if (!object->valid || object->color == 0 || !isfinite(object->bearing_deg) ||
        !isfinite(object->distance_cm) || object->distance_cm <= 0.0f) return;
    if (map->colored_count >= LOCAL_MAP_MAX_CAMERA_OBJECTS) return;
    const float a = object->bearing_deg * LOCAL_DEG_TO_RAD;
    const float est_x = sinf(a) * object->distance_cm;
    const float est_f = g->camera_forward_cm + cosf(a) * object->distance_cm;
    const float r = object->distance_cm;
    const float lo = r * (1.0f - g->assoc_range_rel) - g->assoc_range_abs_cm;
    const float hi = r * (1.0f + g->assoc_range_rel) + g->assoc_range_abs_cm;
    /* Un pilar de 5 cm subtiende mas angulo cuanto mas cerca esta. */
    const float half_width_deg = atanf(3.5f / fmaxf(r, 5.0f)) / LOCAL_DEG_TO_RAD;
    const float tol = g->assoc_bearing_tol_deg + half_width_deg;
    size_t best = (size_t)-1;
    float best_range = 1.0e9f;
    for (size_t i = 0; i < hit_count; ++i) {
        const float hx = hits[i].x_cm, hf = hits[i].forward_cm - g->camera_forward_cm;
        const float range = hypotf(hx, hf);
        if (range < lo || range > hi || hf <= 0.0f) continue;
        const float bearing = atan2f(hx, hf) / LOCAL_DEG_TO_RAD;
        if (fabsf(bearing - object->bearing_deg) > tol) continue;
        if (range >= best_range) continue;
        /* Un pilar es COMPACTO: a su alrededor (12 cm) los retornos no se alejan mas de 7,5 cm de el. Una
         * pared o su esquina se prolonga: sin esta prueba una deteccion lejana de la camara se anclaba a la
         * esquina del muro interior y la regla de color empujaba al robot contra ella. */
        bool compact = true;
        for (size_t k = 0; k < hit_count && compact; ++k) {
            const float d = hypotf(hits[k].x_cm - hits[i].x_cm, hits[k].forward_cm - hits[i].forward_cm);
            if (d > g->cluster_radius_cm * 1.25f && d <= 12.0f) compact = false;
        }
        if (!compact) continue;
        best_range = range; best = i;
    }
    local_colored_object_t *entry = &map->colored[map->colored_count++];
    entry->color = object->color;
    if (best == (size_t)-1) {
        entry->x_cm = est_x;
        entry->forward_cm = est_f;
        entry->confirmed = false;
        ++map->camera_unmatched;
        if (g->camera_ghost_fallback) {
            int cx, cy;
            if (to_cell(est_x, est_f, &cx, &cy)) {
                mark_occupied(map, cx, cy, LOCAL_CELL_VISION);
                ++map->camera_hits;
            }
        }
        return;
    }
    float sx = 0.0f, sf = 0.0f;
    int n = 0;
    for (size_t i = 0; i < hit_count; ++i) {
        if (hypotf(hits[i].x_cm - hits[best].x_cm, hits[i].forward_cm - hits[best].forward_cm)
            > g->cluster_radius_cm) continue;
        sx += hits[i].x_cm; sf += hits[i].forward_cm; ++n;
        int cx, cy;
        if (to_cell(hits[i].x_cm, hits[i].forward_cm, &cx, &cy))
            map->cells[cy][cx] |= LOCAL_CELL_VISION;
    }
    entry->x_cm = n ? sx / (float)n : hits[best].x_cm;
    entry->forward_cm = n ? sf / (float)n : hits[best].forward_cm;
    entry->confirmed = true;
    ++map->camera_hits;
}

void local_obstacle_map_update_cfg(local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                   const local_lidar_point_t *points, size_t point_count,
                                   uint64_t scan_timestamp_us,
                                   const local_camera_object_t *objects, size_t object_count,
                                   uint64_t camera_timestamp_us,
                                   const local_imu_sample_t *imu)
{
    if (!map) return;
    use_config(cfg);
    memset(map, 0, sizeof(*map));
    map->timestamp_us = scan_timestamp_us;
    map->imu_fresh = imu && imu->valid && fresh(scan_timestamp_us, imu->timestamp_us);
    if (map->imu_fresh) map->yaw_deg = imu->yaw_deg;
    /* El chasis gira mientras el LiDAR da su vuelta: con el giro duro a 20 cm/s
     * son ~70 grados por segundo, o sea ~7 grados dentro de un barrido y
     * decenas de cm de error en un muro lejano. */
    const bool deskew = g->deskew && map->imu_fresh && imu->has_rate && isfinite(imu->yaw_rate_dps);
    hit_count = 0;
    scan_sample_t previous = {false, 0.0f, 0.0f, 0.0f};
    for (size_t i = 0; points && i < point_count; ++i) {
        float angle = points[i].angle_deg;
        if (deskew && point_count > 1) {
            const float age_s = (1.0f - (float)i / (float)(point_count - 1)) * g->scan_period_s;
            angle -= imu->yaw_rate_dps * age_s;
        }
        const scan_sample_t s = add_lidar_point(map, &points[i], angle);
        if (s.hit && previous.hit &&
            hypotf(s.x_cm - previous.x_cm, s.forward_cm - previous.forward_cm) <= g->bridge_max_gap_cm &&
            fabsf(s.distance_cm - previous.distance_cm) <= g->bridge_max_jump_cm) {
            int ax, ay, bx, by;
            if (to_cell(previous.x_cm, previous.forward_cm, &ax, &ay) &&
                to_cell(s.x_cm, s.forward_cm, &bx, &by))
                bridge(map, ax, ay, bx, by);
        }
        previous = s;
    }
    map->camera_fresh = objects && fresh(scan_timestamp_us, camera_timestamp_us);
    if (map->camera_fresh) {
        const size_t n = object_count < LOCAL_MAP_MAX_CAMERA_OBJECTS
            ? object_count : LOCAL_MAP_MAX_CAMERA_OBJECTS;
        for (size_t i = 0; i < n; ++i) associate_camera(map, &objects[i]);
    }
}

void local_obstacle_map_update(local_obstacle_map_t *map,
                               const local_lidar_point_t *points, size_t point_count,
                               uint64_t scan_timestamp_us,
                               const local_camera_object_t *objects, size_t object_count,
                               uint64_t camera_timestamp_us,
                               const local_imu_sample_t *imu)
{
    local_obstacle_map_update_cfg(map, NULL, points, point_count, scan_timestamp_us,
                                  objects, object_count, camera_timestamp_us, imu);
}

void local_obstacle_map_add_memory_within(local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                          const float *xf, const uint8_t *colors, size_t count, float anchor_cm,
                                          float *found_xf, uint8_t *found)
{
    if (!map || !xf || !colors) return;
    use_config(cfg);
    if (found) memset(found, 0, count);
    for (size_t i = 0; i < count && map->colored_count < LOCAL_MAP_MAX_CAMERA_OBJECTS; ++i) {
        const float x = xf[2 * i], f = xf[2 * i + 1];
        bool seen = false;
        const float seen_r = anchor_cm > 0.0f ? anchor_cm : 2.0f * g->cluster_radius_cm;
        for (size_t k = 0; k < map->colored_count; ++k) {
            const float dk = hypotf(map->colored[k].x_cm - x, map->colored[k].forward_cm - f);
            if (dk < seen_r) {
                seen = true;
                /* Ya hay un objeto con color ahi. Si es del mismo color es un reencuentro (lo ve la camara ahora):
                 * dice donde esta de verdad. Si no, no se ancla nada (seria un fantasma de otro color). */
                if (map->colored[k].color == colors[i]) {
                    if (found) found[i] = 2;
                    if (found_xf) { found_xf[2 * i] = map->colored[k].x_cm; found_xf[2 * i + 1] = map->colored[k].forward_cm; }
                }
                break;
            }
        }
        if (seen) continue;
        /* El recuerdo lleva el error de la estima (giroscopio y velocidad
         * ordenada): se ancla al retorno LiDAR mas cercano a ese sitio. */
        size_t best = (size_t)-1;
        float best_d = anchor_cm > 0.0f ? anchor_cm : 2.0f * g->cluster_radius_cm;
        for (size_t h2 = 0; h2 < hit_count; ++h2) {
            const float d = hypotf(hits[h2].x_cm - x, hits[h2].forward_cm - f);
            if (d < best_d) { best_d = d; best = h2; }
        }
        if (best == (size_t)-1) continue;
        float sx = 0.0f, sf = 0.0f;
        float minx = 1.0e9f, maxx = -1.0e9f, minf = 1.0e9f, maxf = -1.0e9f;
        int n = 0;
        for (size_t h2 = 0; h2 < hit_count; ++h2) {
            if (hypotf(hits[h2].x_cm - hits[best].x_cm, hits[h2].forward_cm - hits[best].forward_cm) > g->cluster_radius_cm) continue;
            sx += hits[h2].x_cm; sf += hits[h2].forward_cm; ++n;
            if (hits[h2].x_cm < minx) minx = hits[h2].x_cm;
            if (hits[h2].x_cm > maxx) maxx = hits[h2].x_cm;
            if (hits[h2].forward_cm < minf) minf = hits[h2].forward_cm;
            if (hits[h2].forward_cm > maxf) maxf = hits[h2].forward_cm;
        }
        /* Un pilar es un grupo compacto (5 cm de lado); un tramo de pared no. Sin
         * esto, un recuerdo desviado se enganchaba a la pared y cerraba el carril. */
        if (maxx - minx > 10.0f || maxf - minf > 10.0f) continue;
        local_colored_object_t *e = &map->colored[map->colored_count++];
        e->x_cm = n ? sx / (float)n : hits[best].x_cm;
        e->forward_cm = n ? sf / (float)n : hits[best].forward_cm;
        e->color = colors[i];
        e->confirmed = true;
        if (found) found[i] = 1;
        if (found_xf) { found_xf[2 * i] = e->x_cm; found_xf[2 * i + 1] = e->forward_cm; }
    }
}

void local_obstacle_map_add_memory(local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                   const float *xf, const uint8_t *colors, size_t count)
{
    local_obstacle_map_add_memory_within(map, cfg, xf, colors, count, 0.0f, NULL, NULL);
}

bool local_obstacle_map_unlabeled_pillar(const local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                         float min_bearing_deg, float max_bearing_deg,
                                         float min_forward_cm, float max_forward_cm,
                                         float *x_cm, float *forward_cm)
{
    if (!map || !x_cm || !forward_cm) return false;
    use_config(cfg);
    float best_f = 1.0e9f, bx = 0.0f;
    for (size_t i = 0; i < hit_count; ++i) {
        const float x = hits[i].x_cm, f = hits[i].forward_cm;
        if (f < min_forward_cm || f > max_forward_cm || f >= best_f) continue;
        const float bearing = fabsf(atan2f(x, f)) / LOCAL_DEG_TO_RAD;
        if (bearing < min_bearing_deg || bearing > max_bearing_deg) continue;
        /* Compacto: un grupo de hasta 10 cm de lado y varios retornos (una pared no lo es). */
        int n = 0;
        float sx = 0.0f, sf = 0.0f, minx = 1.0e9f, maxx = -1.0e9f, minf = 1.0e9f, maxf = -1.0e9f;
        bool wide = false;
        for (size_t h2 = 0; h2 < hit_count; ++h2) {
            const float d = hypotf(hits[h2].x_cm - x, hits[h2].forward_cm - f);
            if (d > g->cluster_radius_cm) { if (d <= 14.0f) wide = true; continue; }
            ++n; sx += hits[h2].x_cm; sf += hits[h2].forward_cm;
            if (hits[h2].x_cm < minx) minx = hits[h2].x_cm;
            if (hits[h2].x_cm > maxx) maxx = hits[h2].x_cm;
            if (hits[h2].forward_cm < minf) minf = hits[h2].forward_cm;
            if (hits[h2].forward_cm > maxf) maxf = hits[h2].forward_cm;
        }
        if (n < 3 || wide || maxx - minx > 10.0f || maxf - minf > 10.0f) continue;
        const float cx = sx / (float)n, cf = sf / (float)n;
        bool labeled = false;
        for (size_t k = 0; k < map->colored_count; ++k)
            if (hypotf(map->colored[k].x_cm - cx, map->colored[k].forward_cm - cf) < 16.0f) labeled = true;
        if (labeled) continue;
        best_f = f; bx = cx;
        *forward_cm = cf;
    }
    if (best_f >= 1.0e8f) return false;
    *x_cm = bx;
    return true;
}

size_t local_obstacle_map_compact_pillars(const local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                          float min_forward_cm, float max_forward_cm, float *xf_pairs, size_t capacity)
{
    if (!map || !xf_pairs || capacity == 0) return 0;
    use_config(cfg);
    size_t found = 0;
    for (size_t i = 0; i < hit_count && found < capacity; ++i) {
        const float x = hits[i].x_cm, f = hits[i].forward_cm;
        if (f < min_forward_cm || f > max_forward_cm) continue;
        int n = 0;
        float sx = 0.0f, sf = 0.0f, minx = 1.0e9f, maxx = -1.0e9f, minf = 1.0e9f, maxf = -1.0e9f;
        bool wide = false;
        for (size_t h2 = 0; h2 < hit_count; ++h2) {
            const float d = hypotf(hits[h2].x_cm - x, hits[h2].forward_cm - f);
            if (d > g->cluster_radius_cm) { if (d <= 14.0f) wide = true; continue; }
            ++n; sx += hits[h2].x_cm; sf += hits[h2].forward_cm;
            if (hits[h2].x_cm < minx) minx = hits[h2].x_cm;
            if (hits[h2].x_cm > maxx) maxx = hits[h2].x_cm;
            if (hits[h2].forward_cm < minf) minf = hits[h2].forward_cm;
            if (hits[h2].forward_cm > maxf) maxf = hits[h2].forward_cm;
        }
        if (n < 3 || wide || maxx - minx > 10.0f || maxf - minf > 10.0f) continue;
        const float cx = sx / (float)n, cf = sf / (float)n;
        bool dup = false;
        for (size_t k = 0; k < found; ++k)
            if (hypotf(xf_pairs[2 * k] - cx, xf_pairs[2 * k + 1] - cf) < 8.0f) dup = true;
        if (dup) continue;
        xf_pairs[2 * found] = cx; xf_pairs[2 * found + 1] = cf;
        ++found;
    }
    return found;
}

bool local_obstacle_map_get(const local_obstacle_map_t *map,
                            int x_cell, int forward_cell, uint8_t *value)
{
    if (!map || !value || x_cell < -LOCAL_MAP_CENTER || x_cell >= LOCAL_MAP_CENTER ||
        forward_cell < -LOCAL_MAP_CENTER || forward_cell >= LOCAL_MAP_CENTER) return false;
    *value = map->cells[LOCAL_MAP_CENTER - forward_cell][LOCAL_MAP_CENTER + x_cell];
    return true;
}

static float corridor_clearance(const local_obstacle_map_t *map, float angle_deg)
{
    /* El paseo arranca en el lidar, no en el eje: los retornos se guardan a
     * lidar_forward_cm por delante del eje con el angulo del sensor. */
    const float a = angle_deg * LOCAL_DEG_TO_RAD;
    const float dx = sinf(a), df = cosf(a), side_x = cosf(a), side_f = -sinf(a);
    for (float distance = LOCAL_MAP_CELL_CM; distance <= 45.0f; distance += LOCAL_MAP_CELL_CM) {
        for (int offset = -1; offset <= 1; ++offset) {
            const float lateral = (float)offset * LOCAL_MAP_CELL_CM;
            const float x_cm = dx * distance + side_x * lateral;
            const float forward_cm = g->lidar_forward_cm + df * distance + side_f * lateral;
            int x, y;
            if (!to_cell(x_cm, forward_cm, &x, &y)) return distance - LOCAL_MAP_CELL_CM;
            if (map->cells[y][x] & LOCAL_CELL_OCCUPIED) return distance - LOCAL_MAP_CELL_CM;
        }
    }
    return 45.0f;
}

bool local_planner_arc_clear(const local_obstacle_map_t *map, const local_planner_config_t *cfg,
                             float steer_deg, float distance_cm)
{
    if (!map) return false;
    use_config(cfg);
    build_distance_field(map);
    const float k = tanf(steer_deg * LOCAL_DEG_TO_RAD) / g->wheelbase_cm;
    return arc_clear(k, distance_cm);
}

float local_planner_axle_clearance_cm(void)
{
    int cx, cy;
    if (!to_cell(0.0f, 0.0f, &cx, &cy)) return 0.0f;
    return (float)local_dt[cy][cx] * LOCAL_DT_UNIT_CM;
}

bool local_planner_rear_clear(const local_obstacle_map_t *map, const local_planner_config_t *cfg)
{
    if (!map) return false;
    use_config(cfg);
    build_distance_field(map);
    return rear_clear();
}

/* ------------------------------------------------- regla de color: cortinas
 *
 * La regla "rojo por la derecha, verde por la izquierda" no es un gusto: pasar
 * por el lado equivocado termina la ronda. Sumar un coste pequeno no la hace
 * cumplir (la banda de empate lo absorbe), asi que se cierra el lado
 * prohibido: desde cada pilar confirmado por el LiDAR se tiende una pared
 * virtual hacia el lado equivocado, perpendicular al rumbo del recorrido, hasta
 * topar con una pared real. La ruta solo puede pasar por el lado correcto, y el
 * LiDAR sigue decidiendo cuanto sitio hay. El lado de cada color es DATO
 * (config), no codigo. */
static size_t apply_curtains(const local_obstacle_map_t *map, float reference_rad, float lane_cm, bool lane_known)
{
    if (!g->enforce_color_side || g->curtain_len_cm <= 0.0f) return 0;
    const float ux = cosf(reference_rad), uf = -sinf(reference_rad); /* derecha del recorrido */
    const float vx = sinf(reference_rad), vf = cosf(reference_rad);  /* adelante del recorrido */
    size_t marked = 0;
    const float rear = g->chassis_rear_cm, front = g->chassis_length_cm - g->chassis_rear_cm;
    const float circum = hypotf(rear > front ? rear : front, 0.5f * g->chassis_width_cm) + 1.0f;
    for (size_t i = 0; i < map->colored_count; ++i) {
        const local_colored_object_t *o = &map->colored[i];
        int side = 0;
        if (o->color == 1) side = g->pass_side_red;
        else if (o->color == 2) side = g->pass_side_green;
        if (side == 0) continue;
        if (!o->confirmed && !g->curtain_unconfirmed) continue;
        if (o->forward_cm < g->curtain_min_forward_cm) continue;
        /* Solo los letreros de ESTE carril: uno visto por encima de la esquina del muro interior (otra recta) con el
         * rumbo de esta cortaria el paso con una cortina que no le corresponde. */
        const float lateral = o->x_cm * ux + o->forward_cm * uf - lane_cm;
        if (fabsf(lateral) > g->lane_half_cm + (lane_known ? 35.0f : 50.0f)) continue;
        /* Se pasa por la derecha (side > 0): el lado prohibido es la izquierda. */
        const float dir = side > 0 ? -1.0f : 1.0f;
        for (float t = LOCAL_MAP_CELL_CM * 0.75f; t <= g->curtain_len_cm; t += LOCAL_MAP_CELL_CM * 0.5f) {
            bool wall = false;
            for (int k = -2; k <= 2; ++k) {
                const float x = o->x_cm + dir * ux * t + vx * (float)k * LOCAL_MAP_CELL_CM;
                const float f = o->forward_cm + dir * uf * t + vf * (float)k * LOCAL_MAP_CELL_CM;
                int cx, cy;
                if (!to_cell(x, f, &cx, &cy)) { wall = true; continue; }
                if (map->cells[cy][cx] & LOCAL_CELL_OCCUPIED) { wall = true; continue; }
                /* La cortina no se pinta encima del propio robot: si el letrero esta pegado (o el robot mas girado
                 * del carril de lo debido) la cortina ocupa la huella y no queda ninguna ruta, ni para alejarse. */
                if (hypotf(x, f) < circum) continue;
                if (local_dt[cy][cx] != 0) { local_dt[cy][cx] = 0; ++marked; }
            }
            if (wall && t > g->chassis_width_cm) break;
        }
    }
    return marked;
}


/* ------------------------------------------------------------------ corredor
 *
 * Trayectoria lateral v(u) en el sistema del carril: u hacia delante del rumbo
 * del recorrido, v hacia su derecha. Cada fila (5 cm) admite las posiciones
 * donde el centro del robot deja media anchura mas un margen libre, segun el
 * mismo campo de distancias que usa la retícula (paredes, pilares y cortinas de
 * color incluidas). Una pasada de programacion dinamica elige, entre filas, la
 * trayectoria de menor coste: desvio del centro del carril, holgura por debajo de
 * la preferida y cambio de posicion entre filas (pendiente <= 0,5). */
#define PLAN_ROWS 36
#define PLAN_COLS 97
#define PLAN_DU 5.0f
#define PLAN_DV 2.5f

typedef struct {
    bool ok;
    int rows;
    float target_x, target_f;     /* punto de persecucion, ejes del robot */
    float x[PLAN_ROWS], f[PLAN_ROWS];
} corridor_t;

static corridor_t corridor;
/* Referencia lateral de la linea de carrera para este ciclo (la copia plan_corridor desde el estado). */
static const float *plan_ref_v;
static int plan_ref_rows;
/* El corredor como polilinea desde el robot, para puntuar rutas contra el. */
static float corridor_px[PLAN_ROWS + 1], corridor_pf[PLAN_ROWS + 1];
static float corridor_tangent[PLAN_ROWS + 1];
static int path_n;

static void plan_corridor_margin(float reference_rad, float lane_cm, float margin_cm)
{
    memset(&corridor, 0, sizeof(corridor));
    path_n = 0;
    if (!g->plan_enabled) return;
    const int rows = (int)fminf((float)PLAN_ROWS, g->plan_horizon_cm / PLAN_DU);
    const int mid = PLAN_COLS / 2;
    const float require = g->chassis_width_cm * 0.5f + margin_cm;
    const float sd = sinf(reference_rad), cd = cosf(reference_rad);
    static float cost[2][PLAN_COLS];
    static int8_t step[PLAN_ROWS][PLAN_COLS];
    static float clr[PLAN_ROWS][PLAN_COLS];
    const float inf = 1.0e9f;
    int last_row = -1, best_col = mid;
    for (int k = 0; k < rows; ++k) {
        const float u = (float)(k + 1) * PLAN_DU;
        const int cur = k & 1, prev = cur ^ 1;
        float row_best = inf;
        int row_col = mid;
        for (int j = 0; j < PLAN_COLS; ++j) {
            cost[cur][j] = inf;
            const float v = (float)(j - mid) * PLAN_DV;
            /* El chasis ocupa desde el eje trasero (u) hasta la nariz (u + largo): se
             * comprueba la holgura en tres puntos de su longitud, no solo en el eje. */
            float clear = 1.0e9f;
            bool inside = true;
            for (int oi = 0; oi < 3 && inside; ++oi) {
                const float uo = u - g->chassis_rear_cm + (float)oi * 0.5f * g->chassis_length_cm;
                int cx, cy;
                if (!to_cell(uo * sd + v * cd, uo * cd - v * sd, &cx, &cy)) { inside = false; break; }
                const float c = (float)local_dt[cy][cx] * LOCAL_DT_UNIT_CM;
                if (c < clear) clear = c;
            }
            if (!inside || clear < require) continue;
            clr[k][j] = clear;
            float pen = 0.0f;
            const bool has_ref = plan_ref_v && k < plan_ref_rows && isfinite(plan_ref_v[k]);
            const float dn = (v - (has_ref ? plan_ref_v[k] : lane_cm)) / g->lateral_scale_cm;
            pen += (has_ref ? g->plan_w_lateral * g->plan_w_race : g->plan_w_lateral) * dn * dn;
            const float pref_clear = has_ref ? g->plan_race_clear_cm : g->clearance_pref_cm;
            if (clear < pref_clear) {
                const float t = (pref_clear - clear) / pref_clear;
                pen += g->plan_w_clearance * t * t;
            }
            float best = inf;
            int8_t from = 0;
            if (k == 0) {
                /* Desde el robot, a lo sumo dos pasos de pendiente. */
                if (abs(j - mid) <= 2) { best = pen + g->plan_w_slew * (float)((j - mid) * (j - mid)); from = 0; }
            } else {
                /* Hasta 45 grados entre filas: la retícula, que conoce el radio de
                 * giro, decide si el chasis puede seguirlo; aqui solo se marca por
                 * donde hay paso. */
                for (int d = -2; d <= 2; ++d) {
                    const int jp = j + d;
                    if (jp < 0 || jp >= PLAN_COLS || cost[prev][jp] >= inf) continue;
                    const float c = cost[prev][jp] + pen + g->plan_w_slew * (float)(d * d);
                    if (c < best) { best = c; from = (int8_t)d; }
                }
            }
            cost[cur][j] = best;
            step[k][j] = from;
            if (best < row_best) { row_best = best; row_col = j; }
        }
        if (row_best >= inf) break;
        last_row = k;
        best_col = row_col;
    }
    /* Hace falta carrera suficiente para fiarse: menos de la mitad del horizonte
     * libre es una pared o un cierre, y eso lo decide la retícula. */
    if (last_row + 1 < rows / 2 + 1) return;
    int col = best_col;
    static float vs[PLAN_ROWS];
    for (int k = last_row; k >= 0; --k) {
        vs[k] = (float)(col - mid) * PLAN_DV;
        col += step[k][col];
    }
    corridor.rows = last_row + 1;
    for (int k = 0; k <= last_row; ++k) {
        const float u = (float)(k + 1) * PLAN_DU;
        corridor.x[k] = u * sd + vs[k] * cd;
        corridor.f[k] = u * cd - vs[k] * sd;
    }
    /* Punto de persecucion: a plan_lookahead_cm a lo largo del carril, pero no mas
     * alla del primer tramo estrecho: apuntar a un punto situado DESPUES de un
     * pilar corta la esquina y mete el costado del chasis en el. Dentro del tramo
     * (o pegado a el) se persigue lo que esta mas cerca. */
    float lookahead = g->plan_lookahead_cm;
    {
        int c2 = best_col;
        int path_col[PLAN_ROWS];
        for (int k = last_row; k >= 0; --k) { path_col[k] = c2; c2 += step[k][c2]; }
        for (int k = 0; k <= last_row; ++k) {
            if (clr[k][path_col[k]] < g->clearance_pref_cm) {
                const float u_narrow = (float)(k + 1) * PLAN_DU;
                if (u_narrow < lookahead) lookahead = u_narrow;
                break;
            }
        }
        if (lookahead < 15.0f) lookahead = 15.0f;
    }
    int ti = (int)(lookahead / PLAN_DU + 0.5f) - 1;
    if (ti < 0) ti = 0;
    if (ti > last_row) ti = last_row;
    corridor.target_x = corridor.x[ti];
    corridor.target_f = corridor.f[ti];
    corridor.ok = true;
    corridor_px[0] = 0.0f; corridor_pf[0] = 0.0f;
    for (int k = 0; k <= last_row; ++k) { corridor_px[k + 1] = corridor.x[k]; corridor_pf[k + 1] = corridor.f[k]; }
    path_n = last_row + 2;
    /* La tangente depende solo del corredor, no de las 343 rutas evaluadas. */
    for (int i = 0; i < path_n; ++i) {
        const int j = i + 1 < path_n ? i : i - 1;
        corridor_tangent[i] = atan2f(corridor_px[j + 1] - corridor_px[j],
                                     corridor_pf[j + 1] - corridor_pf[j]);
    }
}

/* Con sitio se pasa con holgura (el seguimiento con acciones discretas no mantiene el centimetro: un paso planeado
 * justo acaba rozando el pilar); si no cabe asi, con el margen justo. */
static void plan_corridor(float reference_rad, float lane_cm)
{
    if (g->plan_margin_extra_cm > 0.0f) {
        plan_corridor_margin(reference_rad, lane_cm, g->plan_margin_cm + g->plan_margin_extra_cm);
        if (corridor.ok) return;
    }
    plan_corridor_margin(reference_rad, lane_cm, g->plan_margin_cm);
}

/* --------------------------------------------------------- planificador ---
 *
 * En vez de preguntar "hay 30 cm a +-35 grados?", la reticula rueda rutas
 * completas y las puntua. Cada accion del enum ya es una curvatura constante,
 * asi que planificar es rodar arcos: no hace falta buscar rutas en un grafo ni
 * reservar memoria dinamica. La ruta se corta en el primer choque y se compara
 * por cuanto avanza, con cuanta holgura, cuanto se desvia y cuanto cuesta
 * cambiar de decision.
 *
 * Coste por escaneo: ~20k celdas de campo de distancias + 125 rutas x 30 pasos
 * (12 de horizonte y hasta 18 de sonda) x 9 muestras de huella. Muy por debajo
 * del presupuesto de 10 Hz.
 */

typedef struct {
    float cost;
    float mean_k;
    float pass_cm;      /* distancia libre por delante siguiendo la ruta */
    bool viable;
    uint8_t first;
    uint8_t seq[LOCAL_LATTICE_DEPTH];
} local_route_t;

static local_route_t route_cost(const uint8_t *actions, uint8_t camera_action,
                                float reference_rad, float lane_cm)
{
    const float step = LOCAL_LATTICE_STEP_CM;
    const int arc_steps = (int)(g->arc_cm / step + 0.5f);
    const int horizon_steps = LOCAL_LATTICE_DEPTH * arc_steps;
    /* Pasado el horizonte la ruta se prolonga con la curvatura de su ultima
     * accion hasta pass_probe_cm, y ese tramo solo alimenta la sonda: el coste
     * sigue puntuando lo que se ve en el horizonte. */
    int probe_steps = (int)(g->pass_probe_cm / step + 0.5f);
    if (probe_steps < horizon_steps) probe_steps = horizon_steps;
    const float horizon_cm = (float)horizon_steps * step;
    const float cos_ref = cosf(reference_rad), sin_ref = sinf(reference_rad);
    const float k_steer_hard = k_hard;
    const bool use_path = path_n >= 3;
    float theta = 0.0f, x_cm = 0.0f, forward_cm = 0.0f;
    float weight = 1.0f, weight_sum = 0.0f;
    float penalty = 0.0f, lateral = 0.0f, heading = 0.0f;
    float curvature_sum = 0.0f, curvature_change = 0.0f, last_k = 0.0f;
    int done = 0, pass_steps = 0;
    bool blocked = false;
    local_route_t route = { .cost = 0.0f, .mean_k = 0.0f, .pass_cm = 0.0f,
                            .viable = false, .first = actions[0] };
    memcpy(route.seq, actions, LOCAL_LATTICE_DEPTH);

    for (int s = 0; s < probe_steps && !blocked; ++s) {
        const int arc_index = (s < horizon_steps) ? (s / arc_steps)
                                                  : (LOCAL_LATTICE_DEPTH - 1);
        const float k = action_curvature(actions[arc_index]);
        curvature_change += fabsf(k - last_k);
        last_k = k;
        theta += k * step;
        x_cm += sinf(theta) * step;
        forward_cm += cosf(theta) * step;
        const float clearance = footprint_clearance(x_cm, forward_cm, theta);
        if (clearance <= 0.0f) { blocked = true; break; }
        ++pass_steps;
        if (s >= horizon_steps) continue;
        weight_sum += weight;
        const float shortfall = g->clearance_pref_cm - clearance;
        if (shortfall > 0.0f) {
            const float term = shortfall / g->clearance_pref_cm;
            penalty += weight * term * term;
        }
        /* Desvio lateral respecto a la linea del carril. La linea pasa por el
         * robot -no por el punto de horizonte- y lleva la direccion de rumbo del
         * recorrido, desplazada lateralmente por `lane_cm`: asi un error de
         * rumbo tambien cuenta como desvio lateral. */
        if (use_path) {
            /* Distancia al corredor y diferencia con su tangente, en el punto del
             * corredor mas cercano a esta muestra de la ruta. */
            float best_d2 = 1.0e12f;
            int nearest = 0;
            for (int pi = 0; pi < path_n; ++pi) {
                const float dx = x_cm - corridor_px[pi], df = forward_cm - corridor_pf[pi];
                const float d2 = dx * dx + df * df;
                if (d2 < best_d2) { best_d2 = d2; nearest = pi; }
            }
            const float tangent = corridor_tangent[nearest];
            const float dn = sqrtf(best_d2) / g->plan_path_scale_cm;
            lateral += weight * g->plan_w_path * dn * dn;
            float dth = theta - tangent;
            while (dth > 3.14159265f) dth -= 6.2831853f;
            while (dth < -3.14159265f) dth += 6.2831853f;
            const float hn = dth / (g->steer_hard_deg * LOCAL_DEG_TO_RAD);
            heading += weight * hn * hn;
        } else {
            const float deviation = (x_cm - lane_cm) * cos_ref - forward_cm * sin_ref;
            const float dn = deviation / g->lateral_scale_cm;
            lateral += weight * dn * dn;
            const float hn = (theta - reference_rad) / (g->steer_hard_deg * LOCAL_DEG_TO_RAD);
            heading += weight * hn * hn;
        }
        curvature_sum += fabsf(k);
        ++done;
        weight *= g->discount;
    }

    route.pass_cm = (float)pass_steps * step;     /* aunque no sea viable: dice cuanto avanzaria libre */
    if (done == 0 || (float)done * step < g->min_commit_cm)
        return route;
    route.pass_cm = (float)pass_steps * step;
    /* La sonda larga estima si una ruta termina en un bolsillo, pero no puede
     * vetar por si sola una maniobra delantera segura: la siguiente lectura
     * LiDAR replanea desde la nueva posicion. */
    route.mean_k = curvature_sum / (float)done;
    const float reached = (float)done / (float)horizon_steps;
    const float pass_shortfall = route.pass_cm < g->min_pass_cm
        ? (g->min_pass_cm - route.pass_cm) / g->min_pass_cm : 0.0f;
    float cost = g->w_clearance * penalty / weight_sum
        + g->w_lateral * lateral / weight_sum
        + g->w_heading * heading / weight_sum
        + g->w_turn * route.mean_k / k_steer_hard
        + g->w_change * curvature_change / k_steer_hard
        + g->w_dead_end * (1.0f - reached)
        + g->w_pass_shortfall * pass_shortfall * pass_shortfall
        - g->w_progress * fminf(forward_cm, horizon_cm) / horizon_cm;
    if (camera_action != LOCAL_ACTION_STOP &&
        action_turn_dir(actions[0]) != action_turn_dir(camera_action))
        cost += g->w_camera;
    route.cost = cost;
    route.viable = true;
    return route;
}

/* Eleccion entre dos rutas: gana la claramente mejor y, dentro de la banda de
 * empate, la decision anterior antes que cambiar; si ninguna es la anterior, el
 * giro mas suave, que en un empate es seguir recto. */
static bool route_better(const local_route_t *candidate, const local_route_t *best,
                         uint8_t previous)
{
    const float delta = candidate->cost - best->cost;
    if (delta < -g->switch_tie) return true;
    if (delta > g->switch_tie) return false;
    const bool keep_candidate = candidate->first == previous;
    const bool keep_best = best->first == previous;
    if (keep_candidate != keep_best) return keep_candidate;
    const float a = fabsf(candidate->mean_k), b = fabsf(best->mean_k);
    if (fabsf(a - b) > LOCAL_COST_EPS) return a < b;
    return candidate->first < best->first;
}

static void fill_debug_route(local_planner_debug_t *d, const uint8_t *seq)
{
    const float step = LOCAL_LATTICE_STEP_CM;
    const int arc_steps = (int)(g->arc_cm / step + 0.5f);
    const int horizon_steps = LOCAL_LATTICE_DEPTH * arc_steps;
    int probe_steps = (int)(g->pass_probe_cm / step + 0.5f);
    float theta = 0.0f, x = 0.0f, f = 0.0f;
    d->route_count = 0;
    d->probe_count = 0;
    d->pass_cm = 0.0f;
    d->min_clear_cm = 1.0e9f;
    if (!seq) { d->pass_cm = -1.0f; d->min_clear_cm = 0.0f; return; }
    for (int s = 0; s < probe_steps; ++s) {
        const int arc_index = s < horizon_steps ? s / arc_steps : LOCAL_LATTICE_DEPTH - 1;
        theta += action_curvature(seq[arc_index]) * step;
        x += sinf(theta) * step;
        f += cosf(theta) * step;
        const float clear_here = footprint_clearance(x, f, theta);
        if (clear_here <= 0.0f) break;
        if (s < horizon_steps && clear_here < d->min_clear_cm) d->min_clear_cm = clear_here;
        d->pass_cm += step;
        if (s < horizon_steps) {
            if (d->route_count < LOCAL_DEBUG_ROUTE_POINTS) {
                d->route[d->route_count][0] = x;
                d->route[d->route_count][1] = f;
                ++d->route_count;
            }
        } else if (d->probe_count < LOCAL_DEBUG_ROUTE_POINTS) {
            d->probe[d->probe_count][0] = x;
            d->probe[d->probe_count][1] = f;
            ++d->probe_count;
        }
    }
}

#define CAND_COUNT 7
enum { IDX_FORWARD, IDX_NUDGE_R, IDX_NUDGE_L, IDX_SOFT_R, IDX_HARD_R, IDX_SOFT_L, IDX_HARD_L };
static const uint8_t candidates[CAND_COUNT] = {
    LOCAL_ACTION_FORWARD, LOCAL_ACTION_NUDGE_RIGHT, LOCAL_ACTION_NUDGE_LEFT,
    LOCAL_ACTION_RIGHT_SOFT, LOCAL_ACTION_RIGHT_HARD,
    LOCAL_ACTION_LEFT_SOFT, LOCAL_ACTION_LEFT_HARD
};

static uint8_t lattice_action(local_planner_state_t *state,
                              uint8_t camera_action, float reference_rad, float lane_cm,
                              bool course_known, const uint8_t **chosen_seq)
{
    static local_route_t per_first[CAND_COUNT];
    static local_route_t best_route;
    uint8_t actions[LOCAL_LATTICE_DEPTH];
    local_route_t best = { .cost = 0.0f, .mean_k = 0.0f, .pass_cm = 0.0f,
                           .viable = false, .first = 0u };
    /* Mejor ruta que empieza por cada accion: es lo que necesita la vuelta a la
     * linea para saber si el giro que corrige tiene por donde. */
    bool first_viable[CAND_COUNT] = { false };
    bool found = false;
    float free_cm = 0.0f;       /* lo mas lejos que llega libre cualquier ruta candidata */
    *chosen_seq = NULL;

    for (int a = 0; a < CAND_COUNT; ++a) {
        actions[0] = candidates[a];
        for (int b = 0; b < CAND_COUNT; ++b) {
            actions[1] = candidates[b];
            for (int c = 0; c < CAND_COUNT; ++c) {
                actions[2] = candidates[c];
                const local_route_t route = route_cost(actions, camera_action,
                                                       reference_rad, lane_cm);
                if (route.pass_cm > free_cm) free_cm = route.pass_cm;
                if (!route.viable) continue;
                if (!first_viable[a] ||
                    route_better(&route, &per_first[a], state->last_action)) {
                    per_first[a] = route;
                    first_viable[a] = true;
                }
                if (!found || route_better(&route, &best, state->last_action)) {
                    best = route;
                    found = true;
                }
            }
        }
    }
    if (state->debug) state->debug->free_cm = free_cm;
    if (!found) {
        /* Si las rutas de varios centimetros oscilan entre ciclos, conservar
         * un paso del mando delantero anterior mientras su arco inmediato siga
         * libre evita alternar FORWARD/REVERSE por ruido de una celda. */
        const uint8_t previous = state->last_action;
        if (previous >= LOCAL_ACTION_FORWARD && previous <= LOCAL_ACTION_LEFT_HARD &&
            arc_clear(action_curvature(previous), step_guard_cm()))
            return previous;
        return LOCAL_ACTION_STOP;
    }

    /* Vuelta a la linea. El rumbo del recorrido y el carril son la misma
     * correccion vista dos veces: el rumbo dice hacia donde apunta el chasis y
     * el carril cuanto se ha desviado del centro. Se suman en una sola magnitud
     * para que no se contradigan cuando el robot esta a un lado de la linea pero
     * apuntando al otro.
     *
     * Solo se corrige con el camino recto libre: si el carril esta tapado (una
     * fila de bloques al frente), la desviacion no es un error de rumbo sino la
     * unica salida que hay. */
    uint8_t choice = best.first;
    int choice_index = -1;
    const float heading_deg = course_known ? reference_rad / LOCAL_DEG_TO_RAD : 0.0f;
    const float recenter_cm = lane_cm + heading_deg * g->recenter_deg_to_cm;
    const bool line_clear = arc_clear(0.0f, g->recenter_clear_cm);
    if (!corridor.ok && line_clear && fabsf(recenter_cm) > g->recenter_lane_cm) {
        const bool lane_seen = fabsf(lane_cm) > g->recenter_lane_cm;
        const bool hard = lane_seen ? fabsf(lane_cm) > g->recenter_hard_cm
            : fabsf(heading_deg) > g->recenter_hard_deg;
        const int index = recenter_cm > 0.0f ? (hard ? IDX_HARD_R : IDX_SOFT_R)
                                             : (hard ? IDX_HARD_L : IDX_SOFT_L);
        const float slack = lane_seen ? g->recenter_slack : g->recenter_slack_heading;
        if (first_viable[index] &&
            per_first[index].cost <= best.cost + slack) {
            choice = candidates[index];
            choice_index = index;
        }
    }
    /* Corredor: si hay trayectoria lateral que cabe, el volante la persigue
     * (pursuit puro hacia un punto a plan_lookahead_cm) con la accion viable de
     * curvatura mas cercana. Es lo que permite empezar a desplazarse ANTES de que
     * el pilar entre en el horizonte de 60 cm de la retícula. */
    bool corridor_used = false, have_cont = false;
    float cont_k = 0.0f;
    /* Con corredor, la ruta ya esta puntuada contra el: manda la retícula. El
     * pursuit puro de abajo solo sirve de refinamiento del volante. */
    if (corridor.ok && !g->plan_refine) { corridor_used = true; }
    else if (corridor.ok) {
        const float dist = hypotf(corridor.target_x, corridor.target_f);
        const float alpha = atan2f(corridor.target_x, corridor.target_f);
        const float k_want = dist > 1.0f ? 2.0f * sinf(alpha) / dist : 0.0f;
        const float mag = fabsf(k_want);
        /* Orden de preferencia: la accion mas cercana a la curvatura pedida, y
         * si no tiene por donde, las mas suaves. */
        int idx = IDX_FORWARD;
        if (mag >= 0.5f * (k_soft + k_hard)) idx = k_want > 0.0f ? IDX_HARD_R : IDX_HARD_L;
        else if (mag >= 0.5f * (k_nudge + k_soft)) idx = k_want > 0.0f ? IDX_SOFT_R : IDX_SOFT_L;
        else if (mag >= 0.5f * k_nudge) idx = k_want > 0.0f ? IDX_NUDGE_R : IDX_NUDGE_L;
        while (idx != IDX_FORWARD &&
               !(first_viable[idx] && per_first[idx].cost <= best.cost + g->plan_slack)) {
            if (idx == IDX_HARD_R) idx = IDX_SOFT_R;
            else if (idx == IDX_HARD_L) idx = IDX_SOFT_L;
            else if (idx == IDX_SOFT_R) idx = IDX_NUDGE_R;
            else if (idx == IDX_SOFT_L) idx = IDX_NUDGE_L;
            else idx = IDX_FORWARD;
        }
        if (first_viable[idx] && per_first[idx].cost <= best.cost + g->plan_slack) {
            choice = candidates[idx];
            choice_index = idx;
            corridor_used = true;
            /* La curvatura pedida, sin cuantizar, acotada por la accion viable mas
             * cercana (un 70 % mas, como mucho): evita el subvirado de elegir
             * siempre el escalon de abajo. */
            const float cap = idx == IDX_FORWARD ? 0.0f
                : fminf(k_hard, 1.7f * fabsf(action_curvature(candidates[idx])));
            cont_k = fmaxf(-cap, fminf(cap, k_want));
            have_cont = true;
        }
    }
    /* Seguimiento fino: la accion de menor curvatura que lleva el chasis a la
     * linea del carril en `track_lookahead_cm`. */
    if (!corridor_used && line_clear) {
        const float want = (course_known ? reference_rad : 0.0f) + atanf(lane_cm / g->track_lookahead_cm);
        if (fabsf(want) > g->track_deadband_deg * LOCAL_DEG_TO_RAD) {
            const float k_want = want / g->track_lookahead_cm, mag = fabsf(k_want);
            (void)k_want;
            int idx = IDX_FORWARD;
            if (mag >= 0.5f * (k_soft + k_hard)) idx = want > 0.0f ? IDX_HARD_R : IDX_HARD_L;
            else if (mag >= 0.5f * (k_nudge + k_soft)) idx = want > 0.0f ? IDX_SOFT_R : IDX_SOFT_L;
            else if (mag >= 0.5f * k_nudge) idx = want > 0.0f ? IDX_NUDGE_R : IDX_NUDGE_L;
            /* Si esa accion no tiene por donde, se baja a la siguiente mas suave. */
            while (idx != IDX_FORWARD &&
                   !(first_viable[idx] && per_first[idx].cost <= best.cost + g->track_slack)) {
                if (idx == IDX_HARD_R) idx = IDX_SOFT_R;
                else if (idx == IDX_HARD_L) idx = IDX_SOFT_L;
                else if (idx == IDX_SOFT_R) idx = IDX_NUDGE_R;
                else if (idx == IDX_SOFT_L) idx = IDX_NUDGE_L;
                else idx = IDX_FORWARD;
            }
            if (idx != IDX_FORWARD || first_viable[IDX_FORWARD]) {
                choice = candidates[idx];
                choice_index = first_viable[idx] ? idx : -1;
                const float cap = idx == IDX_FORWARD ? 0.0f
                    : fminf(k_hard, 1.7f * fabsf(action_curvature(candidates[idx])));
                cont_k = fmaxf(-cap, fminf(cap, k_want));
                have_cont = true;
            }
        } else if (first_viable[IDX_FORWARD] && per_first[IDX_FORWARD].cost <= best.cost + g->track_slack) {
            choice = LOCAL_ACTION_FORWARD;
            choice_index = IDX_FORWARD;
        }
    }

    /* La ruta se muestrea cada 5 cm pero el chasis avanza de 1 en 1: hay que
     * verificar el tramo inmediato o el carrito se cuela en el margen entre dos
     * muestras y luego no le vale ninguna ruta. */
    if (!arc_clear(action_curvature(choice), step_guard_cm())) {
        const uint8_t previous = state->last_action;
        if (previous >= LOCAL_ACTION_FORWARD && previous <= LOCAL_ACTION_LEFT_HARD &&
            arc_clear(action_curvature(previous), step_guard_cm()))
            return previous;
        return LOCAL_ACTION_STOP;
    }
    lattice_curvature = action_curvature(choice);
    if (have_cont && arc_clear(cont_k, step_guard_cm())) lattice_curvature = cont_k;
    best_route = choice_index >= 0 ? per_first[choice_index] : best;
    *chosen_seq = best_route.seq;
    return choice;
}

void local_planner_state_reset(local_planner_state_t *state)
{
    if (!state) return;
    /* Tambien suelta `debug`: asignalo despues de reiniciar. */
    memset(state, 0, sizeof(*state));
}

/* Maniobra de escape. La reticula mira hacia delante -su sonda llega a 150 cm,
 * y ya le basta para negarse a entrar en un bolsillo-, pero sigue sin poder ver
 * que la salida esta detras: cualquier costo que premie avanzar prefiere meterse
 * recto en un callejon antes que sacar el cuerpo. Por eso el escape es una
 * maniobra de estado y no una ruta. */
static uint8_t escape_step(const local_obstacle_map_t *map, local_planner_state_t *state)
{
    const float step = step_cm();
    if (state->escape_phase == LOCAL_ESCAPE_PHASE_BACK) {
        /* Solo importa si hay sitio, no cuanto: sin recortar, el lado que sale
         * elegido depende de que la rejilla es impar (250 cm a un lado, 245 al
         * otro) y no de la situacion. */
        const float room_right = fminf(lateral_room(map, 1), g->escape_room_cm);
        const float room_left = fminf(lateral_room(map, -1), g->escape_room_cm);
        /* Hay sitios donde el hueco lateral no llega nunca: pasado el tope se
         * deja de buscar y se intenta el giro por el lado con mas hueco. */
        if (fmaxf(room_right, room_left) >= g->escape_room_cm ||
            state->escape_back_cm >= g->escape_back_max_cm) {
            state->escape_dir = (room_right >= room_left) ? 1u : 2u;
            state->escape_phase = LOCAL_ESCAPE_PHASE_MARGIN;
            state->escape_back_left_cm = g->escape_margin_cm;
        }
        if (rear_clear()) {
            state->escape_back_cm += step;
            return LOCAL_ACTION_REVERSE;
        }
        return LOCAL_ACTION_STOP;
    }
    if (state->escape_phase == LOCAL_ESCAPE_PHASE_MARGIN) {
        if (state->escape_back_left_cm > 0.0f && rear_clear()) {
            state->escape_back_left_cm -= step;
            return LOCAL_ACTION_REVERSE;
        }
        state->escape_phase = LOCAL_ESCAPE_PHASE_TURN;
        state->escape_use_imu = map->imu_fresh;
        state->escape_turn_start_deg = map->yaw_deg;
        state->escape_turned_deg = 0.0f;
    }

    /* Giro comprometido: se completa antes de volver a planear, si no el
     * planificador local reanudaria recto al primer centimetro. */
    const uint8_t turn = (state->escape_dir == 1u)
        ? LOCAL_ACTION_RIGHT_HARD : LOCAL_ACTION_LEFT_HARD;
    /* Progreso del giro: con IMU fresco se mide el rumbo; sin el se acumula el
     * avance por la curvatura comprometida (velocidad x tiempo, no hay
     * odometria). */
    float progress;
    if (state->escape_use_imu) {
        progress = fabsf(yaw_delta_deg(map->yaw_deg, state->escape_turn_start_deg));
    } else {
        state->escape_turned_deg += step * k_hard / LOCAL_DEG_TO_RAD;
        progress = state->escape_turned_deg;
    }
    /* Cumplido el giro la maniobra acaba aqui. */
    if (progress >= g->escape_turn_deg) {
        state->escaping = false;
        state->escape_phase = LOCAL_ESCAPE_PHASE_NONE;
        state->escape_dir = 0u;
        return LOCAL_ESCAPE_DONE;
    }
    const float remaining_deg = g->escape_turn_deg - progress;
    const float remaining_cm = remaining_deg * LOCAL_DEG_TO_RAD / k_hard
        + g->escape_probe_pad_cm;
    if (arc_clear(action_curvature(turn), remaining_cm)) return turn;
    /* El giro comprometido no cabe: se acaba la maniobra y se replanea en este
     * mismo ciclo. Parar aqui dejaria al robot clavado aunque el escaneo, ya
     * desde la posicion nueva, si tenga salida. */
    state->escaping = false;
    state->escape_phase = LOCAL_ESCAPE_PHASE_NONE;
    state->escape_dir = 0u;
    return LOCAL_ESCAPE_DONE;
}

uint8_t local_obstacle_map_recommend_cfg(const local_obstacle_map_t *map,
                                         const local_planner_config_t *cfg,
                                         local_planner_state_t *state,
                                         uint8_t camera_action,
                                         float course_heading_deg)
{
    use_config(cfg);
    local_planner_debug_t *debug = state ? state->debug : NULL;
    if (debug) {
        memset(debug, 0, sizeof(*debug));
        debug->pass_cm = -1.0f;
    }
    if (!map || !state || map->lidar_points == 0 ||
        camera_action > (uint8_t)LOCAL_ACTION_RETURN_RIGHT)
        return (uint8_t)LOCAL_ACTION_STOP;

    /* Con un escaneo LiDAR util, una tabla de camara ausente/vieja no debe
     * bloquear la navegacion autonoma. Tampoco se ejecutan acciones semanticas
     * de parqueo/recentrado que hayan quedado obsoletas. */
    if (!map->camera_fresh && camera_action >= (uint8_t)LOCAL_ACTION_PARK_LEFT)
        camera_action = (uint8_t)LOCAL_ACTION_FORWARD;
    if (camera_action == (uint8_t)LOCAL_ACTION_PARK_READY)
        return (uint8_t)LOCAL_ACTION_STOP;

    /* Parqueo y contravolante: son guiados de precision con una direccion ya
     * elegida por la camara, no una eleccion de ruta, asi que conservan su
     * comprobacion de espacio. Las maniobras de parqueo bloqueadas paran. */
    if (camera_action == (uint8_t)LOCAL_ACTION_PARK_APPROACH)
        return corridor_clearance(map, 0.0f) >= 40.0f
            ? camera_action : (uint8_t)LOCAL_ACTION_STOP;
    if (camera_action == (uint8_t)LOCAL_ACTION_PARK_LEFT)
        return corridor_clearance(map, -35.0f) >= 30.0f
            ? camera_action : (uint8_t)LOCAL_ACTION_STOP;
    if (camera_action == (uint8_t)LOCAL_ACTION_PARK_RIGHT)
        return corridor_clearance(map, 35.0f) >= 30.0f
            ? camera_action : (uint8_t)LOCAL_ACTION_STOP;
    if (camera_action == (uint8_t)LOCAL_ACTION_RETURN_LEFT) {
        if (corridor_clearance(map, -35.0f) >= 30.0f) return camera_action;
        if (corridor_clearance(map, 35.0f) >= 30.0f)
            return (uint8_t)LOCAL_ACTION_RIGHT_SOFT;
        return (uint8_t)LOCAL_ACTION_STOP;
    }
    if (camera_action == (uint8_t)LOCAL_ACTION_RETURN_RIGHT) {
        if (corridor_clearance(map, 35.0f) >= 30.0f) return camera_action;
        if (corridor_clearance(map, -35.0f) >= 30.0f)
            return (uint8_t)LOCAL_ACTION_LEFT_SOFT;
        return (uint8_t)LOCAL_ACTION_STOP;
    }

    build_distance_field(map);

    /* Referencia de rumbo: si el controlador conoce el rumbo del recorrido y el
     * IMU esta fresco, la referencia es esa (el error de rumbo se ve desde el
     * chasis porque la rejilla esta centrada en el). Sin rumbo de recorrido la
     * referencia es cero, o sea mantener el rumbo actual. */
    float lane_cm = corridor_center_cm(map);
    if (lane_measured) lane_cm += state->lane_offset_cm;
    float reference_rad = (isfinite(course_heading_deg) && map->imu_fresh)
        ? yaw_delta_deg(course_heading_deg, map->yaw_deg) * LOCAL_DEG_TO_RAD
        : 0.0f;
    /* Una esquina de 90 grados con el morro 3 grados atrasado da un error de 93: el corredor y el reticulo tratan >90 como "detras",
     * invierten el giro y el coche daba 270 grados a la izquierda hacia la pared (atasco de 120 s: cw 5 vuelta 3). Entre 90 y 135 grados
     * se sigue pidiendo el giro hacia el lado mas corto, con 85 grados. Mas de 135 (media vuelta) se deja como estaba. */
    {
        const float lim = 85.0f * LOCAL_DEG_TO_RAD, lo = 90.0f * LOCAL_DEG_TO_RAD, hi = 135.0f * LOCAL_DEG_TO_RAD;
        if (fabsf(reference_rad) > lo && fabsf(reference_rad) < hi) reference_rad = reference_rad > 0.0f ? lim : -lim;
    }
    const size_t curtains = apply_curtains(map, reference_rad, lane_cm, lane_measured);
    plan_ref_v = state->ref_rows > 0 ? state->ref_v_cm : NULL;
    plan_ref_rows = state->ref_rows < LOCAL_REF_ROWS ? state->ref_rows : LOCAL_REF_ROWS;
    if (isfinite(course_heading_deg) && map->imu_fresh) plan_corridor(reference_rad, lane_cm);
    else { memset(&corridor, 0, sizeof(corridor)); path_n = 0; }
    if (debug) {
        debug->lane_cm = lane_cm;
        debug->reference_deg = reference_rad / LOCAL_DEG_TO_RAD;
        debug->curtain_cells = curtains;
        debug->plan_count = 0;
        if (corridor.ok)
            for (int i = 0; i < corridor.rows && debug->plan_count < LOCAL_DEBUG_ROUTE_POINTS; ++i) {
                debug->plan[debug->plan_count][0] = corridor.x[i];
                debug->plan[debug->plan_count][1] = corridor.f[i];
                ++debug->plan_count;
            }
    }

    uint8_t action = 0u;
    bool planned = false, planned_by_lattice = false;
    const uint8_t *chosen = NULL;
    /* El escape solo compromete el giro, no la marcha atras. Reevalua las
     * trayectorias delanteras con cada escaneo: si aparece un hueco alcanzable
     * mientras retrocede para buscar sitio, cancela la recuperacion y sigue
     * avanzando por ese hueco. */
    float reversed_cm = state->escape_back_cm;
    if (state->escape_phase == LOCAL_ESCAPE_PHASE_MARGIN)
        reversed_cm += g->escape_margin_cm - state->escape_back_left_cm;
    if (state->escaping && state->escape_phase != LOCAL_ESCAPE_PHASE_TURN &&
        reversed_cm >= g->escape_min_back_cm) {
        const uint8_t forward = lattice_action(state, camera_action, reference_rad, lane_cm,
                                               isfinite(course_heading_deg), &chosen);
        if (forward != (uint8_t)LOCAL_ACTION_STOP) {
            state->escaping = false;
            state->escape_phase = LOCAL_ESCAPE_PHASE_NONE;
            state->escape_dir = 0u;
            state->escape_back_cm = 0.0f;
            state->escape_back_left_cm = 0.0f;
            state->escapes = 0u;
            action = forward;
            planned = true;
            planned_by_lattice = true;
        }
    }
    if (state->escaping) {
        const uint8_t step = escape_step(map, state);
        /* LOCAL_ESCAPE_DONE significa "giro cumplido": no es una accion, hay
         * que volver a planear en este mismo ciclo. */
        if (step != LOCAL_ESCAPE_DONE) {
            action = step;
            planned = true;
            chosen = NULL;
        }
    }
    if (!planned) {
        action = lattice_action(state, camera_action, reference_rad, lane_cm,
                                isfinite(course_heading_deg), &chosen);
        /* Histeresis: un instante antes habia ruta y ahora no por unas decimas de centimetro (la cuantizacion de
         * celdas y el ruido del LiDAR cuando el robot ya pasa rozando un pilar). Se reintenta con la huella un poco
         * mas estrecha antes de dar marcha atras. */
        if (action == (uint8_t)LOCAL_ACTION_STOP && g->squeeze_cm > 0.0f &&
            state->last_action != (uint8_t)LOCAL_ACTION_STOP && state->last_action != (uint8_t)LOCAL_ACTION_REVERSE) {
            static local_planner_config_t squeezed;
            const local_planner_config_t *keep = g;
            squeezed = *g;
            squeezed.footprint_shrink_cm += g->squeeze_cm;
            g = &squeezed;
            const uint8_t again = lattice_action(state, camera_action, reference_rad, lane_cm,
                                                 isfinite(course_heading_deg), &chosen);
            g = keep;
            if (again != (uint8_t)LOCAL_ACTION_STOP) action = again;
        }
        if (action == (uint8_t)LOCAL_ACTION_STOP) {
            chosen = NULL;
            if (state->escapes >= g->escape_max_attempts) {
                if (++state->stop_cycles * g->cycle_s >= g->escape_retry_s) {
                    state->stop_cycles = 0;
                    state->escapes = 0u;
                }
                state->last_action = (uint8_t)LOCAL_ACTION_STOP;
                if (debug) fill_debug_route(debug, NULL);
                return (uint8_t)LOCAL_ACTION_STOP;
            }
            ++state->escapes;
            state->escaping = true;
            state->escape_phase = LOCAL_ESCAPE_PHASE_BACK;
            state->escape_dir = 0u;
            state->escape_back_left_cm = 0.0f;
            state->escape_back_cm = 0.0f;
            action = rear_clear() ? (uint8_t)LOCAL_ACTION_REVERSE
                                  : (uint8_t)LOCAL_ACTION_STOP;
        } else {
            state->escapes = 0u;
            state->stop_cycles = 0u;
            planned_by_lattice = true;
        }
    }
    state->last_action = action;
    if (planned_by_lattice) state->steer_deg = atanf(lattice_curvature * g->wheelbase_cm) / LOCAL_DEG_TO_RAD;
    else state->steer_deg = atanf(action_curvature(action) * g->wheelbase_cm) / LOCAL_DEG_TO_RAD;
    if (debug) {
        if (chosen) memcpy(debug->seq, chosen, LOCAL_LATTICE_DEPTH);
        fill_debug_route(debug, chosen);
    }
    return action;
}

uint8_t local_obstacle_map_recommend(const local_obstacle_map_t *map,
                                     local_planner_state_t *state,
                                     uint8_t camera_action,
                                     float course_heading_deg)
{
    return local_obstacle_map_recommend_cfg(map, NULL, state, camera_action, course_heading_deg);
}

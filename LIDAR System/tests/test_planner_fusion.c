/* Pruebas de lo que se corrigio y se hizo configurable en el planificador:
 * mediana del carril, frescura asincrona, camara anclada al LiDAR (sin
 * obstaculos fantasma), regla de color como DATO, union de retornos, correccion
 * del giro durante el barrido y zona ciega. */
#include "local_obstacle_map.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Geometria con la que se escribieron estas escenas: chasis de 8 x 12 cm con direccion tipo coche
 * (batalla 12 cm), LiDAR 12 cm y camara 6 cm por delante del eje. Los valores por defecto del
 * firmware son ahora los del robot real (17 x 13 cm, diferencial). */
static void legacy_config(local_planner_config_t *c)
{
    local_planner_config_default(c);
    c->chassis_width_cm = 8.0f; c->chassis_length_cm = 12.0f; c->chassis_rear_cm = 0.0f;
    c->footprint_shrink_cm = 0.0f; c->wheelbase_cm = 12.0f;
    c->steer_soft_deg = 12.0f; c->steer_hard_deg = 30.0f;
    c->lidar_forward_cm = 12.0f; c->camera_forward_cm = 6.0f;
}
static void use_test_chassis(void)
{
    local_planner_config_t c;
    legacy_config(&c);
    local_planner_set_default_config(&c);
}

#include <string.h>

#define RAD_TO_DEG 57.29577951308232f

/* Retorno del LiDAR (montado a lidar_forward del eje) para un punto del plano
 * dado respecto al eje trasero. */
static local_lidar_point_t at(float x_cm, float forward_cm)
{
    const float f = forward_cm - 12.0f;
    return (local_lidar_point_t){.angle_deg = atan2f(x_cm, f) * RAD_TO_DEG,
                                 .distance_cm = hypotf(x_cm, f), .valid = true};
}

static bool is_right(uint8_t a) { return a == LOCAL_ACTION_RIGHT_SOFT || a == LOCAL_ACTION_RIGHT_HARD || a == LOCAL_ACTION_NUDGE_RIGHT; }
static bool is_left(uint8_t a) { return a == LOCAL_ACTION_LEFT_SOFT || a == LOCAL_ACTION_LEFT_HARD || a == LOCAL_ACTION_NUDGE_LEFT; }

/* Pasillo recto de 100 cm con el robot centrado: paredes cada 5 cm. */
static size_t lane_points(local_lidar_point_t *out, size_t start, float left_cm, float right_cm)
{
    size_t n = start;
    for (float f = -10.0f; f <= 140.0f; f += 4.0f) {
        out[n++] = at(-left_cm, f);
        out[n++] = at(right_cm, f);
    }
    return n;
}

static void test_median_rejects_outlier_row(void)
{
    local_lidar_point_t points[200];
    local_obstacle_map_t map;
    local_planner_state_t state;
    local_planner_config_t cfg;
    legacy_config(&cfg);
    size_t n = 0;
    /* Carril de 100 cm centrado, excepto UNA fila (a 6 cm del eje) cuyas paredes
     * parecen desplazadas 25 cm: una celda suelta. Las filas se leen a 2, 6 y
     * 10 cm; la del medio es la atipica. */
    for (float f = -10.0f; f <= 140.0f; f += 4.0f) {
        const bool odd_row = fabsf(f - 6.0f) < 3.0f;
        points[n++] = at(odd_row ? -25.0f : -50.0f, f);
        points[n++] = at(odd_row ? 75.0f : 50.0f, f);
    }
    local_lidar_point_t front = at(0.0f, 150.0f);
    points[n++] = front;
    local_obstacle_map_update_cfg(&map, &cfg, points, n, 1000000, NULL, 0, 0, NULL);
    local_planner_state_reset(&state);
    local_planner_debug_t debug;
    state.debug = &debug;
    const uint8_t action = local_obstacle_map_recommend_cfg(&map, &cfg, &state, LOCAL_ACTION_FORWARD, NAN);
    /* Dos filas de tres no coinciden: no hay carril fiable y no se corrige. */
    assert(fabsf(debug.lane_cm) < 0.01f);
    assert(action == LOCAL_ACTION_FORWARD);
}

static void test_async_sensors_count_as_fresh(void)
{
    local_lidar_point_t points[200];
    local_obstacle_map_t map;
    const size_t n = lane_points(points, 0, 50.0f, 50.0f);
    const local_camera_object_t cam = {.bearing_deg = 0.0f, .distance_cm = 60.0f, .color = 1, .valid = true};
    /* La camara y la IMU llegan DESPUES del escaneo, como en el P4 real. */
    const local_imu_sample_t imu = {.valid = true, .yaw_deg = 12.0f, .timestamp_us = 1020000};
    local_obstacle_map_update(&map, points, n, 1000000, &cam, 1, 1030000, &imu);
    assert(map.camera_fresh && map.imu_fresh && map.yaw_deg == 12.0f);
    /* Y siguen caducando cuando se separan demasiado, en cualquier sentido. */
    const local_imu_sample_t late = {.valid = true, .yaw_deg = 1.0f, .timestamp_us = 1400000};
    local_obstacle_map_update(&map, points, n, 1000000, &cam, 1, 1400000, &late);
    assert(!map.camera_fresh && !map.imu_fresh);
}

static void test_camera_is_anchored_to_lidar(void)
{
    local_lidar_point_t points[200];
    local_obstacle_map_t map;
    local_planner_config_t cfg;
    legacy_config(&cfg);
    size_t n = lane_points(points, 0, 50.0f, 50.0f);
    /* Pilar real a 40 cm: tres retornos de su cara frontal. */
    points[n++] = at(-2.0f, 40.0f);
    points[n++] = at(0.0f, 40.0f);
    points[n++] = at(2.0f, 40.0f);
    /* La camara lo ve a 25 cm del lente en vez de 34: error tipico del ancho. */
    local_camera_object_t cam = {.bearing_deg = 0.0f, .distance_cm = 25.0f, .color = 1, .valid = true};
    local_obstacle_map_update_cfg(&map, &cfg, points, n, 1000000, &cam, 1, 1000000, NULL);
    assert(map.camera_hits == 1 && map.camera_unmatched == 0 && map.colored_count == 1);
    assert(map.colored[0].confirmed && map.colored[0].color == 1);
    /* La posicion es la del LiDAR (40 cm), no la estimada por la camara (31). */
    assert(fabsf(map.colored[0].forward_cm - 40.0f) < 1.0f);
    uint8_t cell;
    int ghosts = 0;
    for (int fc = 10; fc <= 14; ++fc)   /* la celda donde la camara cree que esta */
        if (local_obstacle_map_get(&map, 0, fc, &cell) && (cell & LOCAL_CELL_OCCUPIED)) ++ghosts;
    assert(ghosts == 0);

    /* Una deteccion sin retorno que la respalde no inventa obstaculos... */
    n = lane_points(points, 0, 50.0f, 50.0f);
    local_obstacle_map_update_cfg(&map, &cfg, points, n, 1000000, &cam, 1, 1000000, NULL);
    assert(map.camera_hits == 0 && map.camera_unmatched == 1 && !map.colored[0].confirmed);
    int occupied = 0;
    for (int fc = 8; fc <= 16; ++fc)
        if (local_obstacle_map_get(&map, 0, fc, &cell) && (cell & LOCAL_CELL_OCCUPIED)) ++occupied;
    assert(occupied == 0);
    /* ...salvo que el equipo lo pida (LiDAR a una altura que no ve el pilar). */
    cfg.camera_ghost_fallback = true;
    local_obstacle_map_update_cfg(&map, &cfg, points, n, 1000000, &cam, 1, 1000000, NULL);
    assert(map.camera_hits == 1);
    occupied = 0;
    for (int fc = 10; fc <= 14; ++fc)
        if (local_obstacle_map_get(&map, 0, fc, &cell) && (cell & LOCAL_CELL_OCCUPIED)) ++occupied;
    assert(occupied == 1);
}

/* Pilar de color a 40 cm en el centro de un carril de 100 cm. Devuelve la accion. */
static uint8_t plan_pillar_at(const local_planner_config_t *cfg, uint8_t color, float x_cm, float ahead_cm)
{
    local_lidar_point_t points[200];
    local_obstacle_map_t map;
    local_planner_state_t state;
    size_t n = lane_points(points, 0, 50.0f, 50.0f);
    points[n++] = at(x_cm - 2.5f, ahead_cm);
    points[n++] = at(x_cm - 1.0f, ahead_cm);
    points[n++] = at(x_cm + 1.0f, ahead_cm);
    points[n++] = at(x_cm + 2.5f, ahead_cm);
    const float bearing = atan2f(x_cm, ahead_cm - 6.0f) * RAD_TO_DEG;
    const local_camera_object_t cam = {.bearing_deg = bearing,
        .distance_cm = hypotf(x_cm, ahead_cm - 6.0f), .color = color, .valid = true};
    local_obstacle_map_update_cfg(&map, cfg, points, n, 1000000, &cam, 1, 1000000, NULL);
    local_planner_state_reset(&state);
    return local_obstacle_map_recommend_cfg(&map, cfg, &state, LOCAL_ACTION_FORWARD, NAN);
}

/* A 40 cm el pilar aun puede esperar: la retícula arranca recto y gira despues. */
static uint8_t plan_pillar(const local_planner_config_t *cfg, uint8_t color, float x_cm)
{
    return plan_pillar_at(cfg, color, x_cm, 28.0f);
}

static void test_color_rule_is_configuration(void)
{
    local_planner_config_t cfg;
    legacy_config(&cfg);
    /* WRO: rojo por la derecha, verde por la izquierda, aunque el pilar este
     * justo en el centro y la geometria sea simetrica. */
    assert(is_right(plan_pillar(&cfg, 1, 0.0f)));
    assert(is_left(plan_pillar(&cfg, 2, 0.0f)));
    /* Con el pilar desplazado hacia el lado "equivocado", sigue mandando el color:
     * un verde a la derecha del centro se rebasa por su izquierda, entre el
     * pilar y el centro (recto cabe), y nunca girando hacia su cortina. */
    assert(!is_right(plan_pillar(&cfg, 2, 12.0f)));
    assert(!is_left(plan_pillar(&cfg, 1, -12.0f)));
    /* Mas pegado al centro ya no cabe recto: hay que ceder al lado correcto. */
    assert(is_left(plan_pillar(&cfg, 2, 6.0f)));
    assert(is_right(plan_pillar(&cfg, 1, -6.0f)));
    /* Otra sede o una regla sorpresa invierte los colores: se cambia el dato. */
    cfg.pass_side_red = -1;
    cfg.pass_side_green = 1;
    assert(is_left(plan_pillar(&cfg, 1, 0.0f)));
    assert(is_right(plan_pillar(&cfg, 2, 0.0f)));
    /* Sin regla (o con el color ignorado) manda solo la geometria. */
    cfg.enforce_color_side = false;
    assert(!is_right(plan_pillar(&cfg, 1, 12.0f)));   /* el lado con mas sitio */
    cfg.enforce_color_side = true;
    cfg.pass_side_red = 0;
    assert(!is_right(plan_pillar(&cfg, 1, 12.0f)));
}

static void test_bridging_closes_wall_gaps(void)
{
    local_obstacle_map_t map;
    local_planner_config_t cfg;
    legacy_config(&cfg);
    /* Dos retornos de una pared a 200 cm, separados 10 cm: sin union quedan
     * celdas libres entre ellos, que el planificador tomaria por una salida. */
    const local_lidar_point_t wall[] = {at(-5.0f, 150.0f), at(5.0f, 150.0f)};
    uint8_t cell;
    local_obstacle_map_update_cfg(&map, &cfg, wall, 2, 1000000, NULL, 0, 0, NULL);
    assert(local_obstacle_map_get(&map, 0, 60, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    cfg.bridge_max_gap_cm = 0.0f;
    local_obstacle_map_update_cfg(&map, &cfg, wall, 2, 1000000, NULL, 0, 0, NULL);
    assert(local_obstacle_map_get(&map, 0, 60, &cell) && !(cell & LOCAL_CELL_OCCUPIED));
    /* Un pilar delante de una pared lejana NO se une a ella. */
    cfg.bridge_max_gap_cm = 12.0f;
    const local_lidar_point_t jump[] = {at(0.0f, 60.0f), at(6.0f, 150.0f)};
    local_obstacle_map_update_cfg(&map, &cfg, jump, 2, 1000000, NULL, 0, 0, NULL);
    assert(local_obstacle_map_get(&map, 2, 40, &cell) && !(cell & LOCAL_CELL_OCCUPIED));
}

static void test_deskew_and_blind_zone(void)
{
    local_obstacle_map_t map;
    uint8_t cell;
    local_lidar_point_t scan[] = {
        {.angle_deg = 0.0f, .distance_cm = 100.0f, .valid = true},
        {.angle_deg = 90.0f, .distance_cm = 50.0f, .valid = true}};
    local_imu_sample_t imu = {.valid = true, .yaw_deg = 0.0f, .timestamp_us = 1000000,
                              .has_rate = true, .yaw_rate_dps = 60.0f};
    /* El primer punto se midio 100 ms antes del final del barrido, con el chasis
     * girando a la derecha a 60 grados por segundo: ahora cae 6 grados a la
     * izquierda de donde se leyo. */
    local_obstacle_map_update(&map, scan, 2, 1000000, NULL, 0, 0, &imu);
    assert(local_obstacle_map_get(&map, -4, 45, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    assert(local_obstacle_map_get(&map, 0, 45, &cell) && !(cell & LOCAL_CELL_OCCUPIED));
    local_planner_config_t cfg;
    legacy_config(&cfg);
    cfg.deskew = false;
    local_obstacle_map_update_cfg(&map, &cfg, scan, 2, 1000000, NULL, 0, 0, &imu);
    assert(local_obstacle_map_get(&map, 0, 45, &cell) && (cell & LOCAL_CELL_OCCUPIED));

    /* Un rayo en la zona ciega es un obstaculo pegado al morro, no ausencia. */
    const local_lidar_point_t blind[] = {{.angle_deg = 0.0f, .distance_cm = 0.0f, .valid = false, .blind = true}};
    local_obstacle_map_update(&map, blind, 1, 1000000, NULL, 0, 0, NULL);
    assert(local_obstacle_map_get(&map, 0, 7, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    const local_lidar_point_t ignored[] = {{.angle_deg = 0.0f, .distance_cm = 0.0f, .valid = false}};
    local_obstacle_map_update(&map, ignored, 1, 1000000, NULL, 0, 0, NULL);
    assert(local_obstacle_map_get(&map, 0, 7, &cell) && !(cell & LOCAL_CELL_OCCUPIED));
}

static void test_configuration(void)
{
    local_planner_config_t cfg;
    legacy_config(&cfg);
    assert(!local_planner_config_sanitize(&cfg));
    assert(cfg.speed_cm_s >= 15.0f); /* una vuelta de 8 m x 3 en 180 s */
    cfg.speed_cm_s = -4.0f; cfg.chassis_width_cm = NAN; cfg.pass_side_red = 7;
    assert(local_planner_config_sanitize(&cfg));
    assert(cfg.speed_cm_s == 3.0f && cfg.chassis_width_cm == 2.0f && cfg.pass_side_red == 0);

    /* Un chasis mas ancho necesita un hueco mayor: la geometria es dato. */
    local_lidar_point_t points[8];
    local_planner_state_t state;
    local_obstacle_map_t map;
    legacy_config(&cfg);
    size_t n = 0;
    points[n++] = at(-12.0f, 40.0f);
    points[n++] = at(12.0f, 40.0f);
    local_obstacle_map_update_cfg(&map, &cfg, points, n, 1000000, NULL, 0, 0, NULL);
    local_planner_state_reset(&state);
    assert(local_obstacle_map_recommend_cfg(&map, &cfg, &state, LOCAL_ACTION_FORWARD, NAN) !=
           LOCAL_ACTION_STOP);
    cfg.chassis_width_cm = 26.0f; /* no cabe por un hueco de 24 cm */
    local_obstacle_map_update_cfg(&map, &cfg, points, n, 1000000, NULL, 0, 0, NULL);
    local_planner_state_reset(&state);
    const uint8_t action = local_obstacle_map_recommend_cfg(&map, &cfg, &state, LOCAL_ACTION_FORWARD, NAN);
    assert(action != LOCAL_ACTION_FORWARD);
}

int main(void)
{
    use_test_chassis();
    test_median_rejects_outlier_row();
    test_async_sensors_count_as_fresh();
    test_camera_is_anchored_to_lidar();
    test_color_rule_is_configuration();
    test_bridging_closes_wall_gaps();
    test_deskew_and_blind_zone();
    test_configuration();
    puts("PASS: mediana del carril, sensores asincronos, camara anclada al LiDAR, regla de color, union de retornos, deskew y zona ciega");
    return 0;
}

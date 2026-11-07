/* Pruebas del mapa local y del planificador de rutas.
 *
 * Las capas del mapa se comprueban con valores exactos. El planificador se
 * comprueba con PROPIEDADES, no con acciones exactas: dos rutas simetricas ante
 * un obstaculo centrado son ambas correctas, asi que fijar cual de las dos sale
 * convierte un detalle de desempate en un requisito falso. Lo que si es
 * requisito es que nunca gire hacia el obstaculo, que no pare si hay hueco y
 * que use la reversa cuando no queda salida adelante.
 */
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


static const char *name_of(uint8_t action)
{
    switch (action) {
    case LOCAL_ACTION_STOP: return "STOP";
    case LOCAL_ACTION_FORWARD: return "FORWARD";
    case LOCAL_ACTION_RIGHT_SOFT: return "RIGHT_SOFT";
    case LOCAL_ACTION_RIGHT_HARD: return "RIGHT_HARD";
    case LOCAL_ACTION_LEFT_SOFT: return "LEFT_SOFT";
    case LOCAL_ACTION_LEFT_HARD: return "LEFT_HARD";
    case LOCAL_ACTION_PARK_LEFT: return "PARK_LEFT";
    case LOCAL_ACTION_PARK_RIGHT: return "PARK_RIGHT";
    case LOCAL_ACTION_PARK_APPROACH: return "PARK_APPROACH";
    case LOCAL_ACTION_PARK_READY: return "PARK_READY";
    case LOCAL_ACTION_RETURN_LEFT: return "RETURN_LEFT";
    case LOCAL_ACTION_RETURN_RIGHT: return "RETURN_RIGHT";
    case LOCAL_ACTION_REVERSE: return "REVERSE";
    case LOCAL_ACTION_NUDGE_RIGHT: return "NUDGE_RIGHT";
    case LOCAL_ACTION_NUDGE_LEFT: return "NUDGE_LEFT";
    default: return "?";
    }
}

static bool is_turn(uint8_t action)
{
    return action == LOCAL_ACTION_RIGHT_SOFT || action == LOCAL_ACTION_RIGHT_HARD || action == LOCAL_ACTION_NUDGE_RIGHT || action == LOCAL_ACTION_NUDGE_LEFT ||
        action == LOCAL_ACTION_LEFT_SOFT || action == LOCAL_ACTION_LEFT_HARD;
}

static bool is_left_turn(uint8_t action)
{
    return action == LOCAL_ACTION_LEFT_SOFT || action == LOCAL_ACTION_LEFT_HARD || action == LOCAL_ACTION_NUDGE_LEFT;
}

static bool is_right_turn(uint8_t action)
{
    return action == LOCAL_ACTION_RIGHT_SOFT || action == LOCAL_ACTION_RIGHT_HARD || action == LOCAL_ACTION_NUDGE_RIGHT;
}

static void fail(const char *label, uint8_t action)
{
    printf("FALLO: %s devolvio %s\n", label, name_of(action));
}

/* Escenario con la camara fresca, igual que en la pista. */
static const local_camera_object_t k_objects[] = {
    {.bearing_deg = 0, .distance_cm = 106, .color = 1, .valid = true}
};

/* Callejon sin salida: ninguna ruta de avance cabe en el compromiso minimo. */
static const local_lidar_point_t k_trap[] = {
    {.angle_deg = 0, .distance_cm = 12, .valid = true},
    {.angle_deg = -20, .distance_cm = 13, .valid = true},
    {.angle_deg = 20, .distance_cm = 13, .valid = true},
    {.angle_deg = -45, .distance_cm = 15, .valid = true},
    {.angle_deg = 45, .distance_cm = 15, .valid = true}
};

static void plan(uint8_t *out, const local_lidar_point_t *points, size_t count,
                 uint8_t camera_action)
{
    local_obstacle_map_t map;
    local_planner_state_t state;
    local_obstacle_map_update(&map, points, count, 1000000, k_objects, 1, 990000, NULL);
    local_planner_state_reset(&state);
    *out = local_obstacle_map_recommend(&map, &state, camera_action, NAN);
}

/* Igual que plan() pero con rumbo del recorrido y IMU fresco: es la referencia
 * de alineacion que usa el controlador en la pista. */
static void plan_course(uint8_t *out, const local_lidar_point_t *points, size_t count,
                        uint8_t camera_action, float course_deg, float yaw_deg)
{
    local_obstacle_map_t map;
    local_planner_state_t state;
    const local_imu_sample_t imu = {.valid = true, .yaw_deg = yaw_deg, .timestamp_us = 990000};
    local_obstacle_map_update(&map, points, count, 1000000, k_objects, 1, 990000, &imu);
    local_planner_state_reset(&state);
    *out = local_obstacle_map_recommend(&map, &state, camera_action, course_deg);
}

static void test_map_layers(void)
{
    local_obstacle_map_t map;
    const local_lidar_point_t points[] = {
        {.angle_deg = 0, .distance_cm = 100, .valid = true},
        {.angle_deg = 90, .distance_cm = 50, .valid = true},
        {.angle_deg = 180, .distance_cm = 60, .valid = true},
        {.angle_deg = -90, .distance_cm = 50, .valid = true},
        {.angle_deg = 0, .distance_cm = 20, .valid = false}
    };
    const local_camera_object_t objects[] = {
        {.bearing_deg = 0, .distance_cm = 106, .color = 1, .valid = true}
    };
    const local_imu_sample_t imu = {.valid = true, .yaw_deg = 37.5f, .timestamp_us = 990000};
    local_obstacle_map_update(&map, points, 5, 1000000, objects, 1, 960000, &imu);

    uint8_t cell;
    /* Celdas de 2,5 cm: el LiDAR esta 12 cm por delante del eje. */
    assert(local_obstacle_map_get(&map, 0, 45, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    assert(local_obstacle_map_get(&map, 0, 20, &cell) && (cell & LOCAL_CELL_FREE));
    assert(local_obstacle_map_get(&map, 20, 5, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    assert(local_obstacle_map_get(&map, 0, -19, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    assert(local_obstacle_map_get(&map, -20, 5, &cell) && (cell & LOCAL_CELL_OCCUPIED));
    assert(local_obstacle_map_get(&map, 0, 45, &cell) && (cell & LOCAL_CELL_VISION));
    /* Un retorno marca su celda, no un bloque de 3x3: el margen de seguridad lo
     * aplica el planificador al consultar el campo de distancias. */
    assert(local_obstacle_map_get(&map, 0, 47, &cell) && !(cell & LOCAL_CELL_OCCUPIED));
    assert(local_obstacle_map_get(&map, 2, 45, &cell) && !(cell & LOCAL_CELL_OCCUPIED));
    assert(map.lidar_hits == 4 && map.camera_hits == 1);
    assert(map.camera_fresh && map.imu_fresh && map.yaw_deg == 37.5f);

    /* La camara atrasada no se superpone; la IMU tambien caduca. */
    local_obstacle_map_update(&map, points, 5, 1200000, objects, 1, 960000, &imu);
    assert(!map.camera_fresh && !map.imu_fresh);
    assert(map.camera_hits == 0);
    assert(local_obstacle_map_get(&map, 0, 45, &cell) && !(cell & LOCAL_CELL_VISION));
    assert(!local_obstacle_map_get(&map, 81, 0, &cell));

    /* Cada ciclo reemplaza la rejilla. */
    const local_lidar_point_t distant[] = {{.angle_deg = 0, .distance_cm = 1000, .valid = true}};
    local_obstacle_map_update(&map, distant, 1, 2300000, NULL, 0, 0, NULL);
    assert(local_obstacle_map_get(&map, 0, 78, &cell) && (cell & LOCAL_CELL_FREE));
    assert(!(cell & LOCAL_CELL_OCCUPIED));
}

static void test_planner_properties(void)
{
    local_planner_state_t state;
    local_obstacle_map_t map;
    uint8_t action;

    const local_lidar_point_t open[] = {
        {.angle_deg = 0, .distance_cm = 100, .valid = true},
        {.angle_deg = 90, .distance_cm = 50, .valid = true},
        {.angle_deg = 180, .distance_cm = 60, .valid = true},
        {.angle_deg = -90, .distance_cm = 50, .valid = true}
    };
    const local_lidar_point_t centred[] = {{.angle_deg = 0, .distance_cm = 20, .valid = true}};
    const local_lidar_point_t centred_thirty[] = {{.angle_deg = 0, .distance_cm = 30, .valid = true}};
    const local_lidar_point_t bump_left[] = {{.angle_deg = -20, .distance_cm = 18, .valid = true}};
    const local_lidar_point_t bump_right[] = {{.angle_deg = 20, .distance_cm = 18, .valid = true}};
    const local_lidar_point_t gap_twentythree[] = {
        {.angle_deg = -35, .distance_cm = 20, .valid = true},
        {.angle_deg = 35, .distance_cm = 20, .valid = true}
    };
    const local_lidar_point_t block_left[] = {{.angle_deg = -35, .distance_cm = 20, .valid = true}};
    const local_lidar_point_t block_right[] = {{.angle_deg = 35, .distance_cm = 20, .valid = true}};
    /* El fondo de la trampa esta 22 cm por detras de la lente, o sea 10 cm por
     * detras del eje trasero: es lo que hace que la reversa no tenga sitio. Con
     * 8 cm el punto caia DENTRO del chasis (el D6 va 12 cm por delante del eje),
     * y eso lo unico que probaba era el error simetrico de mirar hacia delante
     * desde el morro para decidir si se puede retroceder. */
    const local_lidar_point_t trap_blocked_rear[] = {
        {.angle_deg = 0, .distance_cm = 12, .valid = true},
        {.angle_deg = -20, .distance_cm = 13, .valid = true},
        {.angle_deg = 20, .distance_cm = 13, .valid = true},
        {.angle_deg = -45, .distance_cm = 15, .valid = true},
        {.angle_deg = 45, .distance_cm = 15, .valid = true},
        {.angle_deg = 180, .distance_cm = 22, .valid = true}
    };

    /* Sin mapa no se decide nada. */
    plan(&action, NULL, 0, LOCAL_ACTION_FORWARD);
    assert(action == LOCAL_ACTION_STOP);

    /* Pasillo libre: avanza. */
    plan(&action, open, 4, LOCAL_ACTION_FORWARD);
    assert(action == LOCAL_ACTION_FORWARD);

    /* Bloque centrado: hay que girar, y nunca seguir recto. */
    plan(&action, centred, 1, LOCAL_ACTION_FORWARD);
    if (!is_turn(action)) { fail("bloque centrado a 20 cm", action); assert(false); }
    plan(&action, centred, 1, LOCAL_ACTION_STOP);
    if (!is_turn(action)) { fail("bloque centrado, camara en STOP", action); assert(false); }
    plan(&action, centred_thirty, 1, LOCAL_ACTION_FORWARD);
    if (!is_turn(action)) { fail("bloque centrado a 30 cm", action); assert(false); }

    /* Bulto a un lado: jamas se gira hacia el. */
    plan(&action, bump_left, 1, LOCAL_ACTION_FORWARD);
    if (is_left_turn(action)) { fail("bulto a la izquierda", action); assert(false); }
    plan(&action, bump_right, 1, LOCAL_ACTION_FORWARD);
    if (is_right_turn(action)) { fail("bulto a la derecha", action); assert(false); }

    /* Hueco de 23 cm: cabe, asi que no se para. */
    plan(&action, gap_twentythree, 2, LOCAL_ACTION_FORWARD);
    if (action == LOCAL_ACTION_STOP) { fail("hueco de 23 cm", action); assert(false); }

    /* Trampa cerrada: ninguna ruta sobrevive, empieza la reversa. */
    local_obstacle_map_update(&map, k_trap, 5, 1000000, k_objects, 1, 990000, NULL);
    local_planner_state_reset(&state);
    assert(local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN) ==
           LOCAL_ACTION_REVERSE);
    assert(state.escaping);
    /* La maniobra persiste entre ciclos con el mismo escaneo. */
    assert(local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN) ==
           LOCAL_ACTION_REVERSE);
    /* Reiniciar el estado cancela el escape en curso. */
    local_planner_state_reset(&state);
    assert(!state.escaping && state.last_action == 0u);

    /* Trampa cerrada y ademas con el fondo tapado: ni reversa. */
    plan(&action, trap_blocked_rear, 6, LOCAL_ACTION_FORWARD);
    if (action != LOCAL_ACTION_STOP) { fail("trampa con fondo tapado", action); assert(false); }

    /* Apretado contra un lado: ninguna ruta delantera sobrevive porque la huella
     * pisa la celda de la pared, pero DETRAS no hay nada. La maniobra tiene que
     * retroceder, no rendirse: medido en la pista, con 4,8 cm de holgura real el
     * robot se paraba para siempre porque la comprobacion de la parte de atras
     * miraba hacia delante y veia el bloque que tenia al lado. */
    const local_lidar_point_t squeeze_clear_rear[] = {
        {.angle_deg = 0, .distance_cm = 15, .valid = true},
        {.angle_deg = -25, .distance_cm = 16, .valid = true},
        {.angle_deg = 25, .distance_cm = 16, .valid = true},
        {.angle_deg = -90, .distance_cm = 6, .valid = true},
        {.angle_deg = 90, .distance_cm = 7, .valid = true}
    };
    plan(&action, squeeze_clear_rear, 5, LOCAL_ACTION_FORWARD);
    if (action != LOCAL_ACTION_REVERSE) { fail("apretado sin fondo tapado", action); assert(false); }

    /* Parqueo y contravolante conservan su comprobacion de espacio. */
    plan(&action, open, 4, LOCAL_ACTION_PARK_READY);
    assert(action == LOCAL_ACTION_STOP);
    plan(&action, open, 4, LOCAL_ACTION_PARK_LEFT);
    assert(action == LOCAL_ACTION_PARK_LEFT);
    plan(&action, open, 4, LOCAL_ACTION_PARK_RIGHT);
    assert(action == LOCAL_ACTION_PARK_RIGHT);
    plan(&action, open, 4, LOCAL_ACTION_PARK_APPROACH);
    assert(action == LOCAL_ACTION_PARK_APPROACH);
    plan(&action, open, 4, LOCAL_ACTION_RETURN_LEFT);
    assert(action == LOCAL_ACTION_RETURN_LEFT);
    plan(&action, open, 4, LOCAL_ACTION_RETURN_RIGHT);
    assert(action == LOCAL_ACTION_RETURN_RIGHT);
    /* Bloqueado el hueco del parqueo, la maniobra se convierte en STOP. */
    plan(&action, centred, 1, LOCAL_ACTION_PARK_APPROACH);
    assert(action == LOCAL_ACTION_STOP);
    plan(&action, block_left, 1, LOCAL_ACTION_PARK_LEFT);
    assert(action == LOCAL_ACTION_STOP);
    plan(&action, block_right, 1, LOCAL_ACTION_PARK_RIGHT);
    assert(action == LOCAL_ACTION_STOP);
    /* Una accion de camara fuera de rango se rechaza. */
    plan(&action, open, 4, 200u);
    assert(action == LOCAL_ACTION_STOP);
}

/* Reconstruye el callejon visto desde una posicion desplazada `back_cm` hacia
 * atras: retroceder en recta aleja los obstaculos en la misma direccion, asi
 * que basta con alargar cada distancia. Sin esto el mapa nunca cambiaria y la
 * maniobra se probaria siempre contra el mismo muro. */
static void trap_points(local_lidar_point_t *out, size_t count, float back_cm)
{
    for (size_t i = 0; i < count; ++i) {
        out[i] = k_trap[i];
        out[i].distance_cm += back_cm;
    }
}

/* Referencia lateral y regreso a la linea. Dos fallos medidos en la pista:
 * (1) con un lado abierto -un cruce, o el recorte de un retorno lejano contra el
 * borde del mapa- el centro del carril salia de metros y el coste lateral
 * arrastraba al robot en un arco; (2) con la linea anclada al punto de horizonte,
 * la ruta recta quedaba sobre ella por construccion, asi que un error de rumbo
 * de 28 grados no se corregia: el robot esquivaba y seguia torcido. */
/* Retorno de una pared recta a `x_cm` del eje, a `f_cm` por delante del eje. */
static local_lidar_point_t wall_point(float x_cm, float f_cm)
{
    const float f = f_cm - 12.0f;
    return (local_lidar_point_t){.angle_deg = atan2f(x_cm, f) * 57.29577951f,
                                 .distance_cm = hypotf(x_cm, f), .valid = true};
}

static void test_lane_reference(void)
{
    local_planner_state_t state;
    local_obstacle_map_t map;
    uint8_t action;

    /* Pasillo de 100 cm con el robot centrado: sin correccion lateral. Las
     * paredes son LINEAS de retornos (a lo largo de 30 cm), no un punto: la
     * rejilla de 2,5 cm ya distingue una fila de otra. */
    local_lidar_point_t centred_corridor[40], offset_corridor[40], open_field[40];
    size_t nc = 0, no = 0, nf = 0;
    for (float f = -6.0f; f <= 24.0f; f += 3.0f) {
        centred_corridor[nc++] = wall_point(-50.0f, f); centred_corridor[nc++] = wall_point(50.0f, f);
        /* Paredes a 25 cm a la izquierda y 75 a la derecha: el centro del carril
         * queda 25 cm a la derecha y el robot debe corregir hacia el. */
        offset_corridor[no++] = wall_point(-25.0f, f); offset_corridor[no++] = wall_point(75.0f, f);
        /* Paredes lejanisimas y asimetricas (+200 y -150 cm): no es un carril, es
         * mapa abierto. Sin el limite de anchura la resta daba +25 cm. */
        open_field[nf++] = wall_point(-150.0f, f); open_field[nf++] = wall_point(190.0f, f);
    }
    centred_corridor[nc++] = (local_lidar_point_t){.angle_deg = 0, .distance_cm = 150, .valid = true};
    offset_corridor[no++] = (local_lidar_point_t){.angle_deg = 0, .distance_cm = 150, .valid = true};
    open_field[nf++] = (local_lidar_point_t){.angle_deg = 0, .distance_cm = 150, .valid = true};
    plan(&action, centred_corridor, nc, LOCAL_ACTION_FORWARD);
    assert(action == LOCAL_ACTION_FORWARD);
    plan(&action, offset_corridor, no, LOCAL_ACTION_FORWARD);
    if (!is_right_turn(action)) { fail("carril desplazado a la derecha", action); assert(false); }
    plan(&action, open_field, nf, LOCAL_ACTION_FORWARD);
    if (action != LOCAL_ACTION_FORWARD) { fail("campo abierto sin carril", action); assert(false); }

    /* Rumbo del recorrido 30 grados a la derecha del morro y robot alineado con
     * el pasillo: hay que girar a la derecha para volver al rumbo, no seguir
     * recto. Antes esta decision era FORWARD. */
    plan_course(&action, centred_corridor, nc, LOCAL_ACTION_FORWARD, 30.0f, 0.0f);
    if (!is_right_turn(action)) { fail("vuelta al rumbo del recorrido", action); assert(false); }

    /* Y con el rumbo del recorrido a la izquierda, gira a la izquierda. */
    plan_course(&action, centred_corridor, nc, LOCAL_ACTION_FORWARD, -30.0f, 0.0f);
    if (!is_left_turn(action)) { fail("vuelta al rumbo del recorrido (izquierda)", action); assert(false); }

    /* Alineado con el rumbo del recorrido no hay nada que corregir. */
    plan_course(&action, centred_corridor, nc, LOCAL_ACTION_FORWARD, 0.0f, 0.0f);
    assert(action == LOCAL_ACTION_FORWARD);

    /* El rumbo del recorrido manda: si el robot esta a 28 grados y el carril esta
     * centrado, la correccion no es un giro perpetuo. */
    plan_course(&action, centred_corridor, nc, LOCAL_ACTION_FORWARD, 0.0f, -28.0f);
    if (!is_right_turn(action)) { fail("error de rumbo de 28 grados", action); assert(false); }

    (void)state;
    (void)map;
}

static void test_escape_cycle(void)
{
    /* La reversa de escape no queda comprometida: si el siguiente escaneo abre
     * una ruta delantera, el P4 cancela la recuperacion inmediatamente. */
    local_obstacle_map_t map;
    local_planner_state_t state;
    local_lidar_point_t points[5];
    local_imu_sample_t imu = {.valid = true, .yaw_deg = 0.0f, .timestamp_us = 990000};
    uint64_t scan_us = 1000000;
    trap_points(points, 5, 0.0f);
    local_obstacle_map_update(&map, points, 5, scan_us, k_objects, 1, scan_us, &imu);
    local_planner_state_reset(&state);
    assert(local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN) ==
           LOCAL_ACTION_REVERSE);
    assert(state.escaping);

    /* Con el minimo de retroceso por defecto la reversa sigue hasta ganar carrera,
     * aunque el siguiente escaneo ya vea salida delantera... */
    scan_us += 100000;
    trap_points(points, 5, 10.0f); /* nueva geometria: ahora hay salida delantera */
    imu.timestamp_us = scan_us;
    local_obstacle_map_update(&map, points, 5, scan_us, k_objects, 1, scan_us, &imu);
    assert(local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN) ==
           LOCAL_ACTION_REVERSE);
    assert(state.escaping);

    /* ...y sin ese minimo (configurable) la cancela de inmediato. */
    local_planner_config_t cfg;
    legacy_config(&cfg);
    cfg.escape_min_back_cm = 0.0f;
    trap_points(points, 5, 0.0f);
    local_obstacle_map_update_cfg(&map, &cfg, points, 5, scan_us, k_objects, 1, scan_us, &imu);
    local_planner_state_reset(&state);
    assert(local_obstacle_map_recommend_cfg(&map, &cfg, &state, LOCAL_ACTION_FORWARD, NAN) ==
           LOCAL_ACTION_REVERSE);
    scan_us += 100000;
    trap_points(points, 5, 10.0f);
    imu.timestamp_us = scan_us;
    local_obstacle_map_update_cfg(&map, &cfg, points, 5, scan_us, k_objects, 1, scan_us, &imu);
    const uint8_t action = local_obstacle_map_recommend_cfg(&map, &cfg, &state,
                                                            LOCAL_ACTION_FORWARD, NAN);
    assert(action != LOCAL_ACTION_REVERSE);
    assert(!state.escaping);
    assert(state.escapes == 0u);
}

/* La prediccion larga debe ayudar a preferir carriles abiertos, pero no anular
 * una maniobra delantera segura: el siguiente escaneo permite replanear. */
static void test_predictive_pass(void)
{
    local_planner_state_t state;
    local_obstacle_map_t map;
    uint8_t action;

    /* Bolsillo de 20 cm: pared al frente a 20 cm y paredes laterales a ~13 cm. */
    const local_lidar_point_t pocket_twenty[] = {
        {.angle_deg = 0, .distance_cm = 20, .valid = true},
        {.angle_deg = 40, .distance_cm = 16, .valid = true},
        {.angle_deg = -40, .distance_cm = 16, .valid = true},
        {.angle_deg = 70, .distance_cm = 14, .valid = true},
        {.angle_deg = -70, .distance_cm = 14, .valid = true}
    };
    const local_lidar_point_t pocket_twentyfive[] = {
        {.angle_deg = 0, .distance_cm = 25, .valid = true},
        {.angle_deg = 40, .distance_cm = 18, .valid = true},
        {.angle_deg = -40, .distance_cm = 18, .valid = true},
        {.angle_deg = 70, .distance_cm = 15, .valid = true},
        {.angle_deg = -70, .distance_cm = 15, .valid = true}
    };
    /* Bolsillo de 45 cm: se ve el fondo, pero cabe la vuelta, asi que se entra. */
    const local_lidar_point_t pocket_fortyfive[] = {
        {.angle_deg = 0, .distance_cm = 45, .valid = true},
        {.angle_deg = 45, .distance_cm = 26, .valid = true},
        {.angle_deg = -45, .distance_cm = 26, .valid = true},
        {.angle_deg = 75, .distance_cm = 20, .valid = true},
        {.angle_deg = -75, .distance_cm = 20, .valid = true}
    };

    /* El bolsillo corto no fuerza reversa mientras exista una maniobra frontal
     * que supere el compromiso inmediato y la huella tenga espacio. */
    local_obstacle_map_update(&map, pocket_twenty, 5, 1000000, k_objects, 1, 990000, NULL);
    local_planner_state_reset(&state);
    action = local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN);
    if (action == LOCAL_ACTION_REVERSE || action == LOCAL_ACTION_STOP) {
        fail("bolsillo de 20 cm (deberia buscar salida frontal)", action);
        assert(false);
    }
    assert(!state.escaping);

    local_obstacle_map_update(&map, pocket_twentyfive, 5, 1000000, k_objects, 1, 990000, NULL);
    local_planner_state_reset(&state);
    action = local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN);
    if (action == LOCAL_ACTION_REVERSE || action == LOCAL_ACTION_STOP) {
        fail("bolsillo de 25 cm (deberia buscar salida frontal)", action);
        assert(false);
    }

    /* El P4 puede conducir solo con un escaneo LiDAR fresco aunque la cámara
     * esté desconectada o haya expirado su tabla. */
    local_obstacle_map_update(&map, pocket_fortyfive, 5, 1000000, NULL, 0, 0, NULL);
    local_planner_state_reset(&state);
    action = local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_STOP, NAN);
    if (action == LOCAL_ACTION_STOP || action == LOCAL_ACTION_REVERSE) {
        fail("LiDAR autonomo sin tabla reciente de camara", action);
        assert(false);
    }

    /* Un bolsillo mas profundo no es un bloqueo: hay que seguir pasando. */
    plan(&action, pocket_fortyfive, 5, LOCAL_ACTION_FORWARD);
    if (action == LOCAL_ACTION_STOP || action == LOCAL_ACTION_REVERSE) {
        fail("bolsillo de 45 cm (deberia pasar)", action);
        assert(false);
    }
}

/* El retroceso de una tentativa esta acotado por las fases de la maniobra: el
 * retroceso busca hueco lateral (1 ciclo) y el margen son
 * LOCAL_ESCAPE_MARGIN_CM ciclos. El unico tope que hace falta es el numero de
 * tentativas. */
static void test_escape_attempts_bounded(void)
{
    local_obstacle_map_t map;
    local_planner_state_t state;
    const local_lidar_point_t boxed[] = {
        {.angle_deg = 0, .distance_cm = 12, .valid = true},
        {.angle_deg = -20, .distance_cm = 13, .valid = true},
        {.angle_deg = 20, .distance_cm = 13, .valid = true},
        {.angle_deg = -45, .distance_cm = 15, .valid = true},
        {.angle_deg = 45, .distance_cm = 15, .valid = true},
        /* Paredes laterales a la altura del chasis, que es donde se mide el
         * hueco: con ellas los 30 cm de hueco no llegan nunca (10 cm a cada
         * lado), asi que el retroceso no puede ganar margen. */
        {.angle_deg = -125, .distance_cm = 12.2, .valid = true},
        {.angle_deg = 125, .distance_cm = 12.2, .valid = true}
    };
    int cycles = 0, reversals = 0, margin = 0, attempts = 0;
    uint8_t previous = 0u, last = 0u;

    local_obstacle_map_update(&map, boxed, 7, 1000000, k_objects, 1, 990000, NULL);
    local_planner_state_reset(&state);
    for (int i = 0; i < 400; ++i) {
        if (state.escape_phase == LOCAL_ESCAPE_PHASE_MARGIN) ++margin;
        last = local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN);
        if (last == LOCAL_ACTION_REVERSE) ++reversals;
        if (state.escapes > previous) attempts += (int)(state.escapes - previous);
        previous = state.escapes;
        ++cycles;
        if (last == LOCAL_ACTION_STOP) break;
    }
    /* Sin hueco las tentativas se agotan y el robot para; y cada tentativa gasta
     * un margen acotado, no una reversa sin fin. */
    if (last != LOCAL_ACTION_STOP) { fail("caja sin hueco lateral", last); assert(false); }
    assert(!state.escaping);
    assert(cycles < 400);
    assert(attempts == (int)LOCAL_ESCAPE_MAX_ATTEMPTS);
    /* Cada tentativa retrocede hasta el tope y luego gasta su margen: acotado, y
     * sin rendirse antes de intentarlo. */
    assert(reversals >= attempts * (int)LOCAL_ESCAPE_BACK_MAX_CM);
    assert(reversals <= attempts * ((int)LOCAL_ESCAPE_BACK_MAX_CM
                                    + (int)LOCAL_ESCAPE_MARGIN_CM + 4));
    assert(margin == attempts * ((int)LOCAL_ESCAPE_MARGIN_CM + 1));
}

static void test_escape_without_room(void)
{
    /* Muro congelado a 12 cm: no hay donde retroceder porque el mapa no cambia,
     * asi que tras gastar el margen la comprobacion del giro falla y la
     * maniobra tiene que acabar en STOP, no devolviendo REVERSE ni el giro para
     * siempre. */
    local_obstacle_map_t map;
    local_planner_state_t state;
    local_imu_sample_t imu = {.valid = true, .yaw_deg = 0.0f, .timestamp_us = 990000};
    uint64_t scan_us = 1000000;
    int cycles = 0, margin = 0, reversals = 0;
    uint8_t last = 0u;

    local_obstacle_map_update(&map, k_trap, 5, scan_us, k_objects, 1, scan_us, &imu);
    local_planner_state_reset(&state);

    for (int i = 0; i < 400; ++i) {
        if (state.escape_phase == LOCAL_ESCAPE_PHASE_MARGIN) ++margin;
        imu.yaw_deg += 10.0f;   /* el chasis gira, pero el muro no se aleja */
        imu.timestamp_us = scan_us;
        scan_us += 100000;
        local_obstacle_map_update(&map, k_trap, 5, scan_us, k_objects, 1, scan_us, &imu);
        last = local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN);
        if (last == LOCAL_ACTION_REVERSE) ++reversals;
        ++cycles;
        /* Tras abortar el giro el planificador reintenta; solo se para cuando se
         * le acaban los intentos. */
        if (last == LOCAL_ACTION_STOP && state.escapes >= LOCAL_ESCAPE_MAX_ATTEMPTS) break;
    }
    assert(last == LOCAL_ACTION_STOP);
    assert(!state.escaping);
    assert(cycles < 40 * (int)LOCAL_ESCAPE_MAX_ATTEMPTS);
    assert(state.escapes == LOCAL_ESCAPE_MAX_ATTEMPTS);
    assert(margin == (int)state.escapes * ((int)LOCAL_ESCAPE_MARGIN_CM + 1));
    assert(reversals <= (int)state.escapes * ((int)LOCAL_ESCAPE_MARGIN_CM + 2));
    /* Y se queda parado: insistir con el mismo muro no cambia la decision. */
    local_obstacle_map_update(&map, k_trap, 5, scan_us + 100000, k_objects, 1,
                              scan_us + 100000, &imu);
    assert(local_obstacle_map_recommend(&map, &state, LOCAL_ACTION_FORWARD, NAN) ==
           LOCAL_ACTION_STOP);
}

int main(void)
{
    use_test_chassis();
    test_map_layers();
    test_planner_properties();
    test_lane_reference();
    test_escape_cycle();
    test_predictive_pass();
    test_escape_attempts_bounded();
    test_escape_without_room();
    puts("PASS: capas del mapa, campo de distancias, reticula de rutas y reversa");
    return 0;
}

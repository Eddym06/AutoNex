#ifndef LOCAL_OBSTACLE_MAP_H
#define LOCAL_OBSTACLE_MAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Rejilla de 2,5 cm: con celdas de 5 cm el retorno de un pilar caia hasta 2,5 cm
 * dentro de la celda y, junto al redondeo de la huella, un hueco real de 20 cm
 * entre dos pilares quedaba en 10 cm utiles: el carrito de 8 cm no "cabia". */
#define LOCAL_MAP_SIZE 160
#define LOCAL_MAP_CELL_CM 2.5f
#define LOCAL_MAP_CENTER (LOCAL_MAP_SIZE / 2)
#define LOCAL_MAP_MAX_POINTS 512
#define LOCAL_MAP_MAX_CAMERA_OBJECTS 12
#define LOCAL_SENSOR_SYNC_MAX_AGE_US 150000u
/* Montaje de los sensores respecto al EJE de las ruedas motrices. Provisional: el equipo
 * todavia no los ha montado; hay que medirlos. */
#define LOCAL_LIDAR_FORWARD_CM 10.0f
#define LOCAL_CAMERA_FORWARD_CM 13.0f

/* Geometria del chasis y del volante. Son los VALORES POR DEFECTO de
 * local_planner_config_t: hay que medirlos en el carrito real y ajustarlos en la
 * configuracion (un solo sitio), no recompilar. */
#define LOCAL_CHASSIS_WIDTH_CM 14.0f
#define LOCAL_CHASSIS_LENGTH_CM 22.0f
#define LOCAL_CHASSIS_REAR_CM 4.0f
/* Ackermann: curvatura = tan(ángulo efectivo) / batalla. Valores provisionales. */
#define LOCAL_WHEELBASE_CM 14.0f
#define LOCAL_STEER_NUDGE_DEG 4.0f
#define LOCAL_STEER_SOFT_DEG 14.0f
#define LOCAL_STEER_HARD_DEG 30.0f

/* Planificador por retícula de rutas: tres arcos sobre las cinco acciones de
 * avance. Ver la nota de diseno en local_obstacle_map.c. */
#define LOCAL_LATTICE_DEPTH 3
#define LOCAL_LATTICE_ARC_CM 20.0f
#define LOCAL_LATTICE_STEP_CM 5.0f
#define LOCAL_LATTICE_DISCOUNT 0.9f
#define LOCAL_LATTICE_HORIZON_CM (LOCAL_LATTICE_DEPTH * LOCAL_LATTICE_ARC_CM)
/* Apartamiento lateral, alineacion y compromiso minimo. Los dos ultimos se miden
 * contra el horizonte: por debajo de LOCAL_MIN_COMMIT_CM ninguna ruta cuenta. */
#define LOCAL_CLEARANCE_PREF_CM 12.0f
#define LOCAL_LATERAL_SCALE_CM 25.0f
#define LOCAL_MIN_COMMIT_CM 15.0f
#define LOCAL_STEP_GUARD_CM 2.0f

/* Referencia lateral. El carril se mide a la altura del chasis en tres filas y
 * se toma la mediana, porque una celda suelta (un bloque rebasado, un objeto de
 * camara) mueve el centro decenas de centimetros. Una fila solo es carril si las
 * dos paredes estan dentro de LOCAL_LANE_MAX_SPAN_CM. Si las filas no coinciden
 * dentro de LOCAL_LANE_SPREAD_CM tampoco hay carril y la referencia es cero. */
#define LOCAL_LANE_MAX_SPAN_CM 150.0f
#define LOCAL_LANE_SPREAD_CM 15.0f
#define LOCAL_LANE_ROWS 3

/* Vuelta a la linea (ver local_obstacle_map.c, "Volver a la linea"). */
#define LOCAL_RECENTER_LANE_CM 10.0f
#define LOCAL_RECENTER_HARD_DEG 30.0f
#define LOCAL_RECENTER_HARD_CM 30.0f
#define LOCAL_RECENTER_SLACK 6.0f
#define LOCAL_RECENTER_SLACK_HEADING 0.5f
#define LOCAL_RECENTER_DEG_TO_CM 2.0f
#define LOCAL_RECENTER_CLEAR_CM 60.0f

/* Prediccion de paso. */
#define LOCAL_PASS_PROBE_CM 150.0f
#define LOCAL_MIN_PASS_CM 30.0f

/* Maniobra de escape (reversa) cuando la retícula no encuentra ruta. */
#define LOCAL_ESCAPE_ROOM_CM 30.0f
#define LOCAL_ESCAPE_MARGIN_CM 25.0f
#define LOCAL_ESCAPE_TURN_DEG 80.0f
#define LOCAL_ESCAPE_PROBE_PAD_CM 5.0f
#define LOCAL_ESCAPE_MAX_ATTEMPTS 3u
#define LOCAL_ESCAPE_BACK_MAX_CM 50.0f
#define LOCAL_ESCAPE_DONE 0xFFu

enum {
    LOCAL_CELL_FREE = 1u << 0,
    LOCAL_CELL_OCCUPIED = 1u << 1,
    LOCAL_CELL_VISION = 1u << 2
};

/* --------------------------------------------------------------- configuracion
 *
 * Todo lo que depende del carrito, de la pista o de la velocidad vive aqui. Sin
 * odometria, el progreso de las maniobras se estima con velocidad x tiempo: por
 * eso las velocidades estan en la configuracion y no enterradas en el codigo. */
typedef struct {
    /* Geometria y montaje de sensores (cm, grados). */
    float chassis_width_cm, chassis_length_cm, wheelbase_cm;
    float chassis_rear_cm;       /* voladizo trasero respecto al eje motriz */
    float lane_half_cm;          /* mitad del ancho del carril: la regla de color solo vale para letreros dentro de el */
    float tight_clear_cm;        /* con menos holgura que esto en la ruta elegida se avanza a velocidad de maniobra (0 = nunca) */
    float squeeze_cm;            /* si de golpe no hay ruta pero hace un instante si, se reintenta con la huella tanto mas estrecha antes de retroceder (0 = nunca) */
    float footprint_shrink_cm;   /* resta al ancho del chasis al comprobar la huella (compensa el redondeo de celdas) */
    float steer_nudge_deg, steer_soft_deg, steer_hard_deg;
    float lidar_forward_cm, camera_forward_cm, lidar_min_cm;

    /* Movimiento estimado, sin odometria. */
    float speed_cm_s;            /* crucero ordenado al motor */
    float maneuver_speed_cm_s;   /* reversa y giros de escape */
    float cycle_s;               /* periodo de control (un escaneo) */
    float scan_period_s;         /* vuelta del LiDAR, para corregir el giro durante el barrido */
    uint32_t sync_max_age_us;    /* antiguedad maxima de camara/IMU respecto al escaneo */

    /* Pesos del coste (normalizados a ~[0,1]). */
    float w_clearance, w_lateral, w_heading, w_turn, w_progress;
    float w_dead_end, w_change, w_camera, w_pass_shortfall, switch_tie;

    /* Retícula. */
    float arc_cm;                /* longitud de cada uno de los 3 arcos */
    float discount;
    float clearance_pref_cm, lateral_scale_cm, min_commit_cm;
    float step_guard_cm;         /* 0 = automatico: 2 ciclos de avance, minimo 2 cm */
    /* Seguimiento fino del carril: con el camino libre, el volante sigue la suma
     * del error de rumbo y del apartamiento lateral (mirando `track_lookahead_cm`
     * adelante) con la accion de menor curvatura que lo corrige. La retícula solo
     * decide cuando hay obstaculos: sus acciones suaves (12 grados) son un golpe
     * demasiado grande para corregir 3 grados de rumbo. */
    float track_lookahead_cm, track_deadband_deg, track_slack;

    /* Corredor: la retícula solo mira 60 cm, y un pilar que pide ponerse 30 cm a un
     * lado necesita mas carrera que esa. El corredor mira `plan_horizon_cm` en el
     * sistema del carril (adelante = rumbo del recorrido), busca por programacion
     * dinamica la trayectoria lateral que cabe entre paredes, pilares y cortinas
     * -prefiere el centro del carril y la holgura- y el volante la persigue con
     * pursuit puro. La retícula sigue siendo quien vigila la seguridad inmediata. */
    bool plan_enabled;
    float plan_margin_extra_cm;  /* el corredor se planea primero con este margen de mas; si no cabe, con el justo */
    float plan_horizon_cm, plan_margin_cm, plan_lookahead_cm, plan_slack;
    float plan_w_lateral, plan_w_clearance, plan_w_slew;
    /* La retícula puntua cada ruta por su distancia al corredor (en vez de al
     * centro del carril): es la que sabe si el chasis PUEDE seguirlo, con su radio
     * de giro y su huella. */
    float plan_w_path, plan_path_scale_cm;
    /* Referencia lateral externa (la linea de carrera): en las filas donde el controlador la da, el corredor prefiere
     * esa posicion (peso plan_w_race sobre plan_w_lateral) y se conforma con menos holgura (plan_race_clear_cm) en vez
     * de pegarse al centro del carril. La viabilidad la sigue decidiendo el mapa vivo del LiDAR. */
    float plan_w_race, plan_race_clear_cm;
    bool plan_refine;            /* afina el volante con pursuit puro hacia el corredor */
    float pass_probe_cm, min_pass_cm;

    /* Carril y vuelta a la linea. */
    float lane_max_span_cm, lane_spread_cm;
    float recenter_lane_cm, recenter_hard_deg, recenter_hard_cm;
    float recenter_slack, recenter_slack_heading, recenter_deg_to_cm, recenter_clear_cm;

    /* Escape. */
    float escape_room_cm, escape_margin_cm, escape_turn_deg, escape_probe_pad_cm;
    float escape_back_max_cm;
    /* La reversa de escape no se cancela hasta haber retrocedido esto: sin ese
     * minimo, una ruta delantera que aparece y desaparece con un centimetro de
     * diferencia deja al robot cabeceando adelante y atras sin ganar carrera. */
    float escape_min_back_cm;
    uint8_t escape_max_attempts;
    /* Agotados los intentos el robot se detiene, pero no para siempre: una ronda de
     * tres minutos perdida por un bloqueo de dos segundos es peor que otro intento. */
    float escape_retry_s;

    /* LiDAR: retornos y correccion. */
    float bridge_max_gap_cm;     /* une retornos vecinos mas cerca que esto */
    float bridge_max_jump_cm;    /* ...si su distancia al sensor casi no cambia */
    bool deskew;                 /* corrige la rotacion del chasis durante el barrido */

    /* Fusion con la camara y regla de color. */
    int8_t pass_side_red, pass_side_green; /* +1 pasar por la derecha, -1 izquierda, 0 sin regla */
    bool enforce_color_side;     /* false: el color no restringe la ruta */
    float curtain_len_cm;        /* longitud de la cortina virtual en el lado prohibido */
    float curtain_min_forward_cm;/* no se crea cortina para pilares ya rebasados */
    float assoc_bearing_tol_deg; /* camara <-> retorno LiDAR */
    float assoc_range_rel, assoc_range_abs_cm;
    float cluster_radius_cm;
    bool camera_ghost_fallback;  /* marcar ocupada una deteccion sin retorno LiDAR */
    bool curtain_unconfirmed;    /* cortina tambien para detecciones sin retorno LiDAR */
} local_planner_config_t;

void local_planner_config_default(local_planner_config_t *cfg);
/* Acota cada campo a un rango con el que el planificador funciona. */
/* Configuracion que usan las funciones sin parametro de configuracion (NULL). Sin llamarla, los
 * valores por defecto de local_planner_config_default. Las pruebas la usan para fijar la geometria
 * del escenario que describen. */
void local_planner_set_default_config(const local_planner_config_t *cfg);
bool local_planner_config_sanitize(local_planner_config_t *cfg);

/* Un color de camara vale para la regla de lado solo si la camara y el LiDAR
 * coinciden: la camara dice QUE es, el LiDAR dice DONDE esta. */
typedef struct {
    float x_cm, forward_cm;      /* respecto al eje trasero, derecha positiva */
    uint8_t color;               /* 1 rojo, 2 verde, 3 parqueo */
    bool confirmed;              /* hay un retorno LiDAR que lo respalda */
} local_colored_object_t;

typedef struct {
    float angle_deg;       /* zero forward, positive to robot's right */
    float distance_cm;
    float intensity;
    bool valid;
    bool blind;            /* valid == false por estar dentro de la zona ciega: se marca como obstaculo */
} local_lidar_point_t;

typedef struct {
    float bearing_deg;     /* camera-relative; zero is image center */
    float distance_cm;     /* alcance a lo largo del rayo */
    uint8_t color;         /* 1 red, 2 green, 3 parking marker */
    bool valid;
    uint8_t flags;         /* WRO_OBJ_* del protocolo */
} local_camera_object_t;

typedef struct {
    bool valid;
    float yaw_deg;         /* compass-style heading, clockwise positive */
    uint64_t timestamp_us;
    bool has_rate;
    float yaw_rate_dps;    /* clockwise positive; enables LiDAR deskew */
} local_imu_sample_t;

typedef struct {
    uint8_t cells[LOCAL_MAP_SIZE][LOCAL_MAP_SIZE];
    uint64_t timestamp_us;
    float yaw_deg;
    size_t lidar_points;
    size_t lidar_hits;
    size_t camera_hits;          /* detecciones de camara ancladas al mapa */
    size_t camera_unmatched;     /* sin retorno LiDAR que las respalde */
    bool imu_fresh;
    bool camera_fresh;
    size_t colored_count;
    local_colored_object_t colored[LOCAL_MAP_MAX_CAMERA_OBJECTS];
} local_obstacle_map_t;

/* Acciones. Los valores 0..11 coinciden con vision_action_t para que el mapa
 * pueda reenviar tal cual la propuesta de camara. LOCAL_ACTION_REVERSE queda
 * fuera de ese enum: el mapa no la recomienda por si mismo, la devuelve
 * unicamente la maniobra de escape cuando la retícula no halla ruta y hay
 * hueco detras, y el controlador debe tratarla como recuperacion. */
typedef enum {
    LOCAL_ACTION_STOP = 0,
    LOCAL_ACTION_FORWARD = 1,
    LOCAL_ACTION_RIGHT_SOFT = 2,
    LOCAL_ACTION_RIGHT_HARD = 3,
    LOCAL_ACTION_LEFT_SOFT = 4,
    LOCAL_ACTION_LEFT_HARD = 5,
    LOCAL_ACTION_PARK_LEFT = 6,
    LOCAL_ACTION_PARK_RIGHT = 7,
    LOCAL_ACTION_PARK_APPROACH = 8,
    LOCAL_ACTION_PARK_READY = 9,
    LOCAL_ACTION_RETURN_LEFT = 10,
    LOCAL_ACTION_RETURN_RIGHT = 11,
    LOCAL_ACTION_REVERSE = 12,
    LOCAL_ACTION_NUDGE_RIGHT = 13,   /* correccion fina: no la propone la camara */
    LOCAL_ACTION_NUDGE_LEFT = 14
} local_action_t;

typedef enum {
    LOCAL_ESCAPE_PHASE_NONE = 0u,
    LOCAL_ESCAPE_PHASE_BACK = 1u,
    LOCAL_ESCAPE_PHASE_MARGIN = 2u,
    LOCAL_ESCAPE_PHASE_TURN = 3u
} local_escape_phase_t;

/* Lo que el planificador penso en el ultimo ciclo, para dibujarlo o registrarlo.
 * Solo se rellena si state->debug apunta a una estructura. */
#define LOCAL_DEBUG_ROUTE_POINTS 48
typedef struct {
    float lane_cm;               /* centro del carril medido */
    float reference_deg;         /* error de rumbo respecto al recorrido */
    float pass_cm;               /* paso previsto de la ruta elegida; <0 si no hay */
    float min_clear_cm;          /* holgura minima de la huella en el primer tramo de la ruta elegida */
    float cost;
    uint8_t seq[LOCAL_LATTICE_DEPTH];
    size_t route_count;
    float route[LOCAL_DEBUG_ROUTE_POINTS][2];  /* x, adelante */
    size_t probe_count;
    float probe[LOCAL_DEBUG_ROUTE_POINTS][2];
    size_t curtain_cells;        /* celdas virtuales de la regla de color */
    float free_cm;               /* lo mas lejos que llega libre la mejor ruta candidata (aunque no sea viable) */
    size_t plan_count;           /* puntos del corredor */
    float plan[LOCAL_DEBUG_ROUTE_POINTS][2];
} local_planner_debug_t;

/* Estado que sobrevive entre ciclos: histeresis de la decision anterior y la
 * maniobra de escape en curso. El mapa se reconstruye entero en cada escaneo,
 * asi que esto no puede vivir dentro de local_obstacle_map_t. */
#define LOCAL_REF_ROWS 40
typedef struct {
    uint8_t last_action;
    /* Posicion lateral preferida (cm, derecha positiva, ejes del carril) a 5, 10, 15... cm por delante; NAN = sin
     * preferencia en esa fila. La rellena el controlador cada ciclo. */
    float ref_v_cm[LOCAL_REF_ROWS];
    int ref_rows;
    /* Desplazamiento pedido de la linea del carril (cm, derecha positiva): pegarse a un lado,
     * por ejemplo a la pared exterior para ver el cajon de parqueo. Solo actua si el carril
     * se mide (dos paredes a la vista). */
    float lane_offset_cm;
    /* Volante continuo (grados, derecha positiva) que acompana a `last_action`: el
     * seguimiento fino y el corredor piden curvaturas entre las cinco discretas,
     * y un carrito de verdad tiene un servo, no cinco posiciones. */
    float steer_deg;
    bool escaping;
    uint8_t escape_phase;         /* local_escape_phase_t */
    uint8_t escape_dir;           /* 0 sin elegir, 1 derecha, 2 izquierda */
    uint8_t escapes;              /* maniobras consecutivas sin ruta viable */
    uint16_t stop_cycles;         /* ciclos parado tras agotar los intentos */
    float escape_back_left_cm;
    float escape_back_cm;         /* retroceso acumulado en la tentativa actual */
    float escape_turn_start_deg;  /* rumbo al empezar el giro comprometido */
    float escape_turned_deg;      /* giro acumulado cuando no hay IMU fresco */
    bool escape_use_imu;          /* fuente del rumbo en ese giro */
    local_planner_debug_t *debug; /* opcional */
} local_planner_state_t;

void local_planner_state_reset(local_planner_state_t *state);

/* Replaces the local map with one lidar scan and overlays only fresh camera data.
 * Coordinates are body-relative; this deliberately performs no odometry/SLAM.
 * La version _cfg usa la geometria y los parametros dados; la otra, los de
 * local_planner_config_default(). Los puntos deben venir en orden de barrido. */
void local_obstacle_map_update_cfg(local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                   const local_lidar_point_t *points, size_t point_count,
                                   uint64_t scan_timestamp_us,
                                   const local_camera_object_t *objects, size_t object_count,
                                   uint64_t camera_timestamp_us,
                                   const local_imu_sample_t *imu);
void local_obstacle_map_update(local_obstacle_map_t *map,
                               const local_lidar_point_t *points, size_t point_count,
                               uint64_t scan_timestamp_us,
                               const local_camera_object_t *objects, size_t object_count,
                               uint64_t camera_timestamp_us,
                               const local_imu_sample_t *imu);

/* Letreros ya vistos y recordados (posicion en ejes del robot y color). La camara
 * solo ve de frente: cuando el pilar sale de su campo, el color se conserva y se
 * le vuelve a poner cara con el retorno LiDAR que hay en su sitio, para que la
 * regla de lado siga cerrando el lado prohibido hasta rebasarlo. Hay que llamarla
 * justo despues de local_obstacle_map_update(_cfg). */
void local_obstacle_map_add_memory(local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                   const float *x_forward_pairs, const uint8_t *colors, size_t count);
/* Igual, pero el recuerdo puede anclarse a un retorno LiDAR hasta `anchor_cm` de donde se espera (0 = el defecto). */
/* Si `found_xf` no es NULL (2 floats por recuerdo) recibe donde se encontro el pilar, y `found` (1 si se ancló). */
void local_obstacle_map_add_memory_within(local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                          const float *xf, const uint8_t *colors, size_t count, float anchor_cm,
                                          float *found_xf, uint8_t *found);

/* Un pilar compacto del LiDAR (5 cm de lado) delante, sin ningun objeto con color asociado, cuyo rumbo respecto al
 * morro esta entre `min_bearing_deg` y `max_bearing_deg` (fuera del centro del campo de la camara): el robot puede
 * girar un poco hacia el para que la camara vea su color antes de pasarlo. Devuelve false si no hay. */
bool local_obstacle_map_unlabeled_pillar(const local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                         float min_bearing_deg, float max_bearing_deg,
                                         float min_forward_cm, float max_forward_cm,
                                         float *x_cm, float *forward_cm);

/* Pilares compactos del LiDAR (grupos de hasta 10 cm, no paredes) entre dos distancias por delante; devuelve cuantos y
 * rellena (x, adelante) de cada uno. Incluye los que ya tienen color: el llamador decide si son conocidos. */
size_t local_obstacle_map_compact_pillars(const local_obstacle_map_t *map, const local_planner_config_t *cfg,
                                          float min_forward_cm, float max_forward_cm, float *xf_pairs, size_t capacity);

bool local_obstacle_map_get(const local_obstacle_map_t *map,
                            int x_cell, int forward_cell, uint8_t *value);

/* Seguridad sobre el escaneo actual. Se comparan todas las rutas de la retícula
 * contra un campo de distancias y se elige la de menor costo: avance y giros
 * salen de ahi, no de umbrales fijos. La camara aporta el COLOR: con la regla de
 * lado activa, el lado equivocado de cada pilar confirmado por el LiDAR se
 * cierra con una cortina virtual y la ruta ya no puede cruzarla.
 *
 * Si ninguna ruta sobrevive, arranca la maniobra de escape y puede devolver
 * LOCAL_ACTION_REVERSE. El estado se conserva en `state` entre llamadas.
 *
 * `course_heading_deg` es el rumbo del recorrido segun el controlador (mismo
 * convenio que local_imu_sample_t::yaw_deg); NAN si no se conoce. */
uint8_t local_obstacle_map_recommend_cfg(const local_obstacle_map_t *map,
                                         const local_planner_config_t *cfg,
                                         local_planner_state_t *state,
                                         uint8_t camera_action,
                                         float course_heading_deg);
uint8_t local_obstacle_map_recommend(const local_obstacle_map_t *map,
                                     local_planner_state_t *state,
                                     uint8_t camera_action,
                                     float course_heading_deg);

/* Holgura delantera (cm) a lo largo de un arco de la huella, 0 si choca: sirve
 * al controlador para comprobar un giro de esquina antes de comprometerlo. */
/* Hay sitio para retroceder (la huella por detras del eje no toca nada). */
bool local_planner_rear_clear(const local_obstacle_map_t *map, const local_planner_config_t *cfg);
/* Distancia (cm) del eje motriz al obstaculo mas cercano en el campo de distancias del ULTIMO
 * local_obstacle_map_recommend: sirve para saber si el chasis cabe girando en el sitio. */
float local_planner_axle_clearance_cm(void);
bool local_planner_arc_clear(const local_obstacle_map_t *map, const local_planner_config_t *cfg,
                             float steer_deg, float distance_cm);

#endif

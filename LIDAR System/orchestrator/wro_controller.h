#ifndef WRO_CONTROLLER_H
#define WRO_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "camera_table_receiver.h"
#include "imu_yaw.h"
#include "local_obstacle_map.h"
#include "arena_localizer.h"
#include "odometry.h"
#include "park_localize.h"
#include "park_planner.h"
#include "park_ackermann.h"
#include "race_line.h"

/* Controlador de la mision del P4 (WRO Future Engineers, reto con obstaculos).
 *
 * Es codigo portable, sin dependencias del hardware: recibe las muestras de los
 * sensores y devuelve la orden de volante y velocidad. Los drivers (UART del D6,
 * bus del MPU, PWM del puente H y del servo) viven en el adaptador del P4 y
 * llaman a estas funciones; el mismo codigo corre en el simulador del navegador.
 *
 * NO HAY ODOMETRIA. Lo que ata al carrito al circuito es:
 *   - el rumbo del giroscopio, reanclado contra las paredes en cada recta;
 *   - la distancia al muro frontal y los huecos laterales del LiDAR (esquinas);
 *   - velocidad ordenada x tiempo para los tramos cortos (parada final).
 * El circuito (reglamento): cuatro esquinas y cuatro rectas por vuelta, tres
 * vueltas. El sentido (horario/antihorario) lo decide el sorteo y el robot no
 * puede recibirlo como dato: lo aprende en la primera esquina, que es por donde
 * se abre el lado interior. */

typedef enum {
    WRO_PHASE_IDLE = 0,       /* esperando el boton; el giroscopio calibra su sesgo */
    WRO_PHASE_RUN,            /* vueltas */
    WRO_PHASE_UTURN,          /* media vuelta antes de la ultima vuelta */
    WRO_PHASE_FINISH,         /* tres vueltas hechas: parar dentro de la seccion de salida */
    WRO_PHASE_PARK_SEARCH,    /* buscar el parqueo (los topes magenta) */
    WRO_PHASE_PARKING,        /* maniobra de parqueo en paralelo */
    WRO_PHASE_DONE,
    WRO_PHASE_FAULT,
    WRO_PHASE_PARK_EXIT       /* arranco dentro del cajon: salir al carril (reglamento 2026: 7 puntos) */
} wro_phase_t;

typedef struct {
    local_planner_config_t planner;

    /* Pista (reglamento: carril de 100 cm en el reto con obstaculos). */
    float lane_width_cm;
    uint8_t laps;                   /* vueltas oficiales */
    float time_limit_s;             /* 0 = sin limite */

    /* Esquinas. El giro duro empieza cuando el muro frontal esta a
     * lane/2 + radio - adelanto del LiDAR (+ este extra) para salir centrado. */
    float corner_trigger_extra_cm;
    float corner_min_travel_cm;     /* entre dos esquinas: evita contar la misma dos veces */
    float corner_done_deg;          /* error de rumbo con el que se da por terminada */
    float open_side_cm;             /* un lado mas lejos que esto esta abierto */
    float wall_align_gain;          /* reancla el rumbo con las paredes (0 = no) */
    float wall_align_max_deg;
    float wall_align_heading_deg;   /* solo reancla con el morro a menos de esto del carril */
    float corner_gyro_deg;          /* esquina por giroscopio si el muro no se vio (0 = no) */
    float corner_gyro_min_travel_cm;/* ese respaldo exige haber recorrido tanto desde la ultima esquina: una esquiva no es una esquina */
    float corner_max_heading_deg;  /* el muro frontal solo cuenta como esquina con el morro a menos de esto del carril */
    /* Giro en el sitio para salir de un atasco (solo un robot diferencial): sin ruta hacia delante durante
     * `pivot_after_s` y con el morro girado de la pista, gira hacia ella si el chasis cabe girando. */
    bool pivot_enabled;
    float pivot_after_s, pivot_rot_dps, pivot_margin_cm;
    /* Desplazamiento lateral con giros en el sitio (solo diferencial): cuando un letrero pide un cambio de lado
     * que las curvas no alcanzan en la recta que hay, gira 90 grados, avanza de lado lo que falta y vuelve a girar. */
    /* Mirar antes de pasar: un pilar del LiDAR sin color fuera del centro del campo de la camara (la camara ve unos
     * +-34 grados) se pasaria a ciegas; se gira un poco hacia el (sin salirse mas de peek_max_heading_deg del carril)
     * para que la camara llegue a verlo y su cortina fije el lado correcto. */
    bool arena_gates_corners;       /* una esquina solo puede empezar si la pose del campo esta en una zona de esquina */
    float arena_corner_margin_cm;
    bool peek_enabled;
    float peek_bearing_min_deg, peek_max_heading_deg, peek_rate_dps;
    bool arena_signs_enabled;       /* memoria de letreros en coordenadas del campo (necesita la pose del localizador) */
    bool arena_map_lead;            /* los letreros se llevan al campo con la pose a la hora del mapa del planificador (avance y giro hasta que la orden hace efecto) */
    float arena_sign_bias_cm;       /* el centroide LiDAR de un letrero esta en su cara, no en su centro: se corrige tanto al recordarlo como al reinyectarlo */
    bool arena_enabled;             /* corrige la pose con el LiDAR contra las paredes conocidas del campo */
    arena_config_t arena;           /* mitad del cuadrado exterior e interior, tolerancias */
    bool odometry_enabled;          /* usa los encoders de las ruedas si llegan muestras (si no, velocidad ordenada x tiempo) */
    odom_config_t odom;             /* rueda, via, escalas y signos de los encoders (se calibran) */
    /* Linea de carrera (race_line.h): terminada la primera vuelta se calcula la trayectoria y el perfil de velocidad con
     * los letreros conocidos, y las vueltas siguientes la usan de referencia. El LiDAR sigue mandando. */
    /* Pegarse a la pared interior (vueltas 2 y 3): la primera vuelta va centrada y recuerda los letreros; despues el robot
     * va pegado a la pared interior en las rectas y se despega solo cuando un letrero conocido le exige el otro lado. */
    bool hug_enabled;
    float hug_offset_cm;            /* cuanto se separa del centro hacia la pared interior */
    float hug_detach_cm;            /* un letrero conocido a menos de esto por delante fija el lado donde hay que estar */
    float hug_after_sign_cm;        /* y se sigue respetando hasta que el robot lo ha dejado atras tanto */
    float hug_corner_clear_cm;      /* >0: a tanto del giro de la esquina vuelve al centro (0 = llega pegado) */
    float hug_after_corner_cm;      /* tras una esquina, centrado durante este recorrido */
    float hug_rate;                 /* cm de desplazamiento lateral por cm avanzado (pendiente maxima del cambio) */
    float hug_unknown_hold_s;       /* un pilar que no esta en el mapa: centrado este tiempo */
    uint8_t hug_stop_before_end;    /* las ultimas esquinas se hacen centrado (entrega limpia al parqueo) */
    bool race_enabled;
    bool race_speed_enabled;        /* la velocidad sigue el perfil de la linea (mas rapido en recta libre, la base junto a letreros y esquinas) */
    bool race_lateral_enabled;      /* el corredor del planificador prefiere la posicion lateral de la linea */
    race_config_t race;
    uint8_t race_after_corners;     /* esquinas completadas antes de calcularla (4 = tras la primera vuelta) */
    uint8_t race_stop_before_end;   /* la linea se deja de usar cuando faltan tantas esquinas para terminar (entrega limpia al parqueo) */
    float race_lookahead_cm;        /* la velocidad se toma tanto por delante en la linea (compensa el retardo del motor) */
    float race_gain_lap2;           /* fraccion del exceso de velocidad sobre la base permitida en la 2.a vuelta (la 3.a usa todo) */
    float race_max_dist_cm;         /* lejos de la linea no se usa (esquivando algo, o recuperandose) */
    float race_speed_dist_cm;       /* la velocidad extra solo se concede a menos de esto de la linea */
    float race_unknown_hold_s;      /* tras ver un pilar que no esta en el mapa, velocidad base durante este tiempo */
    float race_corner_sign_cm, race_corner_sign_after_cm;   /* con un letrero conocido a menos de esto antes / despues de la esquina, esta no se adelanta */
    float race_blend_before_cm, race_blend_after_cm;   /* la referencia lateral se mezcla con el centro del carril a tanto antes / despues de una esquina */
    float race_slip_derate;         /* cada patinaje de los encoders desde que se calculo la linea quita esta fraccion del exceso de velocidad (agarre real) */
    float race_corner_extra_cm;     /* en la carrera la esquina empieza tanto antes (corta la esquina por dentro) */
    bool lap_memory_enabled;        /* los letreros vistos en una vuelta se recuerdan por seccion para las siguientes */
    bool sidestep_enabled;
    float sidestep_after_s, sidestep_cooldown_s, sidestep_speed_cm_s, sidestep_margin_cm;
    float corner_abort_cm;          /* una esquina que no termina en este recorrido se da por falsa y se deshace */
    bool motion_comp;               /* adelanta el barrido al instante en que la orden hace efecto */
    float latency_s;                /* calculo + actuacion hasta que el volante responde */
    float stuck_relax_s;            /* sin avanzar tanto tiempo: suspende el lado de paso un trecho (0 = nunca) */
    float stuck_relax_cm;           /* cuanto avanza antes de volver a exigirlo */

    /* Ultima vuelta. El ultimo letrero de la segunda vuelta decide si se sigue
     * (verde) o se da media vuelta (rojo). Los colores son dato porque una regla
     * sorpresa puede cambiarlos; 0 desactiva la media vuelta. */
    uint8_t reverse_on_color;       /* 1 = rojo (reglamento) */
    uint8_t continue_on_color;      /* 2 = verde (reglamento) */
    bool turnaround_enabled;

    /* Final. */
    float finish_extra_cm;          /* avance (velocidad x tiempo) tras ver la pared interior */
    float uturn_extra_cm;           /* idem antes de la media vuelta: la vuelta 2 se cierra al salir de la esquina */
    /* Parqueo en paralelo (el cajon esta en la seccion de arranque, junto a la pared exterior).
     * Se desactiva por defecto: el robot no puede saber si la pista tiene cajon. */
    bool parking_enabled;           /* al terminar las vueltas, buscar el cajon y aparcar */
    bool park_ackermann;            /* maniobras con avance y reversa, sin pivote */
    bool start_in_lot;              /* arranca dentro del cajon: primero sale a la pista */
    float park_speed_cm_s;          /* avance/retroceso en las maniobras */
    float park_rot_dps;             /* giro en el sitio, maximo */
    float park_margin_cm;           /* holgura con topes y pared al planear */
    float park_lot_length_cm;       /* 0 = 1,5 x el largo del robot */
    float park_lane_w_cm;           /* al buscar el cajon, a que distancia de la pared exterior ir */
    float park_search_speed_cm_s;
    float park_search_max_cm;       /* si no lo encuentra en este recorrido, se detiene como sin parqueo */
    /* Demostracion: arrancar ya en el carril con las vueltas dadas y ponerse a buscar el cajon.
     * -1 = sentido antihorario, +1 = horario, 0 = ronda normal. Hace falta parking_enabled. */
    int8_t park_demo_dir;
    float start_lot_w_cm;           /* donde arranca el eje motriz respecto a la pared (cerca del lado del carril) */
    uint32_t sensor_timeout_us;     /* sin escaneo mas reciente que esto: parar */
    uint32_t gyro_min_samples;      /* muestras para el sesgo antes de arrancar */
} wro_controller_config_t;

void wro_controller_config_default(wro_controller_config_t *cfg);
/* Configuracion como texto "clave valor" (la del planificador lleva el prefijo
 * "planner."). parse_text aplica todo o nada y acota los valores. */
bool wro_controller_config_set(wro_controller_config_t *cfg, const char *key, const char *value);
size_t wro_controller_config_format(const wro_controller_config_t *cfg, char *out, size_t capacity);
bool wro_controller_config_parse_text(wro_controller_config_t *cfg, const char *text,
                                      char *error, size_t error_size);

typedef struct {
    uint8_t action;                 /* local_action_t */
    float steer_deg;                /* volante: derecha positiva */
    float speed_cm_s;               /* negativo = marcha atras */
    wro_phase_t phase;
    bool done;                      /* mision terminada: motor parado */
    int corners;                    /* esquinas completadas */
    int laps;                       /* vueltas completadas */
    int direction;                  /* +1 horario, -1 antihorario, 0 aun no se sabe */
    bool reversed;                  /* se dio media vuelta para la ultima vuelta */
    float yaw_deg, course_deg;      /* rumbo del giroscopio y del recorrido */
    float front_cm, left_cm, right_cm;
    float lane_cm;
    float pass_cm;
    float elapsed_s;
    uint8_t last_sign_color;        /* ultimo letrero rebasado */
    bool relaxed;                   /* regla del lado de paso suspendida por atasco */
    /* Giro del cuerpo en grados/s, horario positivo. Es la curvatura que pide el planificador
     * (velocidad x tan(volante) / batalla), o un giro en el sitio con velocidad 0 si
     * explicit_yaw_rate. Un robot diferencial reparte velocidad y giro entre sus dos ruedas. */
    float yaw_rate_dps;
    bool explicit_yaw_rate;
} wro_command_t;

typedef struct {
    wro_controller_config_t cfg;
    imu_yaw_t yaw;
    local_obstacle_map_t map;
    local_planner_state_t planner_state;
    local_planner_debug_t debug;

    camera_table_t camera;
    bool camera_seen;

    wro_phase_t phase;
    int direction;
    int corners_done;
    bool corner_active;
    bool pivoting;
    int sidestep_phase;             /* 0 no, 1 girar hacia fuera, 2 avanzar de lado, 3 girar de vuelta */
    int sidestep_side;              /* +1 hacia la derecha del recorrido, -1 hacia la izquierda */
    int sidestep_sign;              /* letrero (indice en signs[]) que lo motiva */
    float sidestep_need_cm, sidestep_slid_cm, sidestep_wait_s, sidestep_cooldown_s;
    float sidestep_phase_s;         /* tiempo en la fase actual */
    float pivot_wait_s;
    int corner_side;                /* hacia donde giro la esquina en curso (+1 derecha, -1 izquierda) */
    bool aligned;
    bool turnaround_decided, reversed;
    bool uturn_pending;             /* hay que dar media vuelta en cuanto el robot este en la recta de salida */
    float uturn_travel_cm;
    bool uturn_wall_seen;
    float course_deg;
    float travel_cm;                /* velocidad ordenada x tiempo desde el ultimo evento */
    float finish_travel_cm;
    bool finish_seen_wall;
    uint8_t last_sign_color;
    /* Letreros vistos, seguidos con giroscopio y velocidad ordenada x tiempo (no hay
     * odometria): el color solo se ve de frente, y el letrero se rebasa cuando ya
     * esta fuera del campo de la camara. */
    struct { float x_cm, forward_cm, stale_cm; uint8_t color; bool valid; } signs[12];
    /* Memoria por seccion (los letreros no cambian entre vueltas, reglamento 2026): donde se vio cada uno con color
     * (a lo largo de la recta desde la ultima esquina, y a la derecha del centro del carril). En las vueltas
     * siguientes se reinyecta: un letrero fuera del campo de la camara conserva su cortina. */
    struct { float along_cm, lat_cm; uint8_t color; bool used; } lap_mem[4][6];
    /* Letreros conocidos, en el campo. (x, z) es donde el LiDAR los ve (sirve para reinyectarlos: se compara con lo que
     * el LiDAR vuelve a ver); (fx, fz) es su posicion real corregida (centro del letrero, pose a la hora del mapa), la que usa
     * la linea de carrera. */
    struct { float x_cm, z_cm, fx_cm, fz_cm; uint8_t color, hits; bool used; } world_signs[16];
    size_t live_colored;            /* objetos con color de este escaneo que vienen de la camara (no de la memoria) */
    arena_loc_t arena;              /* pose en el campo (odometria corregida con el LiDAR) */
    float arena_yaw_off;            /* rumbo del campo = rumbo del giroscopio + esto */
    float map_lead_cm, map_lead_deg;   /* el mapa del planificador esta adelantado a la hora en que la orden hace efecto: avance y giro que lleva sobre la pose del campo */
    int arena_tries;
    odom_t odom;                    /* odometria de los encoders con el giroscopio */
    bool odom_have, odom_in_use;    /* hay muestras / este escaneo se uso la odometria */
    uint64_t odom_last_us;
    float odom_accum_cm;            /* avance medido desde el ultimo escaneo */
    float scan_ds_cm;               /* avance de este ciclo (medido, o velocidad ordenada x tiempo) */
    float lap_off_cm, lap_prev_travel_cm;   /* correccion del avance de la seccion con los letreros reconocidos */
    float hug_cmd_cm, hug_target_cm, hug_hold_s;   /* desplazamiento pedido al planificador, objetivo y espera por pilar desconocido */
    bool hug_active;
    race_line_t race;               /* linea de carrera de la vuelta (valida tras calcularla) */
    bool race_tried, race_active, race_corner_clear;
    int race_slips0;                /* patinajes contados cuando se calculo la linea */
    float race_grip;                /* 1 = agarre como en la primera vuelta; baja con cada patinaje nuevo */
    int race_idx;                   /* estacion mas cercana del robot */
    float race_v_cmd, race_v_line, race_lat_cm, race_dist_cm, race_hold_s;
    float lap_dbg[4];               /* telemetria: recuerdos inyectados, anclados al LiDAR, vistos en vivo, ultimo error (cm) */
    float prev_yaw_deg;
    bool prev_yaw_valid;
    uint64_t start_us, last_scan_us, last_imu_us;
    float last_speed_cm_s;
    /* Parqueo. */
    int park_field_goal;            /* 0 sin campo, 1 entrar, 2 salir (park_planner.h) */
    park_pose_t park_pose;          /* pose estimada en el marco del cajon */
    float park_course_deg;          /* rumbo del giroscopio que deja el morro paralelo a la pared */
    park_move_t park_prev;
    int park_lost, park_goal_hits;
    float park_arc_remaining_cm, park_arc_steer_deg;
    int park_arc_direction;
    unsigned park_arc_index, park_arc_count;
    int8_t park_arc_directions[PARK_ACK_MAX_STEPS], park_arc_steering[PARK_ACK_MAX_STEPS];
    bool park_init, parked;
    float park_search_cm;
    /* Entrada desde donde acaba la salida (la ronda empieza saliendo del cajon, asi que esa pose
     * se conoce): 0 sin empezar, 1 yendo por el carril, 2 un solo plan, 3 tanteo de siempre. */
    int park_stage;
    park_pose_t park_exit_pose;     /* donde acabo la salida, medido en esta ronda */
    bool park_exit_valid;
    park_pose_t park_plan_pose;     /* pose que toca segun el plan (avance a ciegas de sus arcos) */
    float park_arc_theta0_deg, park_arc_dtheta_deg;   /* rumbo al empezar el arco y giro que le toca */
    /* Giro en el sitio del parqueo: con el retardo del motor, repetir la consigna cada
     * barrido hace oscilar el rumbo en vez de asentarlo. Se gira un barrido y se deja
     * asentar el siguiente. */
    int park_rot_hold;
    /* Diagnostico del ultimo plan de la maniobra Ackermann (ver core_park_dbg_ptr). */
    int park_plan_found, park_plan_expanded, park_plan_at_goal, park_plan_count, park_plan_step;
    int park_rot_sign;              /* sentido del ultimo giro (+1 contra reloj, -1 a favor) */
    float net_cm, progress_best_cm, stuck_s, relax_from_cm;   /* avance neto (velocidad ordenada x tiempo) */
    bool relaxed;

    /* Media vuelta (K-turn). */
    float uturn_target_deg;
    int uturn_dir;
    bool uturn_reversing;
    float uturn_leg_cm;
    int uturn_legs;
} wro_controller_t;

void wro_controller_init(wro_controller_t *c, const wro_controller_config_t *cfg);
/* Sustituye la configuracion en caliente (se acota). Lo que no cambia es el
 * estado de la mision. */
void wro_controller_set_config(wro_controller_t *c, const wro_controller_config_t *cfg);

/* Muestras del IMU, a su propia frecuencia. gyro_dps: horario positivo. */
void wro_controller_imu(wro_controller_t *c, uint64_t t_us, float gyro_dps, float mag_heading_deg);
/* Angulos acumulados de las ruedas (grados) a la hora t_us: lo que entregan los encoders de las ruedas. La
 * velocidad ordenada deja de ser la fuente del avance en cuanto llegan muestras. */
void wro_controller_encoders(wro_controller_t *c, uint64_t t_us, float left_deg, float right_deg);
void wro_controller_axle_encoder(wro_controller_t *c, uint64_t t_us, float axle_deg);
/* Ultima tabla del S3; se copia. */
void wro_controller_camera(wro_controller_t *c, const camera_table_t *table);
/* Boton de arranque. Cierra la calibracion del giroscopio. Devuelve false si no
 * hubo suficientes muestras de giroscopio quieto (el sesgo queda en 0). */
bool wro_controller_start(wro_controller_t *c, uint64_t t_us);
/* Un barrido completo del LiDAR (en orden de barrido, angulos derecha-positivo,
 * distancias desde el sensor). Rellena la orden. */
void wro_controller_scan(wro_controller_t *c, const local_lidar_point_t *points, size_t count,
                         uint64_t t_us, wro_command_t *out);
/* Perro guardian: si no llegan escaneos, la orden pasa a parar (devuelve true). */
bool wro_controller_watchdog(wro_controller_t *c, uint64_t now_us, wro_command_t *out);

#endif

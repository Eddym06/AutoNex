/* Intelligent Trajectory (IT): trayectoria y velocidad de una vuelta, DESCUBIERTAS por optimizacion a partir del mapa.
 *
 * No hay una linea de carrera dada: el planificador recibe el espacio libre (paredes del campo y letreros del mapa con su
 * color) y la fisica del vehiculo, y busca el desplazamiento lateral de cada estacion que minimiza el tiempo de vuelta:
 *   - Corredor libre por estacion: dentro de las paredes, fuera de cada letrero y del lado que su color permite.
 *   - Fisica: el coche solo agarra lo que le toca de peso (traccion trasera: el eje delantero va ligero y subvira; solo el
 *     trasero frena), con circulo de friccion entre empujar y sujetarse de lado, traslado de peso, curvatura maxima del
 *     volante, velocidad de giro del servo (MG90S) y tope por percepcion.
 *   - Margenes que dependen de la velocidad: error de pose + error de seguimiento + barrido del morro en curva. A mas
 *     velocidad el corredor se estrecha y el propio perfil de velocidad baja donde no cabe (letrero pegado a la pared).
 *   - Busqueda: relajacion de curvatura minima como punto de partida y despues busqueda local con "jorobas" suaves
 *     evaluando el tiempo real (perfil adelante-atras con las limitaciones anteriores). 3 pasadas: la velocidad cambia el
 *     corredor, que cambia la trayectoria.
 *   - Agarre aprendido: `grip` (0..1) escala mu. Cada vuelta se mide el error de seguimiento y los contactos y `it_adapt_grip`
 *     lo sube si sobra margen o lo baja si no. El limite lo encuentra el coche.
 * Seguidor (it_track): persecucion pura con adelanto por el retardo del servo, y velocidad del perfil recortada si el
 * error lateral se come el margen.
 *
 * Marco del campo: origen en el centro, x al este, z al norte, rumbo horario desde el norte (arena_localizer.h). La pose es el
 * eje trasero (motriz). C99 puro, sin asignacion dinamica. */
#ifndef IT_PLAN_H
#define IT_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#define IT_MAX_N 256
#define IT_MAX_OBS 24

typedef struct {
    float x_cm, z_cm;
    uint8_t color;               /* 1 rojo, 2 verde, 0 sin regla de lado */
} it_obstacle_t;

typedef struct {
    /* Campo. */
    float outer_half_cm, inner_half_cm, corner_radius_cm;
    /* Vehiculo (la pose es el eje trasero). */
    float wheelbase_cm, half_width_cm, front_ext_cm, rear_ext_cm;   /* extensiones del chasis desde el eje trasero */
    float steer_max_deg, steer_rate_dps, lag_s;       /* lag: orden -> el volante responde (retardo + constante del servo + calculo) */
    float steer_use, rate_use;   /* fraccion del volante y de la velocidad del servo que usa el plan (el resto es reserva del seguidor) */
    /* Agarre y traccion. */
    float mu, rear_weight, cg_height_cm;
    float grip;                  /* 0..1: fraccion de mu que se usa (se aprende entre vueltas) */
    float safety;                /* fraccion del agarre disponible que el perfil se concede (0,85) */
    float a_motor_cm_s2;         /* aceleracion que da el motor (medir con MIC) */
    float v_max_cm_s, v_min_cm_s;
    /* Margenes. */
    float wall_margin_cm, obs_margin_cm, obs_half_cm;
    float pose_err_cm;           /* error de pose (0,8 en el simulador) */
    float track_err_cm;          /* error de seguimiento a 100 cm/s; crece con el cuadrado de la velocidad */
    float curtain_cm, approach_cm;
    int8_t side_red, side_green; /* +1 derecha, -1 izquierda */
    /* Busqueda. */
    float station_cm;
    int passes, sweeps, relax_iterations;
} it_config_t;

typedef struct {
    it_config_t cfg;
    bool valid;
    uint8_t why;                 /* si no es valido: 1 volante no llega, 2 salto lateral, 4 la huella del chasis toca algo */
    int n, direction;           /* direction: +1 horario, -1 antihorario */
    float cx[IT_MAX_N], cz[IT_MAX_N], nx[IT_MAX_N], nz[IT_MAX_N];     /* linea central y normal a la derecha */
    float lo0[IT_MAX_N], hi0[IT_MAX_N];                                /* tramo libre con los margenes estaticos */
    float lo[IT_MAX_N], hi[IT_MAX_N];                                  /* idem recortado por los margenes dinamicos */
    float d[IT_MAX_N];                                                 /* desplazamiento elegido (derecha positivo) */
    float x[IT_MAX_N], z[IT_MAX_N], s[IT_MAX_N], h[IT_MAX_N];
    float kappa[IT_MAX_N];       /* curvatura sin signo (1/cm) */
    float kappa_s[IT_MAX_N];     /* con signo: derecha positiva */
    float heading_deg[IT_MAX_N];
    float v[IT_MAX_N];           /* velocidad objetivo (cm/s) */
    float vcap[IT_MAX_N];        /* tope de velocidad por estacion a que el hueco no deja margen mayor (0 = sin tope) */
    float room[IT_MAX_N];        /* holgura lateral hasta el contacto, sobre los margenes estaticos (cm) */
    float margin[IT_MAX_N];      /* margen dinamico aplicado en la estacion */
    float length_cm, time_s, time_center_s, violation;
    float clip;                  /* penetracion total (cm) de la huella del chasis en la trayectoria evaluada */
    float room_min;              /* menor holgura lateral del plan sobre el corredor de punto (cm) */
    int obstacles, evals;
    it_obstacle_t obs[IT_MAX_OBS];
    float obs_rx[IT_MAX_OBS], obs_rz[IT_MAX_OBS];   /* derecha del avance en cada obstaculo */
} it_plan_t;

typedef struct {
    int idx;
    float e_cm, psi_err_deg, steer_deg, v_cmd_cm_s, v_plan_cm_s, room_cm;
    bool on_path;
} it_tracker_t;

typedef struct {
    float max_err_cm;            /* mayor error lateral de la vuelta */
    float min_room_cm;           /* menor holgura en las estaciones donde estuvo (room - |error|) */
    int contacts;
    float max_slip_deg;          /* deriva trasera maxima */
    int traction_events;
} it_lap_stats_t;

void it_config_default(it_config_t *c);
bool it_config_sanitize(it_config_t *c);
bool it_plan_build(it_plan_t *p, const it_config_t *cfg, int direction, const it_obstacle_t *obs, int count);

void it_tracker_reset(it_tracker_t *t);
/* Pose del eje trasero (cm, grados horario desde el norte) y velocidad medida. Devuelve false si no hay plan o el robot esta
 * lejos de el (el llamador usa su respaldo). */
bool it_track(it_tracker_t *t, const it_plan_t *p, float x_cm, float z_cm, float yaw_deg, float v_cm_s,
              float *steer_deg, float *speed_cm_s);
/* Comprueba la trayectoria final con la huella del chasis y su propio rumbo (devuelve la menor holgura al letrero mas cercano en
 * *min_obstacle_gap_cm si no es NULL). */
bool it_path_clear(const it_plan_t *p, float *min_obstacle_gap_cm);
int it_locate(const it_plan_t *p, float x_cm, float z_cm, int hint, float window_cm, float *lateral_cm, float *dist_cm);
float it_speed_at(const it_plan_t *p, int idx, float ahead_cm);
/* Nuevo `grip` a partir de lo ocurrido en la vuelta. */
float it_adapt_grip(const it_config_t *c, const it_lap_stats_t *s, float grip_max);

#endif

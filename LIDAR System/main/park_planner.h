#ifndef PARK_PLANNER_H
#define PARK_PLANNER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Maniobras en el cajon de parqueo (entrar en paralelo, o salir de el al arrancar) para un
 * robot diferencial que puede avanzar, retroceder y girar en el sitio.
 *
 * Marco del cajon: u a lo largo de la pared exterior, en el sentido de la marcha, con el
 * origen en el centro del cajon; w desde la pared hacia el carril (0 = pared); theta
 * contra reloj desde +u, en grados. La posicion es la del EJE MOTRIZ. El cajon ocupa
 * u en [-L/2, L/2] y w en [0, 20]; dos topes de 2 cm cierran sus extremos.
 *
 * El espacio de poses (u, w, rumbo) se discretiza en una retícula entera: avanzar o
 * retroceder un vector (dx, dy) de celdas a lo largo de un rumbo de la lista, o girar al
 * rumbo vecino. Un Dijkstra INVERSO desde la meta da el coste a la meta de cada pose
 * libre (con la holgura pedida); la politica elige en cada barrido del LiDAR el movimiento
 * que mas lo baja, desde la pose medida: no sigue un camino fijo, asi que se recupera de
 * cualquier desvio. No hay odometria: la pose sale del LiDAR (park_localize.h).
 *
 * Memoria: unos 0,9 MB estaticos (campo de costes, libres y cola). En el P4 va en PSRAM. */

#define PARK_NU 80
#define PARK_NW 50
#define PARK_CELL_CM 1.0f
#define PARK_U0_CM (-40.0f)
#define PARK_MAX_HEADINGS 32

typedef struct {
    float robot_width_cm, robot_length_cm;
    float rear_overhang_cm;        /* cuanto sobresale el chasis por detras del eje motriz */
    float lot_length_cm;           /* 0 = 1,5 x el largo del robot (reglamento 2026) */
    float lot_width_cm;            /* 20 */
    float limiter_thickness_cm;    /* 2 */
    float margin_cm;               /* holgura minima con topes y pared */
    float soft_margin_cm;          /* por debajo de esta holgura el camino se encarece: prefiere pasar mas lejos */
    float goal_theta_tol_deg;      /* en paralelo: tolerancia del rumbo de la meta */
    float goal_slack_cm;           /* la meta de entrar exige el chasis a esta distancia del borde del cajon */
    float exit_clear_cm;           /* salida: la meta empieza a esta distancia de la pared */
} park_params_t;

typedef enum { PARK_GOAL_IN = 0, PARK_GOAL_OUT = 1 } park_goal_t;

typedef enum {
    PARK_MOVE_NONE = 0,   /* en la meta, o sin camino */
    PARK_MOVE_FORWARD,
    PARK_MOVE_BACK,
    PARK_MOVE_ROT_POS,    /* gira hacia theta mayor (contra reloj) */
    PARK_MOVE_ROT_NEG
} park_move_t;

typedef struct { float u_cm, w_cm, theta_deg; } park_pose_t;

typedef struct {
    park_move_t move;
    float cost;            /* coste a la meta desde la pose; < 0 si no hay camino */
    float target_theta_deg;/* rumbo al que hay que estar para el movimiento (el del bin, o el vecino) */
    bool at_goal;
} park_decision_t;

void park_params_default(park_params_t *p, float robot_width_cm, float robot_length_cm, float rear_overhang_cm);
/* Construye el campo para una meta. Falsa si los parametros son imposibles o no hay meta libre. */
bool park_field_build(const park_params_t *p, park_goal_t goal);
bool park_field_ready(void);
park_goal_t park_field_goal(void);
const park_params_t *park_field_params(void);

/* Decide el movimiento desde una pose. `previous` aporta histeresis. */
park_decision_t park_field_decide(const park_pose_t *pose, park_move_t previous);
/* Coste a la meta de la celda mas cercana a la pose (-1 = sin camino o fuera del campo). */
float park_field_cost(const park_pose_t *pose);
/* Pose libre de choque (con holgura) en el campo? */
bool park_field_free(const park_pose_t *pose);
/* Geometria continua, sin depender de la rejilla del planificador diferencial. */
bool park_pose_free(const park_params_t *params, const park_pose_t *pose);

size_t park_heading_count(void);
float park_heading_deg(size_t k);
/* Vector (cm) de un avance de la retícula a lo largo del rumbo k (el retroceso es el opuesto). */
void park_heading_step(size_t k, float *du_cm, float *dw_cm);
/* Indice del rumbo de la retícula mas cercano a un angulo. */
size_t park_heading_nearest(float theta_deg);

/* Geometria compartida con el localizador: las esquinas del chasis en el marco del cajon. */
void park_body_corners(const park_params_t *p, const park_pose_t *pose, float out[4][2]);
/* Distancia con signo desde un punto del marco del cajon a la superficie del modelo
 * (pared + los dos topes); siempre >= 0. */
float park_model_distance(const park_params_t *p, float u, float w);

#endif

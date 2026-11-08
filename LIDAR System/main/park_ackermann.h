#ifndef PARK_ACKERMANN_H
#define PARK_ACKERMANN_H
#include "park_planner.h"
#define PARK_ACK_MAX_STEPS 128

typedef struct {
    float wheelbase_cm;
    float steer_max_deg;
    float step_cm;
    int previous_direction;       /* penaliza inversiones que causan oscilaciones */
} park_ackermann_config_t;

typedef struct {
    bool found, at_goal;
    int direction;             /* +1 avance, -1 reversa, 0 parado */
    float steer_deg;           /* positivo: theta canonico aumenta en avance */
    unsigned expanded;
    unsigned count;
    int8_t directions[PARK_ACK_MAX_STEPS], steering[PARK_ACK_MAX_STEPS];
} park_ackermann_result_t;

/* Replanificación en poses discretas; nunca produce giro en el sitio. */
park_ackermann_result_t park_ackermann_plan(const park_params_t *params, const park_ackermann_config_t *cfg,
                                            const park_pose_t *start, park_goal_t goal);
/* Entrada en una pasada: una S de dos arcos (reversa larga + avance de enderezado) hasta quedar
 * centrada y paralela dentro del cajon. Falsa si no existe con esta geometria. */
bool park_ackermann_entry_s(const park_params_t *params, const park_ackermann_config_t *cfg,
                            const park_pose_t *start, park_ackermann_result_t *out);
bool park_ackermann_arc_free(const park_params_t *params, const park_ackermann_config_t *cfg,
                            const park_pose_t *start, int direction, int steering);
#endif

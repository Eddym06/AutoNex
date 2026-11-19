#ifndef TRACE_PARKING_H
#define TRACE_PARKING_H
#include "trace_map.h"

/* Prototipo conservador: el mapa y la pose son estimados, nunca verdad del
 * simulador. La cámara aporta una sola detección angular, no ambos topes. */
typedef struct {
    arena_pose_t camera_pose;
    float camera_forward_cm, camera_lateral_cm;
    float min_bearing_deg, max_bearing_deg;
    float distance_cm;
    float spacing_cm;             /* distancia esperada entre centros de topes */
} trace_parking_observation_t;
typedef enum { TRACE_PARK_NO_EVIDENCE=0, TRACE_PARK_CANDIDATE=1, TRACE_PARK_AMBIGUOUS=2 } trace_parking_status_t;
typedef struct {
    trace_parking_status_t status;
    float wall_x[2], wall_z[2]; /* intersecciones estimadas, no extremos medidos */
    float spacing_cm;
    unsigned wall_cells, alternatives;
} trace_parking_result_t;

/* Supone paredes paralelas a los ejes del campo localizado. No controla el
 * vehículo ni certifica que la hipótesis sea segura para maniobrar. */
trace_parking_result_t trace_parking_estimate(const trace_map_t *map, const trace_parking_observation_t *camera);
/* Compara ambas asignaciones de extremos con un ancla fija; no confirma que el ancla sea correcta. */
bool trace_parking_consistent(const trace_parking_result_t *a, const trace_parking_result_t *b, float tolerance_cm);
#endif

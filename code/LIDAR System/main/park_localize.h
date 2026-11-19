#ifndef PARK_LOCALIZE_H
#define PARK_LOCALIZE_H

#include "local_obstacle_map.h"
#include "park_planner.h"

/* Localiza el robot respecto al cajon de parqueo con el barrido del LiDAR: casa los puntos
 * con el MODELO conocido (pared exterior y los dos topes de 20 x 2 cm) buscando la pose
 * (u, w, rumbo) que deja mas puntos sobre sus superficies. No necesita odometria ni ve
 * la verdad: con topes parcialmente tapados casa lo que ve; si solo ve la pared, la
 * posicion a lo largo de ella (u) no se observa y manda la estima anterior.
 *
 * `mirror`: +1 si la pared exterior queda a la DERECHA del robot (ronda antihoraria), -1 si
 * a la izquierda. La pose sale siempre en el marco canonico de park_planner.h. */

typedef struct {
    bool valid;
    park_pose_t pose;
    float score;            /* puntos sobre el modelo (ponderados) */
    int wall_hits;
    int rear_hits, front_hits;   /* puntos sobre cada tope */
} park_fix_t;

typedef struct {
    const park_params_t *params;
    float lidar_forward_cm;      /* montaje del LiDAR respecto al eje motriz */
    int mirror;
} park_loc_t;

/* Busqueda sin conocimiento previo, para encontrar el cajon desde el carril, viniendo de
 * detras (u < u_max). Falla si no hay pared y al menos un tope con puntos suficientes. */
bool park_localize_acquire(const park_loc_t *loc, const local_lidar_point_t *points, size_t count,
                           float u_max_cm, park_fix_t *fix);
/* Seguimiento alrededor de una pose previa (la anterior mas el movimiento). */
bool park_localize_track(const park_loc_t *loc, const local_lidar_point_t *points, size_t count,
                         const park_pose_t *prior, park_fix_t *fix);

#endif

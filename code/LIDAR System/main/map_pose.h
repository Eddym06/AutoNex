#ifndef MAP_POSE_H
#define MAP_POSE_H

#include <stddef.h>
#include <stdint.h>

#include "local_obstacle_map.h"

/* Pose del robot para construir un mapa del entorno (independiente de la mision).
 *
 *  - Rumbo: integra el giroscopio (grados/s, horario positivo). 0 = el norte, que es hacia donde miraba el robot al
 *    arrancar o al llamar a map_pose_reset. Crece en el sentido de las agujas del reloj, como una brujula.
 *  - Posicion (este, norte) en cm respecto al punto de arranque: cada barrido del LiDAR se alinea con el anterior
 *    (ICP punto-a-linea solo de traslacion, el giro ya lo da el giroscopio). La estimacion inicial puede venir de los encoders
 *    (avance a lo largo del rumbo). Sin rasgos en el entorno no se mueve: una zona muerta evita la deriva en reposo. */
#define MAP_POSE_MAX_POINTS 256
#define MAP_POSE_BINS 360            /* perfil de distancias por grado, para detectar reposo y giro */

typedef struct {
    float heading_deg;               /* rumbo en [-180, 180): 0 = norte, horario positivo */
    float e_cm, n_cm;                /* este y norte */
    uint64_t last_gyro_us;
    int have_prev;
    int prev_count;
    float prev_e[MAP_POSE_MAX_POINTS], prev_n[MAP_POSE_MAX_POINTS];
    float prev_ne[MAP_POSE_MAX_POINTS], prev_nn[MAP_POSE_MAX_POINTS];   /* normal de cada punto (0,0 = sin normal) */
    /* Calibracion del giroscopio con el LiDAR: en reposo (el perfil de distancias no cambia) cualquier lectura es
     * sesgo y se descuenta; al girar a mano se compara el giro del LiDAR con el del giroscopio y, si van al reves,
     * se pide invertir el signo del giroscopio (flip_request). */
    float bias_dps;                  /* sesgo extra aprendido (se resta a cada muestra) */
    float ref_prof[MAP_POSE_BINS], rot_prof[MAP_POSE_BINS];
    int ref_valid, rot_valid;
    float ref_dps_sum, ref_time_s;   /* suma de lecturas crudas y tiempo desde el perfil de referencia de reposo */
    float rot_heading_deg, rot_time_s;
    int votes_ok, votes_flip;
    int flip_request;                /* 1 = el giroscopio gira al reves: invertir su signo (lo atiende quien lo llamo) */
    int bias_updates;
    int last_matches;                /* diagnostico: pares usados en el ultimo alineado (0 = no se pudo alinear) */
    float last_shift_cm;             /* desplazamiento que corrigio el ultimo alineado */
} map_pose_t;

void map_pose_reset(map_pose_t *m);
/* Una muestra del giroscopio (grados/s, horario positivo) a la hora t_us. */
void map_pose_gyro(map_pose_t *m, float dps, uint64_t t_us);
/* Un barrido completo. odom_forward_cm: avance medido por los encoders desde el barrido anterior a lo largo del
 * rumbo (0 si no hay encoders fiables, p. ej. con el robot movido a mano). Actualiza la posicion. */
void map_pose_scan(map_pose_t *m, const local_lidar_point_t *points, size_t count, float odom_forward_cm);

#endif

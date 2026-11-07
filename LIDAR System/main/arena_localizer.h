/* Localizacion del robot en el campo con el LiDAR, sobre un modelo de las paredes.
 *
 * La pista WRO es un cuadrado exterior y un bloque cuadrado interior, ambos con las paredes alineadas con
 * los ejes: es un mapa que el robot CONOCE. La odometria (encoders + giroscopio) da una pose que deriva
 * entre el 0,2 % y el 2 % del recorrido; este modulo la corrige casando los puntos del LiDAR con las paredes
 * (minimos cuadrados robustos punto-recta, tolerante a letreros y topes, que quedan como valores atipicos).
 *
 * Marco del campo: origen en el centro, x al este, z al norte, rumbo horario desde el norte (como el
 * simulador y el giroscopio). La seccion de salida es la de abajo (z < 0): el robot arranca en ella mirando al
 * este (antihorario) o al oeste (horario); la pose inicial se busca casando el primer barrido.
 *
 * Todo lo que describe el campo (mitad del cuadrado exterior, mitad del bloque interior) es configuracion. */
#ifndef ARENA_LOCALIZER_H
#define ARENA_LOCALIZER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    float outer_half_cm;     /* mitad del cuadrado exterior (pared exterior): 150 con carril de 100 */
    float inner_half_cm;     /* mitad del bloque interior: 50 */
    float tol_cm;            /* un punto a menos de esto de una pared cuenta como suyo */
    float gain;              /* fraccion de la correccion que se aplica por barrido (0..1) */
    int min_support;         /* puntos que respaldan un eje para corregirlo */
    float max_shift_cm;      /* una correccion mayor que esto se descarta */
    float max_rot_deg;
    float min_range_cm;      /* se ignoran los retornos mas cerca que esto (el propio robot) */
    float max_range_cm;
    float init_min_score;    /* fraccion minima de puntos que casan para aceptar la pose inicial */
} arena_config_t;

typedef struct {
    float x_cm, z_cm;        /* eje de las ruedas en el campo */
    float theta_deg;         /* rumbo horario desde el norte, en (-180, 180] */
} arena_pose_t;

typedef struct {
    arena_config_t cfg;
    arena_pose_t pose;
    bool valid;              /* hay pose (se ha adquirido) */
    float sup_x, sup_z, sup_rot;   /* respaldo del ultimo ajuste (puntos con peso) por eje */
    int inliers, used;       /* puntos que casaron / puntos considerados en el ultimo barrido */
    float rms_cm;            /* residuo de los que casaron */
    int updates, rejected;
    float last_dx, last_dz, last_dtheta_deg;
} arena_loc_t;

typedef struct {
    float angle_deg;         /* derecha positiva desde el frente del sensor */
    float distance_cm;
    bool valid;
} arena_point_t;

void arena_config_default(arena_config_t *cfg);
bool arena_config_sanitize(arena_config_t *cfg);
void arena_init(arena_loc_t *a, const arena_config_t *cfg);

/* Pose inicial por busqueda: prueba rumbos este y oeste en la seccion de abajo y se queda con la que mas puntos
 * casa. `lidar_forward_cm` es lo que el sensor esta delante del eje. Devuelve false si ninguna casa lo bastante. */
bool arena_acquire(arena_loc_t *a, const arena_point_t *pts, size_t n, float lidar_forward_cm);

/* Avance del eje (cm, con signo) y rumbo ABSOLUTO del campo que se estima ahora (grados): mueve la pose. */
void arena_predict(arena_loc_t *a, float ds_cm, float theta_deg);

/* Corrige la pose con un barrido. Devuelve true si se aplico una correccion. */
bool arena_update(arena_loc_t *a, const arena_point_t *pts, size_t n, float lidar_forward_cm);

/* Un punto del robot (x derecha, f delante del eje) en el campo, con la pose dada. */
void arena_to_world(const arena_pose_t *p, float qx, float qf, float *wx, float *wz);
/* Al reves: un punto del campo en el marco del robot. */
void arena_to_robot(const arena_pose_t *p, float wx, float wz, float *qx, float *qf);

#endif

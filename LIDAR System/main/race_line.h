/* Linea de carrera: la trayectoria y el perfil de velocidad de una vuelta, calculados sobre el mapa del campo una vez
 * conocidos los letreros (despues de la primera vuelta).
 *
 * Los letreros no se mueven y el robot conoce el campo (paredes exteriores e interior), asi que el mejor recorrido se
 * puede calcular en vez de improvisarlo en cada barrido:
 *   1. Se parte de una linea central redondeada del carril, con una estacion cada `station_cm`.
 *   2. En cada estacion se mide, a lo largo de la normal, el tramo libre: dentro de las paredes (con holgura), fuera
 *      del disco de cada letrero y del lado permitido por su color (la "cortina": el lado prohibido queda cerrado).
 *   3. Se desplaza cada estacion dentro de su tramo libre minimizando una mezcla de longitud y curvatura (minimos
 *      cuadrados con cotas, resuelto por sobrerrelajacion proyectada). Se prueban varias mezclas y se queda la que da
 *      la vuelta mas rapida con el perfil de velocidad del paso siguiente.
 *   4. Perfil de velocidad: v <= sqrt(a_lat / curvatura), tope junto a letreros y esquinas, y rampas de aceleracion y
 *      frenada (hacia delante y hacia atras, dando la vuelta completa).
 *
 * Es una REFERENCIA: el planificador reactivo (LiDAR + camara) sigue mandando. Un letrero que no se vio en la primera
 * vuelta no esta en la linea, pero el robot lo esquiva igual; el controlador baja a la velocidad base si ve algo que
 * la linea no esperaba.
 *
 * Marco del campo: origen en el centro, x al este, z al norte, rumbo horario desde el norte (como arena_localizer.h). */
#ifndef RACE_LINE_H
#define RACE_LINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RACE_MAX_STATIONS 192
#define RACE_MAX_SIGNS 16
#define RACE_CANDIDATES 6

typedef struct {
    float x_cm, z_cm;
    uint8_t color;               /* 1 rojo, 2 verde: los demas no llevan regla de lado */
} race_sign_t;

typedef struct {
    /* Campo y chasis. */
    float outer_half_cm;         /* mitad del cuadrado exterior (150) */
    float inner_half_cm;         /* mitad del bloque interior (50) */
    float half_width_cm;         /* mitad del ancho del chasis (6,5) */
    float wall_margin_cm;        /* holgura de la linea respecto a las paredes, sobre el chasis */
    float sign_margin_cm;        /* idem respecto a un letrero */
    float sign_half_cm;          /* mitad del lado del letrero (2,5) */
    float curtain_cm;            /* longitud de la cortina desde el letrero hacia el lado prohibido */
    float approach_cm;           /* la cortina se adelanta tanto antes del letrero: distancia para cambiar de lado */
    int8_t pass_side_red, pass_side_green;   /* +1 pasar por la derecha, -1 izquierda, 0 sin regla */
    float station_cm;            /* separacion nominal entre estaciones */
    float corner_radius_cm;      /* redondeo de la linea central en las esquinas */
    /* La linea no se separa del centro del carril mas de esto (un robot que sigue una referencia lejana del centro tarda
     * en llegar a ella); un letrero puede obligar a mas. */
    float max_offset_cm;
    /* Velocidad. */
    float v_max_cm_s;            /* tope en recta libre */
    float v_sign_cm_s;           /* tope en la zona de un letrero */
    float v_corner_cm_s;         /* tope en la zona de una esquina */
    float a_lat_cm_s2;           /* aceleracion lateral admisible */
    float a_acc_cm_s2, a_dec_cm_s2;
    float sign_before_cm, sign_after_cm;      /* zona de un letrero a lo largo de la linea */
    float corner_before_cm, corner_after_cm;  /* zona de una esquina */
    int iterations;              /* barridos maximos de la sobrerrelajacion */
} race_config_t;

typedef struct {
    race_config_t cfg;
    bool valid;
    int n;                       /* estaciones */
    int direction;               /* +1 horario, -1 antihorario */
    float cx[RACE_MAX_STATIONS], cz[RACE_MAX_STATIONS];   /* linea central */
    float nx[RACE_MAX_STATIONS], nz[RACE_MAX_STATIONS];   /* normal a la derecha del avance */
    float lo[RACE_MAX_STATIONS], hi[RACE_MAX_STATIONS];   /* tramo libre sobre la normal (cm) */
    bool corner[RACE_MAX_STATIONS];                       /* la linea central gira aqui */
    float d[RACE_MAX_STATIONS];                           /* desplazamiento elegido */
    float x[RACE_MAX_STATIONS], z[RACE_MAX_STATIONS];     /* la linea de carrera */
    float s[RACE_MAX_STATIONS];                           /* arco acumulado */
    float h[RACE_MAX_STATIONS];                           /* de la estacion i a la i + 1 */
    float kappa[RACE_MAX_STATIONS];                       /* curvatura (1/cm) */
    float heading_deg[RACE_MAX_STATIONS];                 /* rumbo del campo de la tangente */
    float v[RACE_MAX_STATIONS];                           /* velocidad objetivo (cm/s) */
    float length_cm;
    float time_s;                /* vuelta estimada con el perfil */
    float time_center_s;         /* la misma con la linea central (para saber lo que se gana) */
    float mix;                   /* mezcla elegida (0 = la mas corta) */
    int signs;
    race_sign_t sign[RACE_MAX_SIGNS];
    float sign_s[RACE_MAX_SIGNS];
    float sign_rx[RACE_MAX_SIGNS], sign_rz[RACE_MAX_SIGNS];   /* a la derecha del avance en el letrero */
} race_line_t;

void race_config_default(race_config_t *cfg);
bool race_config_sanitize(race_config_t *cfg);

/* Calcula la linea para el sentido dado. Devuelve false (y r->valid = false) si algun tramo no tiene paso. */
bool race_line_build(race_line_t *r, const race_config_t *cfg, int direction, const race_sign_t *signs, int count);

/* Estacion mas cercana a (x, z). Con hint >= 0 solo busca a +-window_cm a lo largo de la linea. Devuelve el indice
 * y, si no son NULL, el desplazamiento lateral (derecha positivo) y la distancia. -1 si no hay linea. */
int race_line_locate(const race_line_t *r, float x_cm, float z_cm, int hint, float window_cm,
                     float *lateral_cm, float *dist_cm);
/* Cuanto vale la linea como referencia en el arco `s_cm`: 0 en una esquina y 1 a `before_cm` antes o `after_cm` despues
 * (lineal). El controlador toma las esquinas girando junto al centro del carril, no siguiendo la linea, asi que cerca
 * de ellas la referencia lateral se mezcla con el centro del carril. */
float race_line_corner_factor(const race_line_t *r, float s_cm, float before_cm, float after_cm);
/* Estacion `ahead_cm` por delante de la dada (da la vuelta). */
int race_line_advance(const race_line_t *r, int idx, float ahead_cm);
/* Punto de la linea a `ahead_cm` de la estacion idx (interpolado) y su rumbo. */
void race_line_point(const race_line_t *r, int idx, float ahead_cm, float *x_cm, float *z_cm, float *heading_deg);
/* Velocidad objetivo `ahead_cm` por delante de la estacion idx. */
float race_line_speed(const race_line_t *r, int idx, float ahead_cm);

#endif

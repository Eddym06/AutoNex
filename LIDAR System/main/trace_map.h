#ifndef TRACE_MAP_H
#define TRACE_MAP_H
/*
 * trace_map: mapa persistente de la pista, en el marco del campo localizado (arena_localizer).
 *
 * Capa 1, rejilla (400 x 400 cm, celdas de 2,5 cm): por celda, un byte con retorno LiDAR, color confirmado por
 * fusion y repeticion, y otro byte con cuantas veces un rayo la ATRAVESO. Eso da espacio ocupado, espacio libre
 * conocido y desconocido, y permite borrar lo que se movio.
 * Capa 2, objetos: las celdas estables se agrupan en objetos (pilar, barra, pared sobrante) con identidad estable,
 * posicion, tamano, orientacion, color y confianza. Las paredes del campo se restan con un modelo del campo
 * (cuadrado exterior e interior), de modo que lo que queda son letreros, topes y cualquier cosa inesperada.
 * Capa 3, plaza de parqueo: el par de topes (barras perpendiculares a la pared exterior, separadas lo que mide el
 * cajon) se reconoce solo con la geometria, sin cuadro de camara; el magenta solo suma confianza.
 *
 * Vueltas: la primera construye el mapa; en cada vuelta siguiente (trace_map_lap_complete) se refina.
 *
 * C99 puro, una sola instancia por proceso (usa buffers de trabajo estaticos), sin asignacion dinamica.
 * RAM: unos 62 KB. No controla el vehiculo: lo consultan los niveles superiores.
 */
#include <stddef.h>
#include <stdint.h>
#include "arena_localizer.h"
#include "local_obstacle_map.h"

#define TRACE_MAP_SIZE 160
#define TRACE_MAP_CELL_CM 2.5f
#define TRACE_MAP_ORIGIN_CM (-200.0f)
#define TRACE_MAP_OBSERVED 0x01u
#define TRACE_MAP_RED 0x02u
#define TRACE_MAP_GREEN 0x04u
#define TRACE_MAP_MAGENTA_CANDIDATE 0x08u
#define TRACE_MAP_SUPPORT_SHIFT 4
#define TRACE_MAP_MAX_OBJECTS 48

typedef enum { TRACE_COLOR_NONE = 0, TRACE_COLOR_RED = 1, TRACE_COLOR_GREEN = 2, TRACE_COLOR_MAGENTA = 3 } trace_color_t;
typedef enum { TRACE_OBJ_UNKNOWN = 0, TRACE_OBJ_WALL = 1, TRACE_OBJ_PILLAR = 2, TRACE_OBJ_BAR = 3 } trace_obj_kind_t;

typedef struct {
    uint16_t id;                 /* estable mientras el objeto siga ahi */
    uint8_t kind;                /* trace_obj_kind_t */
    uint8_t color;               /* trace_color_t */
    uint8_t confidence;          /* 0..255 (repeticion de los retornos) */
    uint16_t cells;
    uint8_t laps_seen;           /* vueltas completas en las que el objeto sigue en el mapa (1 = solo la de construccion) */
    float x_cm, z_cm;            /* centro, en el campo */
    float length_cm, width_cm;   /* extension a lo largo y a lo ancho del eje principal */
    float axis_deg;              /* direccion del eje principal, horario desde el norte (+z), en [0,180) */
    uint32_t last_scan;
} trace_object_t;

typedef enum { TRACE_SPOT_NONE = 0, TRACE_SPOT_HYPOTHESIS = 1, TRACE_SPOT_CONFIRMED = 2 } trace_spot_status_t;
typedef struct {
    trace_spot_status_t status;
    float x_cm, z_cm;            /* centro de la plaza (CONFIRMED) */
    float axis_deg;              /* direccion a lo largo de la pared: 0 si la pared es +-x, 90 si es +-z */
    float length_cm, width_cm;   /* largo libre entre topes y ancho */
    float bar_x_cm[2], bar_z_cm[2];   /* centros de los dos topes (el segundo vale NAN en HYPOTHESIS) */
    uint16_t bar_id[2];
    uint8_t wall_side;           /* 0 pared +x, 1 -x, 2 +z, 3 -z */
    uint8_t confidence;
} trace_spot_t;

typedef struct {
    uint8_t cells[TRACE_MAP_SIZE][TRACE_MAP_SIZE];
    uint8_t miss[TRACE_MAP_SIZE][TRACE_MAP_SIZE];      /* rayos que atravesaron la celda (sin pasar de 255) */
    /* Marca de celdas tocadas en este barrido: 1 bit por celda (un retorno / un rayo cuenta una vez por barrido). */
    uint8_t scan_seen[TRACE_MAP_SIZE * TRACE_MAP_SIZE / 8];
    uint8_t miss_seen[TRACE_MAP_SIZE * TRACE_MAP_SIZE / 8];
    uint32_t occupied_cells;
    uint32_t scans;
    uint32_t clipped_points;
    uint32_t cleared_cells;      /* celdas ocupadas que se borraron por haberse atravesado (algo se movio) */
    /* Modelo del campo y de la plaza. 0 = valor por defecto de la sede (campo de 300 cm, bloque central de 100 cm). */
    float field_outer_half_cm, field_inner_half_cm, wall_band_cm;
    float spot_spacing_cm;       /* distancia entre centros de los dos topes (largo del cajon + grosor de un tope) */
    trace_object_t objects[TRACE_MAP_MAX_OBJECTS];
    uint8_t object_count;
    uint16_t next_id;
    uint32_t refreshed_scan;
    uint8_t laps_mapped;         /* vueltas completas acumuladas: 0 = construyendo la primera, >= 1 = mapa construido */
    uint32_t lap_start_scan;     /* barrido en que empezo la vuelta en curso */
    uint32_t spot_scan;         /* barrido en que se vio por ultima vez la plaza confirmada */
    trace_spot_t spot;
} trace_map_t;

/* Acumula un barrido con la pose del campo. Sin pose valida no hace nada salvo contar. */
void trace_map_update(trace_map_t *m, const local_lidar_point_t *points, size_t count,
                      const local_obstacle_map_t *local, const arena_pose_t *pose,
                      float lidar_forward_cm);
/* Reconstruye la lista de objetos y la plaza. Barato (< 1 ms en PC): llamarlo cada ~10 barridos. */
void trace_map_refresh(trace_map_t *m);
/* Cierra una vuelta: reconstruye objetos y plaza y cuenta la vuelta. La primera construye el mapa; las siguientes lo
 * refinan (cada pasada suma retornos, borra lo que se movio y sube `laps_seen` de los objetos que siguen en su sitio). */
void trace_map_lap_complete(trace_map_t *m);
int trace_map_is_built(const trace_map_t *m);        /* 1 si ya hay al menos una vuelta completa */

/* Consultas. Las coordenadas son del campo, en cm. */
const trace_object_t *trace_map_objects(const trace_map_t *m, size_t *count);
const trace_spot_t *trace_map_spot(const trace_map_t *m);
uint8_t trace_map_cell_at(const trace_map_t *m, float x_cm, float z_cm);       /* byte de la celda; 0 fuera del mapa */
int trace_map_is_free(const trace_map_t *m, float x_cm, float z_cm);           /* 1 = espacio libre conocido */
float trace_map_clearance_cm(const trace_map_t *m, float x_cm, float z_cm, float max_cm);   /* al obstaculo observado mas cercano */

/* Guardado compacto (solo celdas con datos + objetos + plaza, con CRC). Devuelve los bytes escritos, 0 si no cabe. */
size_t trace_map_save(const trace_map_t *m, uint8_t *buf, size_t cap);
int trace_map_load(trace_map_t *m, const uint8_t *buf, size_t len);            /* 1 si es valido */
#endif

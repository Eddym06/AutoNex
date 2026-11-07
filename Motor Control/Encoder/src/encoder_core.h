/*
 * encoder_core.h - Estimador de posición/velocidad para encoder óptico ranurado
 *
 * Disco de 30 ranuras -> 60 flancos por vuelta (6 grados por flanco).
 * C99 puro, sin dependencias de ESP-IDF: el mismo código corre en el ESP32-P4
 * y en el simulador del PC (sim/encoder_sim.c).
 *
 * Ideas principales (ver README.md):
 *   1. Tabla de calibración por flanco en FRACCIONES DE VUELTA (no solo un factor
 *      de tiempo): da la posición real de cada flanco impreso, no la nominal.
 *   2. Velocidad = longitud real del intervalo / tiempo del intervalo (Método T).
 *   3. Aceleración con diferencias entre puntos medios de intervalos (más
 *      exacta que dividir por un solo intervalo).
 *   4. Interpolación MRUA entre flancos, acotada al intervalo que se está
 *      recorriendo y con manejo de "flanco atrasado" (el robot está frenando).
 *   5. Cotas garantizadas [pos_lo, pos_hi]: la posición real SIEMPRE está dentro.
 *   6. Robustez: filtro de glitches con cancelación de pares, detección de
 *      flancos perdidos por paridad de nivel, reversa, timeout de parada.
 *   7. Alineación automática tabla<->disco por "huella" de los intervalos.
 *
 * Unidades: metros, segundos, m/s, m/s^2. Tiempo en microsegundos (uint64_t).
 */
#ifndef ENCODER_CORE_H
#define ENCODER_CORE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENC_EDGES           60   /* 2 x ranuras */
#define ENC_CAL_MAX_REVS    24   /* vueltas que guarda el calibrador */
#define ENC_CAL_SKIP_REVS    2   /* vueltas iniciales que se ignoran (arranque) */
#define ENC_ALIGN_WINDOW    24   /* intervalos consecutivos para la huella */

/* ------------------------------------------------------------------ */
/* Configuración                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    float    wheel_radius_m;     /* radio efectivo de la rueda (calibrar!)        */
    uint32_t min_interval_us;    /* eventos más juntos que esto = ruido           */
    uint32_t stop_timeout_us;    /* sin flancos por más de esto = detenido        */
    float    accel_max;          /* tope |a| en m/s^2                             */
    float    accel_alpha;        /* filtro de a: 0 = sin término a, 1 = sin filtro */
    float    hold_frac;          /* dónde "estacionar" la estimación si el robot
                                    se detiene dentro de un intervalo (0..1)       */
    float    overdue_tol;        /* tolerancia de flanco atrasado (fracción de D)  */
    float    align_cost_max;     /* umbral de varianza relativa para la huella     */
    float    align_ratio_min;    /* el 2do mejor candidato debe ser X veces peor   */
} enc_config_t;

void enc_config_default(enc_config_t *cfg);

/* ------------------------------------------------------------------ */
/* Estado dinámico (se copia entero para cancelar glitches)           */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t  n;                  /* flancos cruzados desde el primero (con signo)  */
    uint64_t t_last_us;          /* timestamp del último flanco aceptado           */
    uint8_t  level_last;         /* nivel del sensor tras ese flanco               */
    int8_t   dir;                /* +1 / -1                                        */
    bool     started;
    bool     v_valid;            /* hay velocidad media del intervalo previo       */
    bool     motion_valid;       /* v y a sirven para extrapolar                   */
    float    dt_prev_s;
    float    vbar_prev;          /* velocidad media del último intervalo           */
    float    v;                  /* velocidad en el instante del último flanco     */
    float    a;                  /* aceleración filtrada                           */
    uint8_t  ivl_count;          /* intervalos consecutivos válidos en el anillo   */
    uint8_t  ivl_head;
    float    ivl[ENC_ALIGN_WINDOW];
} enc_dyn_t;

typedef struct {
    enc_config_t cfg;
    float        circ;                       /* 2*pi*R                       */
    float        cum_u[ENC_EDGES + 1];       /* tabla uniforme (i/60)        */
    float        cum_c[ENC_EDGES + 1];       /* tabla calibrada acumulada    */
    float        frac_c[ENC_EDGES];          /* fracciones calibradas        */
    bool         cal_valid;                  /* hay tabla calibrada          */
    bool         aligned;                    /* tabla alineada con el disco  */
    int32_t      phase;                      /* índice físico del 1er flanco */
    enc_dyn_t    d;
    enc_dyn_t    snap;                       /* estado previo (cancelar glitch) */
    bool         snap_valid;
    /* diagnóstico */
    uint32_t     glitches, missed, accepted;
    int          last_steps;
    uint32_t     last_dt_us;
    bool         last_ivl_ok;
} enc_t;

typedef struct {
    float   pos_m;               /* distancia con signo desde el primer flanco    */
    float   pos_lo_m;            /* cota inferior garantizada                     */
    float   pos_hi_m;            /* cota superior garantizada                     */
    float   vel_mps;             /* velocidad con signo                           */
    float   acc_mps2;
    bool    moving;
    int32_t edges;
} enc_est_t;

/* ------------------------------------------------------------------ */
/* API de tiempo de ejecución                                         */
/* ------------------------------------------------------------------ */
void enc_init(enc_t *e, const enc_config_t *cfg);   /* cfg NULL = valores por defecto */
void enc_reset(enc_t *e);                           /* reinicia posición a 0          */

/* Carga la tabla calibrada (fracciones de vuelta por intervalo, suma = 1). */
void enc_set_table(enc_t *e, const float frac[ENC_EDGES]);

/* Si ya conoces qué flanco físico es el primero (índice de la tabla). */
void enc_set_phase(enc_t *e, int32_t phase);

/* Procesa un flanco. level = nivel del GPIO justo después del flanco.
 * dir = +1/-1 (dirección real de giro, ver README: con un solo sensor
 * la dirección sale del comando del motor).
 * Devuelve false si el evento se descartó como ruido. */
bool enc_on_edge(enc_t *e, uint64_t t_us, int level, int dir);

/* Estimación en un instante cualquiera (típicamente cada 1 ms). Pura: no
 * modifica el estado. t_now_us debe ser >= timestamp del último flanco. */
void enc_estimate(const enc_t *e, uint64_t t_now_us, enc_est_t *out);

/* Intento de alineación por huella (se llama solo desde enc_on_edge). */
bool enc_align_try(enc_t *e);

/* ------------------------------------------------------------------ */
/* Calibración                                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t dt_us[ENC_CAL_MAX_REVS][ENC_EDGES];
    uint64_t valid[ENC_CAL_MAX_REVS];        /* máscara de 60 bits por vuelta */
} enc_cal_t;

typedef struct {
    int   revs_complete;
    int   revs_used;
    float rev_time_s;            /* periodo medio de vuelta                     */
    float spread_mean;           /* dispersión relativa media entre vueltas     */
    float spread_max;            /* y la máxima (ruido/ripple del motor)        */
    float ratio_min, ratio_max;  /* fraccion*60: 1.0 = flanco perfecto          */
    float harm_deg[8];           /* amplitud (grados de eje) de los armónicos 1..8
                                    del error de la tabla ANTES de filtrar; el ripple
                                    del motor decae como 1/h, el error de impresión es
                                    plano (ver README)                            */
    bool  ok;
} enc_cal_report_t;

void enc_cal_begin(enc_cal_t *c);
void enc_cal_feed(enc_cal_t *c, int32_t n, uint32_t dt_us, bool valid);
/* Atajo: alimenta con el último flanco procesado por enc_on_edge()==true. */
void enc_cal_feed_from(enc_cal_t *c, const enc_t *e);
/* harm_cut: se eliminan de la tabla los armónicos 1..harm_cut del error de
 * posición. Motivo: el rizado de velocidad del motor, que es síncrono con el
 * ángulo, es indistinguible de un error geométrico del disco en una calibración
 * a velocidad constante, y contamina sobre todo los armónicos bajos. */
bool enc_cal_finish(const enc_cal_t *c, float frac_out[ENC_EDGES], enc_cal_report_t *rep, int harm_cut);

#ifdef __cplusplus
}
#endif
#endif /* ENCODER_CORE_H */

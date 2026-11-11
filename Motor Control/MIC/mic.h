/*
 * mic.h - MIC, Motor Intelligent Control.
 *
 * Modulo unico y centralizado que gobierna el motor de traccion (JGB37-520 12 V con reductora
 * y encoder A/B de efecto Hall, puente L298N). Los niveles superiores (parqueo, curvas, rectas,
 * esquivar) NO mandan ciclos de PWM: piden velocidad, distancias o una ruta con restricciones, y MIC
 * decide en cada instante la tension que hay que aplicar.
 *
 * Que lo hace distinto de rampas + PID:
 *   1. MODELO FISICO del motor y del coche (R, Ke, masa, friccion, bateria, caida del puente, limite
 *      de corriente del L298N). La tension de cada instante sale de la dinamica inversa: "quiero esta
 *      aceleracion -> necesito esta fuerza -> esta corriente -> esta tension". El PI solo corrige el error.
 *   2. OBSERVADOR (posicion y velocidad) que usa ese modelo para predecir entre cuentas del encoder y
 *      se corrige con el. Da velocidad limpia a 1 kHz y la anticipa.
 *   3. PLANIFICADOR CON ANTICIPACION: una ruta (recta -> curva -> parqueo) se convierte en un perfil de
 *      velocidad v(s) con la maxima aceleracion y la desaceleracion que el motor realmente puede dar a
 *      cada velocidad. El coche mantiene la velocidad (usa su impulso) y frena lo mas tarde posible,
 *      con la fuerza justa, llegando a cada punto con la velocidad pedida.
 *   4. Dos formas de desacelerar, un solo ley de control:
 *        PROGRESIVA: sin tension inversa. Entre rodar (tension = fuerza contraelectromotriz) y el freno
 *                    corto (tension 0) se elige la tension que da la desaceleracion exacta.
 *        AGRESIVA:   frenado activo con tension inversa (limitada por la corriente), solo si hace falta.
 *      Si la progresiva ya no llega al punto, MIC escala a frenado activo (emergencia) y lo registra.
 *
 * C99 puro, sin dependencias de ESP-IDF. Unidades de la API: cm, cm/s, cm/s^2, segundos, voltios.
 * Internamente trabaja en SI (m, m/s, N, A).
 */
#ifndef MIC_H
#define MIC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIC_ROUTE_MAX_SEGMENTS 16
#define MIC_PROFILE_POINTS 640

typedef enum {
    MIC_DECEL_PROGRESSIVE = 0,   /* sin tension inversa */
    MIC_DECEL_AGGRESSIVE = 1     /* frenado activo con tension inversa */
} mic_decel_t;

typedef enum {
    MIC_PHASE_HOLD = 0,   /* parado y sujeto (freno corto) */
    MIC_PHASE_DRIVE,      /* corriente positiva en el sentido de la marcha: acelera o mantiene */
    MIC_PHASE_COAST,      /* corriente casi nula: rueda con su impulso */
    MIC_PHASE_BRAKE,      /* frenado progresivo (tension >= 0 pero menor que la fcem) */
    MIC_PHASE_PLUG        /* frenado activo (tension inversa) */
} mic_phase_t;

typedef struct {
    /* --- geometria y encoder --- */
    float wheel_diam_cm;        /* diametro de la rueda motriz */
    float gear_ratio;           /* reduccion del motor a la rueda (JGB37-520 de 600 rpm: 10) */
    float counts_per_motor_rev; /* cuentas por vuelta del eje del motor (11 pulsos x 4 flancos = 44) */
    /* --- motor --- */
    float v_nl_volts;           /* tension a la que se mide la velocidad en vacio (12 V) */
    float rpm_nl_out;           /* rpm en vacio de la reductora a esa tension (600 hoy; 1000 el motor definitivo) */
    float i_noload_a;           /* corriente en vacio (A) */
    float r_ohm;                /* resistencia del inducido (por medir: ~2,6 ohm supuesto) */
    float eta;                  /* rendimiento de la reductora y la transmision */
    /* --- coche --- */
    float mass_kg;              /* masa del coche */
    float j_rotor_kgm2;         /* inercia del rotor (se refleja a la rueda) */
    float f_coulomb_n;          /* friccion seca total (reductora, escobillas, rodadura) */
    float b_viscous_ns_m;       /* friccion viscosa */
    float a_grip_ms2;           /* tope de aceleracion/frenado por adherencia (m/s^2) */
    /* --- puente y bateria --- */
    float vbat_nominal;         /* V */
    float vdrop_volts;          /* caida del L298N */
    float i_max_a;              /* corriente maxima admitida por el puente (2 A continuos) */
    bool can_coast;             /* ENA controlable: rodar libre sin freno (hoy puenteado: false) */
    /* --- control --- */
    float kv_gain;              /* 1/s: correccion de aceleracion por error de velocidad */
    float ki_gain;              /* 1/s^2: integral del error de velocidad (perturbaciones) */
    float obs_bw_hz;            /* ancho de banda del observador */
    float plan_decel_frac;      /* fraccion de la desaceleracion maxima que usa el planificador (0..1) */
    float plan_accel_frac;      /* fraccion de la aceleracion maxima que usa el planificador */
    float v_creep_cm_s;         /* velocidad de aproximacion final */
    float pos_tol_cm;           /* tolerancia de parada */
    float v_stop_cm_s;          /* por debajo de esto se considera parado */
    float v_max_cm_s;           /* tope de velocidad */
} mic_cfg_t;

/* Perfil del chasis del equipo: JGB37-520 12 V, 10:1, encoder 11 PPR, rueda de 6,6 cm. Varios valores
 * (resistencia, masa, friccion) son SUPUESTOS hasta medirlos: ver README.md, seccion de calibracion.
 * `_600rpm` es el motor que hay hoy (perfil por defecto); `_1000rpm` el definitivo, que aun no ha llegado. */
void mic_cfg_jgb37_520(mic_cfg_t *cfg, float rpm_out);
void mic_cfg_jgb37_520_600rpm(mic_cfg_t *cfg);
void mic_cfg_jgb37_520_1000rpm(mic_cfg_t *cfg);

typedef struct {
    int32_t counts;             /* cuentas acumuladas del encoder (con signo, x4) */
    float vbat;                 /* tension de bateria medida (V); <=0 = usar la nominal */
} mic_in_t;

typedef struct {
    float duty;                 /* ciclo con signo (-1..1) hacia el L298N */
    bool brake_short;           /* true: las dos entradas al mismo nivel (freno corto / sujecion) */
    mic_phase_t phase;
    bool done;                  /* la orden de avance/ruta termino y el coche esta parado */
    bool emergency;             /* se uso frenado activo aunque se pidio progresivo */
    /* telemetria (cm, cm/s) */
    float pos_cm, v_cm_s, v_ref_cm_s, remaining_cm;
    float i_est_a;              /* corriente estimada */
    float disturbance_ms2;      /* aceleracion no modelada que estima el observador */
} mic_out_t;

typedef struct {
    float length_m, v_max, v_exit_max;
} mic_segment_t;

typedef struct {
    mic_cfg_t cfg;
    /* constantes derivadas (SI) */
    float cm_per_count, ke_lin, kf_lin, m_eff, ke_motor, v_avail_nom;
    /* observador */
    float s, v;                 /* m, m/s */
    float d_est;                /* aceleracion no modelada estimada (m/s^2): carga, pendiente, friccion distinta */
    float s_off;                /* origen de posicion del encoder (m) */
    bool obs_init;
    float vm_last;              /* ultima tension aplicada */
    /* orden activa */
    enum { MIC_MODE_IDLE = 0, MIC_MODE_SPEED, MIC_MODE_ROUTE } mode;
    mic_decel_t decel;
    float v_target, v_ref;
    float integ;
    bool fresh, done_flag;
    /* ruta */
    mic_segment_t seg[MIC_ROUTE_MAX_SEGMENTS];
    int nseg;
    float route_s0, route_len, route_ds, route_v_end;
    float prof[MIC_PROFILE_POINTS];   /* v(s) en m/s */
    int nprof;
    bool emergency_latched;
} mic_t;

void mic_init(mic_t *m, const mic_cfg_t *cfg);

/* ---- API para los niveles superiores (todo pasa por aqui) ---- */
void mic_request_speed(mic_t *m, float v_cm_s, mic_decel_t decel);
void mic_request_stop(mic_t *m, mic_decel_t decel);
/* Avanzar dist_cm (con signo) y llegar con v_end_cm_s (0 = parado). Un solo tramo. */
void mic_request_move(mic_t *m, float dist_cm, float v_cruise_cm_s, float v_end_cm_s, mic_decel_t decel);
/* Ruta con anticipacion: se agregan tramos (longitud, velocidad maxima dentro, velocidad maxima al salir)
 * y se confirma con la velocidad final. Devuelve false si no es valida. Sentido: avance (positivo). */
void mic_route_begin(mic_t *m);
bool mic_route_add(mic_t *m, float length_cm, float v_max_cm_s, float v_exit_max_cm_s);
bool mic_route_commit(mic_t *m, float v_end_cm_s, mic_decel_t decel);

/* Un ciclo de control (1 kHz recomendado). */
void mic_update(mic_t *m, const mic_in_t *in, float dt, mic_out_t *out);

/* Utilidades de planificacion para otros modulos. */
float mic_max_decel_cm_s2(const mic_t *m, float v_cm_s, mic_decel_t decel);
float mic_max_accel_cm_s2(const mic_t *m, float v_cm_s);
float mic_stop_distance_cm(const mic_t *m, float v_cm_s, mic_decel_t decel);

#ifdef __cplusplus
}
#endif
#endif

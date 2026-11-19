/* Odometria de un robot diferencial con encoders en las ruedas y giroscopio.
 *
 * Un encoder por rueda entrega el angulo acumulado (grados de
 * rueda). Este modulo no sabe nada del sensor ni del protocolo: recibe
 * dos angulos acumulados y los convierte en distancia y posicion. Todo lo que no se
 * conoce con certeza (diametro de rueda, separacion, escala de cada motor, sentido de
 * giro) es CONFIGURACION y puede calibrarse con el LiDAR y el giroscopio
 * (odom_calibrate_*), asi que cambiar de rueda o de chasis no toca el codigo.
 *
 * Convencion: el rumbo es horario-positivo (como el giroscopio y el resto del
 * firmware). La posicion esta en el marco del giroscopio: x a la derecha del rumbo
 * cero, y hacia donde mira el rumbo cero, en cm, con el origen donde arranca el eje
 * de las ruedas.
 *
 * El rumbo sale del giroscopio (no deriva con el patinaje); los encoders dan la
 * distancia. Si los dos discrepan en el giro mas de lo razonable hay patinaje (una
 * rueda gira sin avanzar el robot): se anota y esa muestra no cuenta como avance. */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float wheel_diameter_cm;     /* diametro de la rueda (se calibra) */
    float track_width_cm;        /* distancia entre los puntos de apoyo de las dos ruedas */
    float left_scale, right_scale;   /* multiplicadores de calibracion de cada lado (1 = nominal) */
    int8_t left_sign, right_sign;   /* +1 si el angulo crece al avanzar; -1 si el motor esta montado al reves */
    float slip_yaw_dps;          /* discrepancia de giro encoder/giroscopio a partir de la cual hay patinaje */
    float slip_min_dt_s;         /* intervalo minimo para evaluar el patinaje */
    float max_gap_s;             /* mas tiempo sin muestras: se reinicia la referencia, no se integra el hueco */
    float max_speed_cm_s;        /* una muestra que implique mas velocidad es un error de lectura */
} odom_config_t;

typedef struct {
    odom_config_t cfg;
    bool have_prev;
    uint64_t t_prev_us;
    float left_prev_deg, right_prev_deg;
    float yaw_prev_deg;
    bool yaw_prev_valid;
    float x_cm, y_cm;            /* posicion en el marco del giroscopio */
    float heading_deg;           /* ultimo rumbo usado (giroscopio, o encoders si no hay) */
    float path_cm;               /* distancia recorrida acumulada (siempre crece) */
    float v_cm_s, omega_dps;     /* velocidad del eje y giro (filtrados) */
    float slip_cm;               /* distancia de encoder descartada por patinaje */
    int slip_count;
    bool slipping;               /* la ultima muestra se descarto por patinaje */
    int samples, rejected;
} odom_t;

void odom_config_default(odom_config_t *cfg);
/* Acota los campos a rangos con los que el modulo funciona. */
bool odom_config_sanitize(odom_config_t *cfg);
void odom_init(odom_t *o, const odom_config_t *cfg);
void odom_reset_pose(odom_t *o, float x_cm, float y_cm, float heading_deg);

/* Una muestra: angulos acumulados de cada rueda (grados) a la hora t_us, y el rumbo del
 * giroscopio en ese instante (yaw_valid=false si no hay). Devuelve la distancia con
 * signo que avanzo el eje desde la muestra anterior (cm), 0 si se descarto. */
float odom_update(odom_t *o, uint64_t t_us, float left_deg, float right_deg, float yaw_deg, bool yaw_valid);

/* Eje comun: un solo encoder aporta la distancia; el rumbo solo lo aporta la IMU.
 * No se puede estimar patinaje comparando dos ruedas con esta configuracion. */
float odom_update_axle(odom_t *o, uint64_t t_us, float axle_deg, float yaw_deg, bool yaw_valid);

/* Avance del eje (cm, con signo) y giro (grados, horario) que implican dos angulos de rueda. */
float odom_wheel_to_cm(const odom_t *o, float deg, bool left);

/* Calibracion con una referencia externa. Cada una devuelve false si el movimiento no fue
 * suficiente o es incoherente (no modifica nada).
 *  - escala de distancia: el LiDAR midio que el robot avanzo `true_cm` mientras los
 *    encoders dieron `enc_cm` (suma de los dos lados / 2);
 *  - separacion de ruedas: el giroscopio midio un giro de `gyro_deg` mientras la diferencia de
 *    los encoders dio `enc_diff_cm` (izquierda - derecha). */
bool odom_calibrate_distance(odom_config_t *cfg, float enc_cm, float true_cm);
bool odom_calibrate_track(odom_config_t *cfg, float enc_diff_cm, float gyro_deg);
/* Compensa que un lado ande mas que el otro: en una recta, el giroscopio midio `gyro_deg` de giro
 * (horario) con los encoders dando `left_cm` y `right_cm`. */
bool odom_calibrate_balance(odom_config_t *cfg, float left_cm, float right_cm, float gyro_deg);

#endif

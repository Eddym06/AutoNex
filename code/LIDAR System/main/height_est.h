#ifndef HEIGHT_EST_H
#define HEIGHT_EST_H

#include <stdbool.h>

/* Altura relativa del robot con el acelerometro del IMU (inclinacion) y la distancia recorrida.
 *
 * Integrar el acelerometro dos veces da errores que crecen con el cuadrado del tiempo; en cambio la inclinacion
 * respecto a la gravedad es estable. Altura = suma de (avance a lo largo del robot) x sen(inclinacion). Sirve para
 * rampas y pendientes; unas escaleras sin inclinacion no se ven (haria falta un barometro o un sensor de distancia).
 *
 * Todo se mide respecto a la referencia que se toma al iniciar (height_start): la gravedad de ese momento es la
 * "horizontal" y la altura vale 0. Si la altura se aleja mas de HEIGHT_LEVEL_CM del nivel actual, empieza un nivel
 * nuevo (una medida distinta). El eje delantero del IMU se aprende levantando el morro (height_calibrate_axis);
 * mientras tanto se supone el eje X del GY-521. */
#define HEIGHT_LEVEL_CM 30.0f

typedef struct {
    float u0[3];            /* fuerza especifica (gravedad medida) unitaria al iniciar */
    float u[3];             /* idem, filtrada, ahora */
    float fwd[3];           /* eje delantero del robot en el marco del IMU (unitario, perpendicular a u0) */
    bool have_u, have_ref, fwd_cal;
    float pitch_deg;        /* inclinacion con signo: morro arriba positivo */
    float tilt_deg;         /* angulo total respecto a la referencia */
    float height_cm;        /* desde que se inicio */
    float level_origin_cm;  /* altura donde empezo el nivel actual */
    int level;              /* 0 = el del inicio */
} height_est_t;

void height_reset(height_est_t *h);
/* Muestra del acelerometro en g (x, y, z del IMU), dt en segundos. Solo cuenta si el modulo esta cerca de 1 g
 * (sin aceleraciones fuertes): asi un arranque o un choque no se confunden con una inclinacion. */
void height_accel(height_est_t *h, const float a[3], float dt);
/* Toma la referencia (la gravedad ahora es "horizontal"), pone la altura a 0 y el nivel a 0. */
void height_start(height_est_t *h);
/* Con el morro levantado al menos 5 grados: aprende el eje delantero. Devuelve false si no hay inclinacion. */
bool height_calibrate_axis(height_est_t *h);
/* Avance (cm, positivo hacia el morro) desde la llamada anterior. Devuelve true si cambio de nivel. */
bool height_move(height_est_t *h, float forward_cm);

#endif

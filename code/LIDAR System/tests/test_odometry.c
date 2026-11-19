#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FALLA: %s (linea %d)\n", msg, __LINE__); ++failures; } } while (0)
#define NEAR(a, b, tol, msg) do { const double _a = (a), _b = (b); if (fabs(_a - _b) > (tol)) { \
    printf("FALLA: %s: %.3f vs %.3f (linea %d)\n", msg, _a, _b, __LINE__); ++failures; } } while (0)

static float deg_per_cm(const odom_t *o) { return 360.0f / (3.14159265f * o->cfg.wheel_diameter_cm); }

/* Un robot ideal: velocidad v (cm/s) y giro w (grados/s horario); devuelve los angulos de rueda acumulados. */
static void drive_ideal(odom_t *o, double v, double w_dps, double secs, double hz, uint64_t *t, double *left, double *right,
                        double *yaw, double *x, double *y)
{
    const double dt = 1.0 / hz;
    for (int i = 0; i < (int)(secs * hz); ++i) {
        const double yaw1 = *yaw + w_dps * dt;
        const double mid = (*yaw + 0.5 * w_dps * dt) * M_PI / 180.0;
        *x += v * dt * sin(mid);
        *y += v * dt * cos(mid);
        const double half = 0.5 * o->cfg.track_width_cm * w_dps * dt * M_PI / 180.0;
        *left += (v * dt + half) * deg_per_cm(o);
        *right += (v * dt - half) * deg_per_cm(o);
        *yaw = yaw1;
        *t += (uint64_t)(dt * 1e6);
        odom_update(o, *t, (float)*left, (float)*right, (float)*yaw, true);
    }
}

int main(void)
{
    odom_t o;
    odom_init(&o, NULL);
    uint64_t t = 1000000;
    double l = 0, r = 0, yaw = 0, x = 0, y = 0;
    odom_update(&o, t, 0.0f, 0.0f, 0.0f, true);

    /* Recta de 100 cm. */
    drive_ideal(&o, 20.0, 0.0, 5.0, 50.0, &t, &l, &r, &yaw, &x, &y);
    NEAR(o.y_cm, 100.0, 0.5, "recta: avance");
    NEAR(o.x_cm, 0.0, 0.2, "recta: sin deriva lateral");
    NEAR(o.path_cm, 100.0, 0.5, "recta: camino recorrido");
    NEAR(o.v_cm_s, 20.0, 1.0, "velocidad filtrada");

    /* Arco: 15 cm/s con 30 grados/s durante 3 s. */
    drive_ideal(&o, 15.0, 30.0, 3.0, 50.0, &t, &l, &r, &yaw, &x, &y);
    NEAR(o.x_cm, x, 0.8, "arco: x");
    NEAR(o.y_cm, y, 0.8, "arco: y");
    NEAR(o.heading_deg, 90.0, 0.5, "arco: rumbo");

    /* Giro en el sitio: el eje no avanza. */
    const double px = o.x_cm, py = o.y_cm;
    drive_ideal(&o, 0.0, 90.0, 2.0, 50.0, &t, &l, &r, &yaw, &x, &y);
    NEAR(o.x_cm, px, 0.3, "pivote: x quieta");
    NEAR(o.y_cm, py, 0.3, "pivote: y quieta");
    CHECK(!o.slipping, "pivote ideal no es patinaje");

    /* Marcha atras. */
    const double by = o.y_cm, bx = o.x_cm;
    drive_ideal(&o, -10.0, 0.0, 2.0, 50.0, &t, &l, &r, &yaw, &x, &y);
    NEAR(hypot(o.x_cm - bx, o.y_cm - by), 20.0, 0.4, "reversa: distancia");
    NEAR(o.path_cm, 100.0 + 15.0 * 3.0 + 20.0, 1.5, "camino recorrido incluye la reversa");

    /* Patinaje: una rueda gira y el robot no se mueve (el giroscopio no ve giro). */
    {
        odom_t s; odom_init(&s, NULL);
        uint64_t ts = 1000000;
        odom_update(&s, ts, 0, 0, 0.0f, true);
        double ls = 0;
        for (int i = 0; i < 25; ++i) {
            ts += 20000; ls += 1.0 * deg_per_cm(&s);          /* la izquierda gira 1 cm por muestra */
            odom_update(&s, ts, (float)ls, 0.0f, 0.0f, true);
        }
        CHECK(s.slip_count > 20, "patinaje detectado");
        NEAR(s.path_cm, 0.0, 0.5, "patinaje: no cuenta como avance");
        NEAR(hypot(s.x_cm, s.y_cm), 0.0, 0.5, "patinaje: la posicion no se mueve");
    }

    /* Sin giroscopio: el rumbo sale de los encoders. */
    {
        odom_t s; odom_init(&s, NULL);
        uint64_t ts = 1000000;
        odom_update(&s, ts, 0, 0, 0.0f, false);
        double ls = 0, rs = 0;
        for (int i = 0; i < 100; ++i) {
            ts += 20000;
            ls += 20.0 * 0.02 * deg_per_cm(&s) * 1.0; rs += 20.0 * 0.02 * deg_per_cm(&s);
            odom_update(&s, ts, (float)ls, (float)rs, 0.0f, false);
        }
        NEAR(s.y_cm, 40.0, 0.3, "sin giroscopio: avance recto");
    }

    /* Lecturas absurdas y huecos largos. */
    {
        odom_t s; odom_init(&s, NULL);
        uint64_t ts = 1000000;
        odom_update(&s, ts, 0, 0, 0.0f, true);
        ts += 20000;
        odom_update(&s, ts, 1.0e6f, 1.0e6f, 0.0f, true);          /* salto del contador */
        CHECK(s.rejected == 1, "salto imposible rechazado");
        NEAR(s.path_cm, 0.0, 1e-3, "salto imposible no suma camino");
        ts += 2000000;                                             /* hueco de 2 s */
        const float before = s.y_cm;
        odom_update(&s, ts, 2.0e6f, 2.0e6f, 0.0f, true);
        NEAR(s.y_cm, before, 1e-3, "hueco largo no se integra");
        CHECK(odom_update(&s, ts + 1000, NAN, 0.0f, 0.0f, true) == 0.0f && s.rejected == 2, "NaN rechazado");
    }

    /* Signos de montaje: un motor al reves. */
    {
        odom_config_t c; odom_config_default(&c);
        c.right_sign = -1;
        odom_t s; odom_init(&s, &c);
        uint64_t ts = 1000000;
        odom_update(&s, ts, 0, 0, 0.0f, true);
        double a = 0;
        for (int i = 0; i < 50; ++i) {
            ts += 20000; a += 0.4 * deg_per_cm(&s);
            odom_update(&s, ts, (float)a, (float)-a, 0.0f, true);
        }
        NEAR(s.y_cm, 20.0, 0.3, "motor derecho invertido");
    }

    /* Calibracion de distancia: el encoder dice 100 cm y el LiDAR 94. */
    {
        odom_config_t c; odom_config_default(&c);
        CHECK(odom_calibrate_distance(&c, 100.0f, 94.0f), "calibra distancia");
        NEAR(c.left_scale, 0.94, 1e-4, "escala izquierda");
        CHECK(!odom_calibrate_distance(&c, 5.0f, 5.0f), "recorrido corto no calibra");
        CHECK(!odom_calibrate_distance(&c, 100.0f, 300.0f), "factor incoherente no calibra");
        CHECK(!odom_calibrate_distance(&c, 100.0f, -94.0f), "sentido contrario no calibra");
    }
    /* Calibracion de la separacion: 360 grados de giro con diferencia de encoders = 12 * 2 * pi. */
    {
        odom_config_t c; odom_config_default(&c);
        CHECK(odom_calibrate_track(&c, 12.8f * 2.0f * 3.14159265f, 360.0f), "calibra separacion");
        NEAR(c.track_width_cm, 12.8, 1e-3, "separacion calibrada");
        CHECK(!odom_calibrate_track(&c, 5.0f, 20.0f), "giro corto no calibra");
    }
    /* Balance: izquierda 100, derecha 96 y el giroscopio dice que no giro. */
    {
        odom_config_t c; odom_config_default(&c);
        CHECK(odom_calibrate_balance(&c, 100.0f, 96.0f, 0.0f), "calibra balance");
        const double l2 = 100.0 * c.left_scale, r2 = 96.0 * c.right_scale;
        NEAR(l2, r2, 0.05, "tras el balance, los dos lados coinciden");
        CHECK(!odom_calibrate_balance(&c, 10.0f, 10.0f, 0.0f), "recta corta no calibra");
    }

    /* Eje comun: el giroscopio permite tomar curvas sin inventar patinaje. */
    {
        odom_t s; odom_init(&s, NULL);
        uint64_t ts = 1000000;
        odom_update_axle(&s, ts, 0.0f, 0.0f, true);
        const float turn = (float)(10.0 * deg_per_cm(&s));
        ts += 100000;
        NEAR(odom_update_axle(&s, ts, turn, 90.0f, true), 10.0, 0.01, "eje comun avanza 10 cm");
        NEAR(s.x_cm, 7.071, 0.03, "eje comun integra rumbo medio x");
        NEAR(s.y_cm, 7.071, 0.03, "eje comun integra rumbo medio y");
        CHECK(s.slip_count == 0, "un solo encoder no inventa patinaje");
        ts += 100000;
        NEAR(odom_update_axle(&s, ts, 0.0f, 90.0f, true), -10.0, 0.01, "eje comun marcha atras");
    }

    /* Configuracion fuera de rango. */
    {
        odom_config_t c; odom_config_default(&c);
        c.wheel_diameter_cm = -3.0f; c.track_width_cm = NAN; c.left_sign = 7;
        CHECK(odom_config_sanitize(&c), "sanea");
        CHECK(c.wheel_diameter_cm >= 1.0f && c.track_width_cm >= 3.0f && c.left_sign == 1, "valores acotados");
    }

    if (failures) { printf("odometria: %d fallos\n", failures); return 1; }
    printf("PASS: odometria (recta, arco, pivote, reversa, patinaje, sin giroscopio, lecturas absurdas, calibracion)\n");
    return 0;
}

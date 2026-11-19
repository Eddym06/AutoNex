/* Campo de coste del cajon: se puede aparcar y salir con el chasis 17 x 13, sin choques. */
#include "park_planner.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static float frand(float lo, float hi) { return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX); }

static bool body_hits(const park_params_t *p, const park_pose_t *pose, float margin)
{
    float c[4][2];
    park_body_corners(p, pose, c);
    /* esquinas y puntos intermedios de cada lado contra la pared y los topes (sin holgura) */
    for (int e = 0; e < 4; ++e)
        for (int s = 0; s <= 8; ++s) {
            const float t = (float)s / 8.0f;
            const float u = c[e][0] + (c[(e + 1) % 4][0] - c[e][0]) * t, w = c[e][1] + (c[(e + 1) % 4][1] - c[e][1]) * t;
            if (park_model_distance(p, u, w) < margin) {
                const float half = (p->lot_length_cm > 0 ? p->lot_length_cm : 1.5f * p->robot_length_cm) * 0.5f;
                const bool inside_limiter = (fabsf(u) > half && fabsf(u) < half + 2.0f && w > 0 && w < 20.0f);
                if (w < margin || inside_limiter || park_model_distance(p, u, w) < margin) return true;
            }
        }
    return false;
}

/* Ejecuta la politica con un modelo de movimiento ruidoso. Devuelve los pasos usados o -1. */
static int run(const park_params_t *p, park_pose_t pose, float pos_noise, float yaw_noise, bool *touched)
{
    park_move_t prev = PARK_MOVE_NONE;
    *touched = false;
    for (int step = 0; step < 600; ++step) {
        const park_decision_t d = park_field_decide(&pose, prev);
        if (d.at_goal) return step;
        if (d.move == PARK_MOVE_NONE) return -1;
        prev = d.move;
        if (d.move == PARK_MOVE_FORWARD || d.move == PARK_MOVE_BACK) {
            /* el robot se alinea con el rumbo del bin y avanza el vector entero de la retícula */
            float diff = d.target_theta_deg - pose.theta_deg;
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            if (fabsf(diff) > 2.0f) { pose.theta_deg = d.target_theta_deg; continue; }
            float du, dw;
            park_heading_step(park_heading_nearest(d.target_theta_deg), &du, &dw);
            const float sg = d.move == PARK_MOVE_FORWARD ? 1.0f : -1.0f;
            pose.u_cm += sg * du + frand(-pos_noise, pos_noise);
            pose.w_cm += sg * dw + frand(-pos_noise, pos_noise);
        } else {
            pose.theta_deg = d.target_theta_deg + frand(-yaw_noise, yaw_noise);
        }
        if (body_hits(p, &pose, 0.0f)) *touched = true;
    }
    return -1;
}

int main(void)
{
    park_params_t p;
    park_params_default(&p, 13.0f, 17.0f, 7.0f);
    assert(park_field_build(&p, PARK_GOAL_IN));
    assert(park_heading_count() == 32);
    bool has_zero = false;
    for (size_t k = 0; k < park_heading_count(); ++k) if (fabsf(park_heading_deg(k)) < 1e-3f) has_zero = true;
    assert(has_zero);                                       /* hay un rumbo exactamente paralelo a la pared */

    /* Meta: dentro y paralelo; coste 0. */
    park_pose_t goal = {-1.5f, 10.0f, 0.0f};
    assert(park_field_free(&goal));
    assert(fabsf(park_field_cost(&goal)) < 1e-3f);
    park_decision_t d = park_field_decide(&goal, PARK_MOVE_NONE);
    assert(d.at_goal && d.move == PARK_MOVE_NONE);

    /* Desde el carril, en paralelo y delante del cajon, hay camino. */
    const float starts[][2] = {{-35, 36}, {-30, 40}, {-25, 34}, {-15, 32}, {-5, 36}, {0, 44}, {-38, 45}};
    for (size_t n = 0; n < sizeof(starts) / sizeof(starts[0]); ++n) {
        park_pose_t s = {starts[n][0], starts[n][1], 0.0f};
        assert(park_field_cost(&s) > 0.0f);
        bool touched;
        const int steps = run(&p, s, 0.0f, 0.0f, &touched);
        assert(steps > 0);
        assert(!touched);
    }
    /* Con ruido de movimiento (0,4 cm, 1,5 grados) sigue llegando. */
    srand(7);
    int ok = 0, total = 0, hit = 0;
    for (int t = 0; t < 60; ++t) {
        park_pose_t s = {frand(-38, -5), frand(30, 46), frand(-8, 8)};
        bool touched;
        const int steps = run(&p, s, 0.4f, 1.5f, &touched);
        ++total; if (steps > 0) ++ok; if (touched) ++hit;
    }
    printf("aparcar con ruido: %d/%d llegan, %d rozan\n", ok, total, hit);
    assert(ok >= total * 9 / 10);

    /* Salir: desde el centro del cajon, paralelo, hay camino hasta el carril. */
    assert(park_field_build(&p, PARK_GOAL_OUT));
    park_pose_t inside = {-1.5f, 12.5f, 0.0f};   /* el robot arranca pegado al lado del carril: centrado (w = 10) no puede girar */
    assert(park_field_cost(&inside) > 0.0f);
    bool touched;
    const int steps = run(&p, inside, 0.0f, 0.0f, &touched);
    assert(steps > 0 && !touched);
    printf("salir: %d movimientos; aparcar con ruido OK\n", steps);

    /* Un cajon demasiado corto para el robot: no hay forma de entrar. */
    park_params_t tight = p;
    tight.lot_length_cm = 18.0f;
    assert(!park_field_build(&tight, PARK_GOAL_IN) || park_field_cost(&(park_pose_t){-30, 38, 0}) < 0.0f);
    puts("PASS: campo de parqueo (entrar en paralelo y salir del cajon con el chasis 17 x 13, sin chocar, con ruido)");
    return 0;
}

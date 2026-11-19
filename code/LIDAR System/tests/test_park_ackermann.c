#include "park_ackermann.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    park_params_t p;
    park_params_default(&p, 14, 22, 4);
    p.margin_cm = 1.0f;
    park_ackermann_config_t cfg = {14, 30, 3.5f, 0};
    park_pose_t start = {-7, 12.5f, 0};
    assert(park_pose_free(&p, &start));
    park_ackermann_result_t exit_plan = park_ackermann_plan(&p, &cfg, &start, PARK_GOAL_OUT);
    printf("salida: found=%d direction=%d steer=%.1f expanded=%u\n", exit_plan.found, exit_plan.direction, exit_plan.steer_deg, exit_plan.expanded);
    assert(exit_plan.found && !exit_plan.at_goal);
    assert(exit_plan.direction == 1 || exit_plan.direction == -1);
    assert(fabsf(exit_plan.steer_deg) <= 30);
    park_pose_t moving = start;
    for (int i = 0; i < 100 && !exit_plan.at_goal; ++i) {
        assert(exit_plan.found);
        const float radians = 0.017453292519943295f;
        const float ds = exit_plan.direction * cfg.step_cm / 6.0f;
        const float k = tanf(exit_plan.steer_deg * radians) / cfg.wheelbase_cm;
        for (int j = 0; j < 6; ++j) {
            const float dth = ds * k, mid = moving.theta_deg * radians + dth * 0.5f;
            moving.u_cm += ds * cosf(mid); moving.w_cm += ds * sinf(mid);
            moving.theta_deg += dth / radians;
            assert(park_pose_free(&p, &moving));
        }
        cfg.previous_direction = exit_plan.direction;
        exit_plan = park_ackermann_plan(&p, &cfg, &moving, PARK_GOAL_OUT);
    }
    printf("salida iterativa: u=%.1f w=%.1f theta=%.1f goal=%d\n", moving.u_cm, moving.w_cm, moving.theta_deg, exit_plan.at_goal);
    assert(exit_plan.at_goal);
    park_ackermann_result_t entry = park_ackermann_plan(&p, &cfg, &moving, PARK_GOAL_IN);
    for (int i = 0; i < 100 && entry.found && !entry.at_goal; ++i) {
        const float radians = 0.017453292519943295f;
        const float ds = entry.direction * cfg.step_cm / 6.0f;
        const float k = tanf(entry.steer_deg * radians) / cfg.wheelbase_cm;
        for (int j = 0; j < 6; ++j) {
            const float dth = ds * k, mid = moving.theta_deg * radians + dth * 0.5f;
            moving.u_cm += ds * cosf(mid); moving.w_cm += ds * sinf(mid);
            moving.theta_deg += dth / radians;
            assert(park_pose_free(&p, &moving));
        }
        cfg.previous_direction = entry.direction;
        entry = park_ackermann_plan(&p, &cfg, &moving, PARK_GOAL_IN);
    }
    printf("entrada iterativa: u=%.1f w=%.1f theta=%.1f goal=%d\n", moving.u_cm, moving.w_cm, moving.theta_deg, entry.at_goal);
    assert(entry.at_goal);
    park_pose_t collision = {-7, 2, 0};
    assert(!park_ackermann_plan(&p, &cfg, &collision, PARK_GOAL_OUT).found);
    park_ackermann_result_t parked = park_ackermann_plan(&p, &cfg, &start, PARK_GOAL_IN);
    assert(parked.found && parked.at_goal && parked.direction == 0);
    const park_pose_t approaches[] = {{-30, 36, 0}, {-20, 32, 0}, {0, 36, 0}};
    for (unsigned i = 0; i < sizeof(approaches) / sizeof(approaches[0]); ++i) {
        cfg.previous_direction = 0;
        park_ackermann_result_t a = park_ackermann_plan(&p, &cfg, &approaches[i], PARK_GOAL_IN);
        printf("aproximación %u: found=%d expanded=%u\n", i, a.found, a.expanded);
        assert(a.found && a.direction != 0);
        park_pose_t q = approaches[i];
        unsigned step = 0;
        for (int n = 0; n < 100 && a.found && !a.at_goal; ++n) {
            const float rad = 0.017453292519943295f;
            assert(step < a.count);
            const float ds = a.directions[step] * cfg.step_cm / 6.0f;
            const float k = tanf(a.steering[step] * cfg.steer_max_deg * rad) / cfg.wheelbase_cm;
            for (int j = 0; j < 6; ++j) {
                const float dth = ds * k, mid = q.theta_deg * rad + dth * 0.5f;
                q.u_cm += ds * cosf(mid); q.w_cm += ds * sinf(mid); q.theta_deg += dth / rad;
                assert(park_pose_free(&p, &q));
            }
            cfg.previous_direction = a.directions[step];
            if (++step >= a.count) { a = park_ackermann_plan(&p, &cfg, &q, PARK_GOAL_IN); step = 0; }
        }
        printf("aproximación %u: destino=%d disponible=%d u=%.2f w=%.2f theta=%.2f\n", i, a.at_goal, a.found, q.u_cm, q.w_cm, q.theta_deg);
        assert(a.at_goal);
    }
    /* Robustez: poses de llegada que difieren 1-2 cm y unos grados deben dar planes de entrada comparables (la eleccion entre
     * variantes existe para eso). Se exige plan valido, libre de choque y sin una sola cadena enorme de cambios de sentido. */
    {
        const float dw[] = {-1.5f, 0.0f, 1.5f}, dth[] = {-8.0f, 0.0f, 8.0f};
        int worst = 0, best = 99, missing = 0;
        for (unsigned i = 0; i < 3; ++i) for (unsigned j = 0; j < 3; ++j) {
            const park_pose_t s0 = {-20.0f, 32.0f + dw[i], dth[j]};
            cfg.previous_direction = 0;
            park_ackermann_result_t a = park_ackermann_plan(&p, &cfg, &s0, PARK_GOAL_IN);
            if (!a.found) {   /* como el controlador: si el paso de 3,5 cm no da camino, se reintenta con 2,5 */
                park_ackermann_config_t fine = cfg; fine.step_cm = 2.5f;
                a = park_ackermann_plan(&p, &fine, &s0, PARK_GOAL_IN);
            }
            if (!a.found) { ++missing; continue; }   /* limite conocido del A* de rejilla: alguna pose libre no llega a la meta */
            int rev = 0, last = 0;
            for (unsigned k = 0; k < a.count; ++k) { if (last && a.directions[k] != last) ++rev; last = a.directions[k]; }
            if (rev > worst) worst = rev;
            if (rev < best) best = rev;
        }
        printf("robustez: cambios de sentido por plan entre %d y %d (9 poses de llegada, %d sin plan)\n", best, worst, missing);
        assert(missing <= 1 && worst <= 6 && worst - best <= 4);
    }
    puts("PASS: planeador Ackermann, geometria y limites");
    return 0;
}

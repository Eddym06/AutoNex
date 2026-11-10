/* Prueba de it_plan: el planificador descubre una trayectoria valida y mas rapida que la linea central.
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -I "LIDAR System/main" "LIDAR System/tests/test_it_plan.c" "LIDAR System/main/it_plan.c" -lm -o it_test && ./it_test */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "it_plan.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { ++fails; printf("FALLA: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static it_plan_t plan;

static void check_plan(const char *name, int dir, const it_obstacle_t *obs, int count, float grip)
{
    it_config_t cfg;
    it_config_default(&cfg);
    cfg.grip = grip;
    clock_t t0 = clock();
    const bool ok = it_plan_build(&plan, &cfg, dir, obs, count);
    const double ms = 1000.0 * (double)(clock() - t0) / CLOCKS_PER_SEC;
    printf("%-28s dir %+d grip %.2f: valido=%d n=%d largo=%.0f cm tiempo=%.2f s (central %.2f s, ganancia %.0f%%) vmax=%.0f evaluaciones=%d %.0f ms\n", name, dir, grip,
           ok, plan.n, plan.length_cm, plan.time_s, plan.time_center_s, 100.0 * (1.0 - plan.time_s / plan.time_center_s),
           plan.v[0], plan.evals, ms);
    CHECK(ok, "%s: plan no valido (violacion %.3f)", name, plan.violation);
    if (!ok) return;
    const float kmax = tanf(cfg.steer_use * cfg.steer_max_deg * 0.0174533f) / cfg.wheelbase_cm;
    float vmax = 0.0f, vmin = 1e9f;
    for (int i = 0; i < plan.n; ++i) {
        CHECK(plan.d[i] >= plan.lo[i] - 1e-3f && plan.d[i] <= plan.hi[i] + 1e-3f, "%s: estacion %d fuera de su corredor", name, i);
        CHECK(plan.kappa[i] <= kmax * 1.25f, "%s: curvatura %.4f > maxima %.4f en %d", name, plan.kappa[i], kmax, i);
        if (plan.v[i] > vmax) vmax = plan.v[i];
        if (plan.v[i] < vmin) vmin = plan.v[i];
        CHECK(plan.v[i] <= cfg.v_max_cm_s + 1e-3f, "%s: velocidad sobre el tope", name);
    }
    /* Huella del chasis (rectangulo con su rumbo) libre de paredes y letreros en cada estacion. */
    float gap = 0.0f;
    CHECK(it_path_clear(&plan, &gap), "%s: la huella del chasis toca algo (holgura al letrero %.1f cm)", name, gap);
    if (count) printf("   holgura minima del chasis al letrero: %.1f cm\n", gap);
    /* Continuidad: el rumbo no da saltos entre estaciones vecinas (un salto es un cambio de carril imposible). */
    float maxturn = 0.0f;
    for (int i = 0; i < plan.n; ++i) {
        float dh = plan.heading_deg[(i + 1) % plan.n] - plan.heading_deg[i];
        while (dh > 180.0f) dh -= 360.0f;
        while (dh < -180.0f) dh += 360.0f;
        if (fabsf(dh) > maxturn) maxturn = fabsf(dh);
    }
    CHECK(maxturn < 25.0f, "%s: salto de rumbo de %.0f grados entre estaciones vecinas", name, maxturn);
    /* Perfil: aceleracion lateral dentro del agarre. */
    for (int i = 0; i < plan.n; ++i) {
        const float ay = plan.v[i] * plan.v[i] * plan.kappa[i];
        CHECK(ay <= cfg.mu * cfg.grip * 981.0f * 1.01f, "%s: a_lat %.0f sobre el agarre en %d", name, ay, i);
    }
    /* Regla de color: el plan pasa por el lado que toca (rojo a la derecha: la linea queda a la derecha del letrero, d mayor). */
    for (int j = 0; j < count; ++j) {
        if (obs[j].color == 0) continue;
        int best = 0;
        float bd = 1e9f;
        for (int i = 0; i < plan.n; ++i) {
            const float dd = hypotf(plan.x[i] - obs[j].x_cm, plan.z[i] - obs[j].z_cm);
            if (dd < bd) { bd = dd; best = i; }
        }
        const float hd = plan.heading_deg[best] * 0.0174533f;
        const float side = (obs[j].x_cm - plan.x[best]) * cosf(hd) + (obs[j].z_cm - plan.z[best]) * (-sinf(hd));   /* + : el letrero queda a la derecha de la linea */
        const bool want_right_of_line = obs[j].color == 1 ? false : true;     /* rojo: se pasa por su derecha => el letrero queda a la izquierda de la linea */
        CHECK((side > 0.0f) == want_right_of_line, "%s: letrero %d (color %d) por el lado equivocado (side %.1f, distancia %.1f)", name, j, obs[j].color, side, bd);
    }
    CHECK(plan.time_s <= plan.time_center_s + 0.01f, "%s: el plan no mejora la linea central", name);
    printf("   v: min %.0f max %.0f cm/s; holgura minima %.1f cm\n", vmin, vmax, ({ float m = 1e9f; for (int i = 0; i < plan.n; ++i) if (plan.room[i] < m) m = plan.room[i]; m; }));
}

int main(void)
{
    const it_obstacle_t none[1] = {{0, 0, 0}};
    const it_obstacle_t three[3] = {{-100.0f, 20.0f, 1}, {30.0f, -100.0f, 2}, {100.0f, 60.0f, 1}};
    const it_obstacle_t tight[2] = {{-100.0f, 0.0f, 2}, {-60.0f, 100.0f, 1}};
    for (int dir = -1; dir <= 1; dir += 2) {
        check_plan("pista vacia", dir, none, 0, 0.7f);
        check_plan("pista vacia grip 1", dir, none, 0, 1.0f);
        check_plan("3 letreros", dir, three, 3, 0.7f);
        check_plan("letreros pegados", dir, tight, 2, 0.7f);
    }
    /* Mas agarre => vuelta mas rapida. */
    it_config_t cfg; it_config_default(&cfg);
    float prev = 1e9f;
    for (float g = 0.4f; g <= 1.001f; g += 0.2f) {
        cfg.grip = g;
        it_plan_build(&plan, &cfg, -1, three, 3);
        printf("agarre %.1f -> vuelta %.2f s\n", g, plan.time_s);
        CHECK(plan.time_s <= prev + 0.05f, "mas agarre no baja el tiempo (%.2f > %.2f)", plan.time_s, prev);
        prev = plan.time_s;
    }
    /* Seguidor: sobre la propia linea la direccion es la de la curvatura y la velocidad la del perfil. */
    it_config_default(&cfg);
    it_plan_build(&plan, &cfg, -1, none, 0);
    it_tracker_t tr; it_tracker_reset(&tr);
    float st, sp;
    const int i0 = 20;
    const bool ok = it_track(&tr, &plan, plan.x[i0], plan.z[i0], plan.heading_deg[i0], plan.v[i0], &st, &sp);
    CHECK(ok, "el seguidor no engancha la linea");
    CHECK(fabsf(tr.e_cm) < 0.5f, "error lateral sobre la linea %.2f", tr.e_cm);
    CHECK(sp > 0.0f, "velocidad ordenada 0");
    printf(fails ? "FALLAN %d\n" : "PASS\n", fails);
    return fails ? 1 : 0;
}

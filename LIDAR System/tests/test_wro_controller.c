/* Configuracion del controlador: valores por defecto, texto, y los ajustes que
 * mantienen el rumbo del recorrido y la compensacion de latencia. */
#include "wro_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    wro_controller_config_t cfg;
    wro_controller_config_default(&cfg);
    assert(cfg.motion_comp && fabsf(cfg.latency_s - 0.10f) < 1e-6f);
    assert(cfg.corner_gyro_deg == 60.0f && cfg.wall_align_heading_deg == 15.0f);
    assert(cfg.stuck_relax_s == 0.0f);   /* el atasco no relaja el lado de paso salvo que se pida */

    char error[96];
    assert(wro_controller_config_parse_text(&cfg,
        "latency_s 0.2\nmotion_comp 0\ncorner_gyro_deg 0\nstuck_relax_s 8\nplanner.pass_side_red -1\n", error, sizeof(error)));
    assert(!cfg.motion_comp && fabsf(cfg.latency_s - 0.2f) < 1e-6f);
    assert(cfg.corner_gyro_deg == 0.0f && cfg.stuck_relax_s == 8.0f && cfg.planner.pass_side_red == -1);

    /* Valores imposibles se acotan, no se aceptan. */
    assert(wro_controller_config_parse_text(&cfg, "latency_s 9\nstuck_relax_cm 1\n", error, sizeof(error)));
    wro_controller_t c;
    wro_controller_init(&c, &cfg);
    assert(c.cfg.latency_s <= 0.5f && c.cfg.stuck_relax_cm >= 20.0f);

    /* Una clave desconocida no cambia nada. */
    wro_controller_config_t before = cfg;
    assert(!wro_controller_config_parse_text(&cfg, "latency_s 0.05\nno_existe 3\n", error, sizeof(error)));
    assert(memcmp(&before, &cfg, sizeof(cfg)) == 0);

    /* Linea de carrera: activa por defecto, se configura como el resto y los valores imposibles se acotan. */
    wro_controller_config_default(&cfg);
    assert(!cfg.race_enabled && cfg.race_speed_enabled && !cfg.race_lateral_enabled && cfg.race_after_corners == 4);
    assert(cfg.race.v_sign_cm_s == cfg.planner.speed_cm_s && cfg.race.v_max_cm_s > cfg.planner.speed_cm_s);
    assert(wro_controller_config_parse_text(&cfg,
        "race_enabled 1\nrace.v_max_cm_s 40\nrace.v_sign_cm_s 99\nrace.a_lat_cm_s2 1\nrace_gain_lap2 5\nplanner.plan_w_race 3\n",
        error, sizeof(error)));
    assert(cfg.race_enabled && cfg.race.v_max_cm_s == 40.0f && cfg.race.v_sign_cm_s == 40.0f);
    assert(cfg.race.a_lat_cm_s2 >= 10.0f && cfg.race_gain_lap2 == 1.0f && cfg.planner.plan_w_race == 3.0f);

    /* Pegado a la pared interior: apagado por defecto y acotado. */
    wro_controller_config_default(&cfg);
    assert(!cfg.hug_enabled && cfg.hug_offset_cm == 20.0f && cfg.hug_stop_before_end == 2);
    assert(wro_controller_config_parse_text(&cfg, "hug_enabled 1\nhug_offset_cm 99\nhug_rate 0\nhug_detach_cm 1\n", error, sizeof(error)));
    assert(cfg.hug_enabled && cfg.hug_offset_cm == 35.0f && cfg.hug_rate >= 0.1f && cfg.hug_detach_cm >= 20.0f);

    char text[8192];
    wro_controller_config_default(&cfg);
    const size_t n = wro_controller_config_format(&cfg, text, sizeof(text));
    assert(n > 0 && strstr(text, "latency_s") && strstr(text, "corner_gyro_deg"));
    puts("PASS: configuracion del controlador (defectos, texto, limites, todo o nada)");
    return 0;
}

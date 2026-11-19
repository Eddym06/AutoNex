#include "p4_runtime.h"

#include <math.h>
#include <string.h>

void p4_runtime_init(p4_runtime_t *rt, const p4_hal_t *hal, const wro_controller_config_t *cfg)
{
    if (!rt || !hal) return;
    memset(rt, 0, sizeof(*rt));
    rt->hal = *hal;
    wro_controller_init(&rt->controller, cfg);
    camera_table_decoder_reset(&rt->decoder);
    if (rt->hal.drive) rt->hal.drive(rt->hal.ctx, 0.0f, 0.0f, 0.0f);   /* arranca parado */
}

void p4_runtime_step(p4_runtime_t *rt)
{
    if (!rt || !rt->hal.now_us || !rt->hal.drive) return;
    const p4_hal_t *h = &rt->hal;
    float dps;
    uint64_t t_us;
    /* El giroscopio manda: es el que mas muestras da y el que ata el rumbo. */
    while (h->poll_gyro && h->poll_gyro(h->ctx, &dps, &t_us)) {
        if (!isfinite(dps) || !t_us || t_us > h->now_us(h->ctx) ||
            (rt->controller.last_imu_us && t_us <= rt->controller.last_imu_us)) continue;
        wro_controller_imu(&rt->controller, t_us, dps, NAN);
        ++rt->gyro_samples;
    }
    if (h->read_camera) {
        uint8_t chunk[128];
        const size_t n = h->read_camera(h->ctx, chunk, sizeof(chunk));
        camera_table_t table;
        if (n && camera_table_decoder_feed(&rt->decoder, chunk, n, h->now_us(h->ctx), &table)) {
            wro_controller_camera(&rt->controller, &table);
            ++rt->tables_ok;
        }
    }
    if (h->start_pressed && rt->controller.phase == WRO_PHASE_IDLE && h->start_pressed(h->ctx))
        wro_controller_start(&rt->controller, h->now_us(h->ctx));
    static local_lidar_point_t points[LOCAL_MAP_MAX_POINTS];
    size_t count = 0;
    const bool scan_received = h->poll_scan && h->poll_scan(h->ctx, points, LOCAL_MAP_MAX_POINTS, &count, &t_us);
    if (scan_received) {
        wro_controller_scan(&rt->controller, points, count, t_us, &rt->last);
        ++rt->scans;
    }
    /* Comprobar antes de enviar una orden nueva: un escaneo reciente no hace
     * seguro conducir con el giroscopio desconectado o sin muestras validas. */
    const uint64_t now = h->now_us(h->ctx);
    wro_command_t stop = rt->last;
    if (p4_runtime_guard(&rt->controller, now, &stop)) {
        rt->last = stop;
        h->drive(h->ctx, 0.0f, 0.0f, 0.0f);
    } else if (scan_received) {
        h->drive(h->ctx, rt->last.steer_deg, rt->last.speed_cm_s, rt->last.yaw_rate_dps);
    }
    else {
        /* Entre escaneos, el lazo rapido de la trayectoria inteligente (encoders + giroscopio). */
        wro_command_t fast;
        if (wro_controller_it_fast(&rt->controller, now, &fast)) {
            rt->last.steer_deg = fast.steer_deg;
            rt->last.speed_cm_s = fast.speed_cm_s;
            rt->last.yaw_rate_dps = fast.yaw_rate_dps;
            h->drive(h->ctx, fast.steer_deg, fast.speed_cm_s, fast.yaw_rate_dps);
        }
    }
}

bool p4_runtime_guard(wro_controller_t *c, uint64_t now, wro_command_t *command)
{
    if (!c || !command) return false;
    const bool active = c->phase != WRO_PHASE_IDLE && c->phase != WRO_PHASE_DONE;
    const uint64_t imu_us = c->last_imu_us;
    const bool gyro_stale = active && (!imu_us || imu_us > now || now - imu_us > c->cfg.sensor_timeout_us);
    if (!gyro_stale && !wro_controller_watchdog(c, now, command)) return false;
    command->speed_cm_s = 0.0f;
    command->steer_deg = 0.0f;
    command->yaw_rate_dps = 0.0f;
    return true;
}

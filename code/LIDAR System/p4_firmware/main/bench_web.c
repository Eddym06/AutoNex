#include "bench_web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw_encoders.h"
#include "hw_imu.h"
#include "hw_lidar.h"
#include "hw_motors.h"
#include "wro_config.h"

#if CONFIG_WRO_BENCH_WEB

#define DEADMAN_US 600000

static volatile float s_cmd_pct;          /* ultima orden: -100..100 */
static volatile int64_t s_cmd_time_us;    /* cuando llego */

static void apply_command(const char *line)
{
    if (line[0] == 'S' || line[0] == 's') {
        s_cmd_pct = 0.0f;
    } else if (line[0] == 'M' || line[0] == 'm') {
        float v = strtof(line + 1, NULL);
        if (v > 100.0f) v = 100.0f;
        if (v < -100.0f) v = -100.0f;
        s_cmd_pct = v;
    }
    s_cmd_time_us = esp_timer_get_time();
}

static void rx_task(void *arg)
{
    (void)arg;
    char line[32];
    int n = 0;
    uint8_t b;
    for (;;) {
        const int r = usb_serial_jtag_read_bytes(&b, 1, pdMS_TO_TICKS(50));
        if (r <= 0) continue;
        if (b == '\n' || b == '\r') {
            if (n > 0) { line[n] = 0; apply_command(line); }
            n = 0;
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)b;
        }
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    int32_t prev_counts = 0;
    int64_t prev_us = esp_timer_get_time();
    float applied = 0.0f;
    for (;;) {
        const int64_t now = esp_timer_get_time();
        /* Hombre muerto: sin orden reciente, el motor se para. */
        float want = s_cmd_pct;
        if (now - s_cmd_time_us > DEADMAN_US) want = 0.0f;
        if (want != applied || want != 0.0f) hw_motors_raw(0.01f * want);
        applied = want;

        int32_t counts = 0;
        const bool enc_ok = hw_encoders_counts(&counts);
        const float dt = (float)(now - prev_us) * 1e-6f;
        const float cps = dt > 0.0f ? (float)(counts - prev_counts) / dt : 0.0f;
        prev_counts = counts;
        prev_us = now;

        uint32_t imu_n = 0, imu_err = 0;
        float dps = 0.0f, acc[3] = {0, 0, 0};
        hw_imu_stats(&imu_n, &dps, &imu_err);
        const bool acc_ok = hw_imu_accel(acc);
        uint32_t pk = 0, chk = 0, rev = 0, pts = 0, drop = 0, raw = 0;
        uint8_t head[8];
        hw_lidar_stats(&pk, &chk, &rev, &pts, &drop);
        hw_lidar_raw(&raw, head);

        printf("$T {\"t\":%lld,\"enc_ok\":%d,\"enc\":%ld,\"cps\":%.1f,\"cmd\":%.0f,\"imu_n\":%lu,\"imu_err\":%lu,"
               "\"dps\":%.2f,\"acc_ok\":%d,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
               "\"lid_raw\":%lu,\"lid_pk\":%lu,\"lid_chk\":%lu,\"lid_rev\":%lu,\"lid_pts\":%lu}\n",
               (long long)(now / 1000), enc_ok ? 1 : 0, (long)counts, cps, applied,
               (unsigned long)imu_n, (unsigned long)imu_err, dps, acc_ok ? 1 : 0, acc[0], acc[1], acc[2],
               (unsigned long)raw, (unsigned long)pk, (unsigned long)chk, (unsigned long)rev, (unsigned long)pts);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void bench_web_start(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) return;
    xTaskCreate(rx_task, "bench_rx", 3072, NULL, 3, NULL);
    xTaskCreate(tx_task, "bench_tx", 4096, NULL, 3, NULL);
}

#else
void bench_web_start(void) {}
#endif

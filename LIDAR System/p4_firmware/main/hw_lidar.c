#include "hw_lidar.h"

#include <math.h>

#include <string.h>

#include "driver/uart.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "lidar";

#define LIDAR_BAUD        230400
#define LIDAR_HDR0        0xAAu
#define LIDAR_HDR1        0x55u
#define LIDAR_PREAMBLE    10u              /* AA 55 CT LSN FSA(2) LSA(2) CS(2) */
#define LIDAR_MAX_LSN     255u
#define LIDAR_PKT_MAX     (LIDAR_PREAMBLE + 3u * LIDAR_MAX_LSN)
#define LIDAR_MAX_RANGE_MM 12000u
#define LIDAR_RX_BUF      4096

static struct {
    bool installed;
    uint8_t pkt[LIDAR_PKT_MAX];
    uint16_t pkt_len;
    uint16_t pkt_expect;
    bool synced;
    local_lidar_point_t cur[LOCAL_MAP_MAX_POINTS];    /* vuelta en curso */
    size_t cur_n;
    local_lidar_point_t ready[LOCAL_MAP_MAX_POINTS];  /* ultima vuelta completa */
    size_t ready_n;
    bool ready_pending;
    uint64_t ready_t_us;
    uint32_t packets_ok, checksum_errors, revolutions, points, points_dropped;
    uint32_t raw_bytes;
    uint8_t raw_head[8];             /* primeros bytes crudos, para diagnosticar cableado y baudios */
} L;

static float s_cal_sign = (float)CONFIG_WRO_LIDAR_ANGLE_SIGN;
static float s_cal_offset_deg = (float)CONFIG_WRO_LIDAR_ANGLE_OFFSET_DEG;

static float wrap180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/* XOR de 16 bits del D6: 0x55AA, CT|LSN<<8, FSA, LSA y por muestra Si_L ^ (Si_H<<8 | Si_2nd). */
static bool checksum_ok(const uint8_t *p, uint8_t lsn)
{
    uint16_t cs = 0x55AAu;
    cs ^= (uint16_t)(p[2] | (p[3] << 8));
    cs ^= (uint16_t)(p[4] | (p[5] << 8));
    cs ^= (uint16_t)(p[6] | (p[7] << 8));
    for (uint8_t i = 0; i < lsn; ++i) {
        const uint8_t b = (uint8_t)(LIDAR_PREAMBLE + 3u * i);
        cs ^= p[b];
        cs ^= (uint16_t)((p[b + 2] << 8) | p[b + 1]);
    }
    return cs == (uint16_t)(p[8] | (p[9] << 8));
}

static void process_packet(const uint8_t *p, uint8_t lsn)
{
    const bool ring_start = (p[2] & 1u) != 0;
    if (ring_start) {
        if (L.synced && L.cur_n > 0) {
            memcpy(L.ready, L.cur, L.cur_n * sizeof L.cur[0]);
            L.ready_n = L.cur_n;
            L.ready_pending = true;
            L.ready_t_us = (uint64_t)esp_timer_get_time();
            ++L.revolutions;
        }
        L.cur_n = 0;
        L.synced = true;
    }
    if (!L.synced) return;               /* descartar hasta el primer inicio de vuelta */

    /* Angulos en centesimas de grado: (valor >> 1) * 100 / 64. */
    int32_t sa = ((int32_t)((p[4] | (p[5] << 8)) >> 1) * 100) / 64;
    int32_t ea = ((int32_t)((p[6] | (p[7] << 8)) >> 1) * 100) / 64;
    if (ea < sa) ea += 36000;
    const int32_t step = (lsn > 1) ? (ea - sa) / (lsn - 1) : 0;

    for (uint8_t i = 0; i < lsn; ++i) {
        const uint8_t b = (uint8_t)(LIDAR_PREAMBLE + 3u * i);
        const uint32_t dist_mm = (uint32_t)p[b + 2] * 64u + (uint32_t)(p[b + 1] >> 2);
        ++L.points;
        if (dist_mm == 0 || dist_mm > LIDAR_MAX_RANGE_MM) continue;   /* 0 = sin retorno */
        if (L.cur_n >= LOCAL_MAP_MAX_POINTS) { ++L.points_dropped; continue; }

        const int32_t a = (sa + step * (int32_t)i) % 36000;
        local_lidar_point_t *out = &L.cur[L.cur_n++];
        out->angle_deg = wrap180(s_cal_sign * ((float)a / 100.0f) + s_cal_offset_deg);
        out->distance_cm = (float)dist_mm / 10.0f;
        out->intensity = (float)p[b];
        out->valid = true;
        out->blind = false;
    }
}

/* Calibracion del angulo: angulo del robot = sentido * angulo del LiDAR + desfase (0 = frente, derecha positiva).
 * Los valores por defecto vienen de menuconfig; la calibracion guiada de la pagina web los guarda en NVS. */

void hw_lidar_calibration_get(float *sign, float *offset_deg)
{
    *sign = s_cal_sign;
    *offset_deg = s_cal_offset_deg;
}

void hw_lidar_calibration_set(float sign, float offset_deg)
{
    s_cal_sign = sign < 0.0f ? -1.0f : 1.0f;
    s_cal_offset_deg = wrap180(offset_deg);
    nvs_handle_t h;
    if (nvs_open("lidar_cal", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i8(h, "sign", (int8_t)s_cal_sign);
        nvs_set_i32(h, "off_c", (int32_t)lroundf(s_cal_offset_deg * 100.0f));
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "calibracion guardada: sentido %+d, desfase %.1f grados", (int)s_cal_sign, s_cal_offset_deg);
}

static void calibration_load(void)
{
    nvs_handle_t h;
    int8_t sign;
    int32_t off_c;
    if (nvs_open("lidar_cal", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_i8(h, "sign", &sign) == ESP_OK && nvs_get_i32(h, "off_c", &off_c) == ESP_OK) {
        s_cal_sign = sign < 0 ? -1.0f : 1.0f;
        s_cal_offset_deg = (float)off_c / 100.0f;
        ESP_LOGI(TAG, "calibracion cargada: sentido %+d, desfase %.1f grados", (int)s_cal_sign, s_cal_offset_deg);
    }
    nvs_close(h);
}

/* Maquina de estados por bytes: busca AA 55, junta el paquete y valida el checksum. */
static void feed_byte(uint8_t b)
{
    if (L.pkt_len == 0) {
        if (b == LIDAR_HDR0) { L.pkt[0] = b; L.pkt_len = 1; }
        return;
    }
    if (L.pkt_len == 1) {
        if (b == LIDAR_HDR1) { L.pkt[1] = b; L.pkt_len = 2; }
        else if (b != LIDAR_HDR0) L.pkt_len = 0;
        return;
    }
    L.pkt[L.pkt_len++] = b;
    if (L.pkt_len == 4) L.pkt_expect = (uint16_t)(LIDAR_PREAMBLE + 3u * L.pkt[3]);
    if (L.pkt_len == L.pkt_expect) {
        const uint8_t lsn = L.pkt[3];
        L.pkt_len = 0;
        if (checksum_ok(L.pkt, lsn)) {
            ++L.packets_ok;
            process_packet(L.pkt, lsn);
        } else {
            ++L.checksum_errors;         /* resincronizar en el siguiente AA 55 */
        }
    }
}

void hw_lidar_init(void)
{
    memset(&L, 0, sizeof L);
    calibration_load();
    if (CONFIG_WRO_LIDAR_RX_GPIO < 0) {
        ESP_LOGW(TAG, "sin LiDAR: WRO_LIDAR_RX_GPIO = -1");
        return;
    }
    const uart_port_t port = (uart_port_t)CONFIG_WRO_LIDAR_UART_NUM;
    const uart_config_t cfg = {
        .baud_rate = LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, LIDAR_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, UART_PIN_NO_CHANGE, CONFIG_WRO_LIDAR_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    L.installed = true;
    ESP_LOGI(TAG, "UART%d a %d baudios, RX en GPIO %d", (int)port, LIDAR_BAUD, CONFIG_WRO_LIDAR_RX_GPIO);
}

bool hw_lidar_poll_scan(void *ctx, local_lidar_point_t *points, size_t capacity, size_t *count, uint64_t *t_us)
{
    (void)ctx;
    if (!L.installed) return false;
    uint8_t chunk[256];
    int n;
    while ((n = uart_read_bytes((uart_port_t)CONFIG_WRO_LIDAR_UART_NUM, chunk, sizeof chunk, 0)) > 0) {
        for (int i = 0; i < n; ++i) {
            if (L.raw_bytes < sizeof L.raw_head) L.raw_head[L.raw_bytes] = chunk[i];
            ++L.raw_bytes;
            feed_byte(chunk[i]);
        }
    }
    if (!L.ready_pending) return false;
    const size_t n_out = L.ready_n < capacity ? L.ready_n : capacity;
    memcpy(points, L.ready, n_out * sizeof points[0]);
    *count = n_out;
    *t_us = L.ready_t_us;
    L.ready_pending = false;
    return true;
}

void hw_lidar_raw(uint32_t *bytes, uint8_t head[8])
{
    *bytes = L.raw_bytes;
    memcpy(head, L.raw_head, sizeof L.raw_head);
}

void hw_lidar_stats(uint32_t *packets_ok, uint32_t *checksum_errors, uint32_t *revolutions, uint32_t *points,
                    uint32_t *points_dropped)
{
    *packets_ok = L.packets_ok;
    *checksum_errors = L.checksum_errors;
    *revolutions = L.revolutions;
    *points = L.points;
    *points_dropped = L.points_dropped;
}

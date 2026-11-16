/* Laboratorio de camara (solo con CONFIG_WRO_CAM_LAB): mide el sensor y vuelca imagenes por UART0 a 2 Mbaud.
 * Lineas de texto empiezan por '#'. Imagenes: 'F','R','M','1' + seq(u16) + w(u16) + h(u16) + len(u32) + datos RGB565. */
#include "sdkconfig.h"

#if CONFIG_WRO_CAM_LAB
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LAB_UART UART_NUM_0
#ifndef LAB_BAUD
#define LAB_BAUD 921600
#endif

static void out(const char *s) { uart_write_bytes(LAB_UART, s, strlen(s)); }
static void outf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void outf(const char *fmt, ...)
{
    char b[300];
    va_list a;
    va_start(a, fmt);
    vsnprintf(b, sizeof b, fmt, a);
    va_end(a);
    out(b);
}

static int rd(sensor_t *s, int reg) { return s->get_reg(s, reg, 0xFF); }
static int rd16(sensor_t *s, int reg) { return (rd(s, reg) << 8) | rd(s, reg + 1); }
static int exposure_lines(sensor_t *s)
{
    return ((rd(s, 0x3500) & 0x0F) << 12) | (rd(s, 0x3501) << 4) | (rd(s, 0x3502) >> 4);
}

static void dump_regs(sensor_t *s, const char *tag)
{
    outf("#REG [%s] total=%dx%d out=%dx%d exposure=%d lineas gain_reg=0x%X aec_mode=%02X pll=%02X/%02X/%02X/%02X pclk_ratio=%02X\n",
         tag, rd16(s, 0x380C), rd16(s, 0x380E), rd16(s, 0x3808), rd16(s, 0x380A), exposure_lines(s), rd16(s, 0x350A) & 0x3FF,
         rd(s, 0x3503), rd(s, 0x3034), rd(s, 0x3035), rd(s, 0x3036), rd(s, 0x3037), rd(s, 0x3824));
}

static inline int g6(const uint8_t *p) { return (((p[0] << 8) | p[1]) >> 5) & 63; }   /* canal verde de RGB565, 0..63 */

/* Mide periodo (reloj del sensor), brillo medio (verde 0..63) y ruido temporal (|dif| entre cuadros consecutivos). */
static void measure(sensor_t *s, const char *tag, int ms)
{
    int64_t t_end = esp_timer_get_time() + (int64_t)ms * 1000, prev = 0, first = 0, last = 0, mn = INT64_MAX, mx = 0;
    int n = 0, fails = 0, flat = 0;
    double luma = 0, noise = 0;
    int noise_n = 0;
    static uint8_t ref[160 * 120 * 2];
    bool have_ref = false;
    while (esp_timer_get_time() < t_end) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { ++fails; continue; }
        const int64_t t = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
        if (prev) { const int64_t d = t - prev; if (d < mn) mn = d; if (d > mx) mx = d; }
        if (!first) first = t;
        last = t; prev = t; ++n;
        int same = 1, sum = 0, cnt = 0, nd = 0;
        for (size_t i = 0; i + 1 < fb->len && i < sizeof ref; i += 16) {
            const int g = g6(fb->buf + i);
            sum += g; ++cnt;
            if (g != g6(fb->buf)) same = 0;
            if (have_ref) nd += abs(g - g6(ref + i));
        }
        if (cnt) { luma += (double)sum / cnt; if (have_ref) { noise += (double)nd / cnt; ++noise_n; } }
        flat += same;
        if (fb->len <= sizeof ref) { memcpy(ref, fb->buf, fb->len); have_ref = true; }
        esp_camera_fb_return(fb);
    }
    if (n > 1 && last > first)
        outf("#FPS %-34s fps=%5.1f periodo=%5.2f/%5.2f ms cuadros=%3d fallos=%d planos=%d brillo=%4.1f/63 ruido=%.2f exp=%d\n",
             tag, 1e6 * (n - 1) / (double)(last - first), mn / 1000.0, mx / 1000.0, n, fails, flat, luma / n,
             noise_n ? noise / noise_n : 0.0, exposure_lines(s));
    else
        outf("#FPS %-34s SIN CUADROS (fallos=%d)\n", tag, fails);
}

static void send_frame(const uint8_t *buf, size_t len, int w, int h, uint16_t seq)
{
    uint8_t hd[14] = {'F', 'R', 'M', '1', (uint8_t)seq, (uint8_t)(seq >> 8), (uint8_t)w, (uint8_t)(w >> 8),
                      (uint8_t)h, (uint8_t)(h >> 8), (uint8_t)len, (uint8_t)(len >> 8), (uint8_t)(len >> 16), (uint8_t)(len >> 24)};
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += buf[i];
    uart_write_bytes(LAB_UART, hd, sizeof hd);
    uart_write_bytes(LAB_UART, buf, len);
    const uint8_t tail[4] = {(uint8_t)sum, (uint8_t)(sum >> 8), (uint8_t)(sum >> 16), (uint8_t)(sum >> 24)};
    uart_write_bytes(LAB_UART, tail, sizeof tail);
}

#define BURST 4
static uint8_t *s_burst[BURST];

/* Puntuacion de corrupcion: media de |verde(x,y) - verde(x,y-1)| (limpio < ~2; filas desalineadas > ~6). */
static double frame_score(const uint8_t *b, int w, int h)
{
    double acc = 0; int n = 0;
    for (int y = 1; y < h; ++y)
        for (int x = 0; x < w; x += 2) {
            acc += abs(g6(b + 2 * (y * w + x)) - g6(b + 2 * ((y - 1) * w + x)));
            ++n;
        }
    return n ? acc / n : 0;
}

static void burst_and_send(sensor_t *s, uint16_t seq0, const char *tag, bool send)
{
    int got = 0, w = 0, h = 0;
    size_t len = 0;
    int64_t t0 = 0, t1 = 0;
    dump_regs(s, tag);
    while (got < BURST) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        if (!s_burst[got]) s_burst[got] = heap_caps_malloc(fb->len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_burst[got]) { esp_camera_fb_return(fb); break; }
        memcpy(s_burst[got], fb->buf, fb->len);
        w = fb->width; h = fb->height; len = fb->len;
        const int64_t t = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
        if (!got) t0 = t;
        t1 = t;
        esp_camera_fb_return(fb);
        ++got;
    }
    char sc[96] = "";
    for (int i = 0; i < got; ++i) {
        char one[16];
        snprintf(one, sizeof one, " %.1f", frame_score(s_burst[i], w, h));
        strcat(sc, one);
    }
    outf("#BURST %s cuadros=%d en %.1f ms  puntuacion:%s\n", tag, got, (t1 - t0) / 1000.0, sc);
    if (send) for (int i = 0; i < got; ++i) send_frame(s_burst[i], len, w, h, seq0 + i);
    outf("#BURST_FIN %s\n", tag);
}

static void set_total_x(sensor_t *s, int tx)
{
    s->set_reg(s, 0x380C, 0xFF, tx >> 8);
    s->set_reg(s, 0x380D, 0xFF, tx & 0xFF);
}

static void set_base_pll(sensor_t *s) { s->set_pll(s, 0, 21, 1, 0, 0, 0, 1, 14); }

void cam_lab_run(void)
{
    uart_driver_install(LAB_UART, 2048, 8192, 0, NULL, 0);
    uart_set_baudrate(LAB_UART, LAB_BAUD);
    esp_log_level_set("*", ESP_LOG_NONE);
    vTaskDelay(pdMS_TO_TICKS(1500));
    sensor_t *s = esp_camera_sensor_get();
    out("#LAB inicio\n");
    vTaskDelay(pdMS_TO_TICKS(2500));
    dump_regs(s, "frio");
    uint16_t seq = 0;
    out("#LAB rafagas continuas: agita el objeto de lado a lado\n");
    for (;;) {
        set_total_x(s, 2300);
        vTaskDelay(pdMS_TO_TICKS(600));
        burst_and_send(s, seq, "tx2300", true);
        seq += BURST;
        set_total_x(s, 1500);
        vTaskDelay(pdMS_TO_TICKS(600));
        burst_and_send(s, seq, "tx1500", true);
        seq += BURST;
    }
    /* A: sensor sin tocar */
    for (int i = 0; i < 10; ++i) { burst_and_send(s, seq, "A-frio-2300", i < 2); if (i < 2) seq += BURST; vTaskDelay(pdMS_TO_TICKS(i < 2 ? 800 : 300)); }
    /* B: cambia total_x a 1500 (una sola vez) */
    set_total_x(s, 1500);
    vTaskDelay(pdMS_TO_TICKS(1500));
    for (int i = 0; i < 10; ++i) { burst_and_send(s, seq, "B-1500", i < 2); if (i < 2) seq += BURST; vTaskDelay(pdMS_TO_TICKS(i < 2 ? 800 : 300)); }
    /* C: vuelve a 2300 */
    set_total_x(s, 2300);
    vTaskDelay(pdMS_TO_TICKS(1500));
    for (int i = 0; i < 10; ++i) { burst_and_send(s, seq, "C-2300-tras-1500", i < 2); if (i < 2) seq += BURST; vTaskDelay(pdMS_TO_TICKS(i < 2 ? 800 : 300)); }
    /* D: reinicia el sensor completo (framesize de nuevo) */
    s->set_framesize(s, FRAMESIZE_QQVGA);
    vTaskDelay(pdMS_TO_TICKS(1500));
    for (int i = 0; i < 10; ++i) { burst_and_send(s, seq, "D-tras-set_framesize", i < 2); if (i < 2) seq += BURST; vTaskDelay(pdMS_TO_TICKS(i < 2 ? 800 : 300)); }
    out("#LAB fin\n");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
#endif

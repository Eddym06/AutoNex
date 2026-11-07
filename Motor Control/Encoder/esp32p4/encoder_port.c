/*
 * encoder_port.c - ver encoder_port.h
 *
 * Arquitectura:
 *   GPIO (ambos flancos) -> ISR minima: lee esp_timer (us) y el nivel -> cola
 *   cola -> tarea de alta prioridad: enc_on_edge() (+ calibracion si esta activa)
 *   encoder_port_get() -> copia el estado bajo un spinlock y llama a enc_estimate()
 *
 * Con el reloj de 1 us, una velocidad maxima ~0.7 m/s (5 ms entre flancos) y
 * jitter de ISR de pocos us, la incertidumbre de tiempo es < 0.1 % del
 * intervalo: no es el factor limitante (ver README).
 */
#include "encoder_port.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

static const char *TAG = "encoder";

#define ENC_QUEUE_LEN   128
#define ENC_NVS_NS      "encoder"

typedef struct {
    uint64_t t_us;
    uint8_t  level;
} edge_evt_t;

struct encoder_port {
    encoder_port_config_t cfg;
    enc_t           core;
    enc_cal_t       cal;
    bool            calibrating;
    bool            cal_done;
    enc_cal_report_t cal_rep;
    volatile int    dir;
    volatile bool   zero_req;
    QueueHandle_t   queue;
    TaskHandle_t    task;
    portMUX_TYPE    lock;
    volatile uint32_t dropped;
};

/* ------------------------------------------------------------------ */
static void IRAM_ATTR encoder_isr(void *arg)
{
    encoder_port_t *p = (encoder_port_t *)arg;
    edge_evt_t ev;
    ev.t_us  = (uint64_t)esp_timer_get_time();
    ev.level = (uint8_t)gpio_get_level(p->cfg.gpio);
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(p->queue, &ev, &woken) != pdTRUE) p->dropped++;
    if (woken) portYIELD_FROM_ISR();
}

/* ------------------------------------------------------------------ */
static esp_err_t table_load(encoder_port_t *p)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ENC_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    float frac[ENC_EDGES];
    size_t len = sizeof frac;
    err = nvs_get_blob(h, p->cfg.nvs_key, frac, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof frac) return ESP_FAIL;
    for (int i = 0; i < ENC_EDGES; i++)                 /* sanidad: fracciones plausibles */
        if (!(frac[i] > 0.4f / ENC_EDGES && frac[i] < 1.8f / ENC_EDGES)) return ESP_ERR_INVALID_CRC;
    portENTER_CRITICAL(&p->lock);
    enc_set_table(&p->core, frac);
    portEXIT_CRITICAL(&p->lock);
    return ESP_OK;
}

static esp_err_t table_save(encoder_port_t *p, const float frac[ENC_EDGES])
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ENC_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, p->cfg.nvs_key, frac, sizeof(float) * ENC_EDGES);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* ------------------------------------------------------------------ */
static void encoder_task(void *arg)
{
    encoder_port_t *p = (encoder_port_t *)arg;
    edge_evt_t ev;
    for (;;) {
        if (xQueueReceive(p->queue, &ev, portMAX_DELAY) != pdTRUE) continue;

        portENTER_CRITICAL(&p->lock);
        if (p->zero_req) { enc_reset(&p->core); p->zero_req = false; }
        const bool ok = enc_on_edge(&p->core, ev.t_us, ev.level, p->dir);
        if (ok && p->calibrating) {
            enc_cal_feed_from(&p->cal, &p->core);
            if (p->core.d.n > ENC_CAL_MAX_REVS * ENC_EDGES) p->cal_done = true;
        }
        portEXIT_CRITICAL(&p->lock);

        if (p->calibrating && p->cal_done) {       /* cálculo pesado fuera de la sección crítica */
            float frac[ENC_EDGES];
            enc_cal_report_t rep;
            const bool good = enc_cal_finish(&p->cal, frac, &rep, p->cfg.harm_cut);
            p->cal_rep = rep;
            p->cal_rep.ok = good;
            if (good) {
                portENTER_CRITICAL(&p->lock);
                enc_set_table(&p->core, frac);
                enc_reset(&p->core);
                portEXIT_CRITICAL(&p->lock);
                if (table_save(p, frac) != ESP_OK) ESP_LOGW(TAG, "no se pudo guardar la tabla en NVS");
                ESP_LOGI(TAG, "calibracion OK: %d vueltas usadas, dispersion max %.2f %%",
                         rep.revs_used, 100.0f * rep.spread_max);
            } else {
                ESP_LOGW(TAG, "calibracion RECHAZADA: vueltas completas %d, usadas %d, dispersion max %.1f %%",
                         rep.revs_complete, rep.revs_used, 100.0f * rep.spread_max);
            }
            p->calibrating = false;
        }
    }
}

/* ------------------------------------------------------------------ */
esp_err_t encoder_port_init(encoder_port_t **out, const encoder_port_config_t *cfg)
{
    if (!out || !cfg) return ESP_ERR_INVALID_ARG;
    encoder_port_t *p = calloc(1, sizeof *p);
    if (!p) return ESP_ERR_NO_MEM;
    p->cfg  = *cfg;
    p->dir  = 1;
    p->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    enc_config_t ec;
    enc_config_default(&ec);
    ec.wheel_radius_m = cfg->wheel_radius_m;
    enc_init(&p->core, &ec);

    p->queue = xQueueCreate(ENC_QUEUE_LEN, sizeof(edge_evt_t));
    if (!p->queue) { free(p); return ESP_ERR_NO_MEM; }
    if (xTaskCreate(encoder_task, "encoder", 4096, p, configMAX_PRIORITIES - 2, &p->task) != pdPASS) {
        vQueueDelete(p->queue); free(p); return ESP_ERR_NO_MEM;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << cfg->gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = cfg->pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;   /* ya instalado: ok */
    err = gpio_isr_handler_add(cfg->gpio, encoder_isr, p);
    if (err != ESP_OK) return err;

    if (table_load(p) == ESP_OK) ESP_LOGI(TAG, "tabla de calibracion cargada de NVS (%s)", cfg->nvs_key);
    else                         ESP_LOGW(TAG, "sin tabla de calibracion: usando tabla uniforme (calibrar!)");
    *out = p;
    return ESP_OK;
}

void encoder_port_set_direction(encoder_port_t *p, int dir) { p->dir = dir >= 0 ? 1 : -1; }

void encoder_port_zero(encoder_port_t *p) { p->zero_req = true; }

void encoder_port_get(encoder_port_t *p, enc_est_t *out)
{
    enc_t snap;
    uint64_t now;
    portENTER_CRITICAL(&p->lock);
    now  = (uint64_t)esp_timer_get_time();
    snap = p->core;                         /* ~1 kB: unos pocos us */
    portEXIT_CRITICAL(&p->lock);
    enc_estimate(&snap, now, out);
}

esp_err_t encoder_port_calibrate_start(encoder_port_t *p)
{
    if (p->calibrating) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&p->lock);
    enc_config_t ec = p->core.cfg;
    enc_init(&p->core, &ec);                /* tabla uniforme, n = 0 */
    enc_cal_begin(&p->cal);
    p->cal_done    = false;
    p->calibrating = true;
    portEXIT_CRITICAL(&p->lock);
    ESP_LOGI(TAG, "calibrando: mantener velocidad constante ~%d vueltas", ENC_CAL_MAX_REVS);
    return ESP_OK;
}

bool encoder_port_calibrate_poll(encoder_port_t *p, enc_cal_report_t *rep)
{
    if (p->calibrating) return false;
    if (rep) *rep = p->cal_rep;
    return true;
}

bool encoder_port_has_table(const encoder_port_t *p)  { return p->core.cal_valid; }
bool encoder_port_is_aligned(const encoder_port_t *p) { return p->core.aligned; }

void encoder_port_stats(encoder_port_t *p, uint32_t *accepted, uint32_t *glitches, uint32_t *missed)
{
    if (accepted) *accepted = p->core.accepted;
    if (glitches) *glitches = p->core.glitches;
    if (missed)   *missed   = p->core.missed;
}

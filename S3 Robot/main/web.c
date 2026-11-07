#include "web.h"

#include <stdlib.h>
#include <string.h>

#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#ifndef WRO_OTA_TOKEN
#define WRO_OTA_TOKEN ""            /* sin clave: la actualizacion por WiFi queda desactivada */
#endif

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "web";
static web_api_t s_api;
static volatile int64_t s_last_poll_us;
static char *s_json;
static uint8_t *s_img;
#define IMG_CAP 24576
#define JSON_CAP 16384

/* page.html se incrusta en el firmware (EMBED_TXTFILES): termina con un NUL que no se envia. */
extern const char page_start[] asm("_binary_page_html_start");
extern const char page_end[] asm("_binary_page_html_end");

static esp_err_t h_root(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    return httpd_resp_send(r, page_start, page_end - page_start - 1);
}

static esp_err_t h_state(httpd_req_t *r)
{
    s_last_poll_us = esp_timer_get_time();
    int len = 0;
    if (!s_api.get_state_json(s_json, JSON_CAP, &len)) return httpd_resp_send_500(r);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, s_json, len);
}

static esp_err_t h_cam(httpd_req_t *r)
{
    const int n = s_api.get_cam_jpeg(s_img, IMG_CAP);
    if (n <= 0) return httpd_resp_send_err(r, HTTPD_404_NOT_FOUND, "sin imagen todavia");
    httpd_resp_set_type(r, "image/jpeg");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, (const char *)s_img, n);
}

static esp_err_t h_start(httpd_req_t *r) { s_last_poll_us = esp_timer_get_time(); s_api.request_start(); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_cal_front(httpd_req_t *r) { s_last_poll_us = esp_timer_get_time(); s_api.request_cal(0); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_cal_right(httpd_req_t *r) { s_last_poll_us = esp_timer_get_time(); s_api.request_cal(1); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_cal_pitch(httpd_req_t *r) { s_last_poll_us = esp_timer_get_time(); s_api.request_cal(3); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_stop(httpd_req_t *r) { s_api.request_stop(); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_motor1(httpd_req_t *r) { s_api.request_motor(1); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_motor2(httpd_req_t *r) { s_api.request_motor(2); return httpd_resp_sendstr(r, "ok"); }
static esp_err_t h_motor_seq(httpd_req_t *r) { s_api.request_motor(3); return httpd_resp_sendstr(r, "ok"); }

/* Actualizacion por WiFi: curl -X POST -H "X-Token: <clave>" --data-binary @build/wro_s3.bin http://<IP>/ota */
static esp_err_t h_ota(httpd_req_t *r)
{
    char tok[48] = "";
    if (!WRO_OTA_TOKEN[0] || httpd_req_get_hdr_value_str(r, "X-Token", tok, sizeof tok) != ESP_OK ||
        strcmp(tok, WRO_OTA_TOKEN) != 0)
        return httpd_resp_send_err(r, HTTPD_401_UNAUTHORIZED, "clave incorrecta");
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t h;
    uint8_t *buf = malloc(4096);
    if (!part || !buf || esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK) {
        free(buf);
        return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo iniciar la actualizacion");
    }
    int left = r->content_len;
    while (left > 0) {
        const int n = httpd_req_recv(r, (char *)buf, left < 4096 ? left : 4096);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0 || esp_ota_write(h, buf, n) != ESP_OK) {
            esp_ota_abort(h);
            free(buf);
            return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "recepcion fallida");
        }
        left -= n;
    }
    free(buf);
    if (esp_ota_end(h) != ESP_OK || esp_ota_set_boot_partition(part) != ESP_OK)
        return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "imagen invalida");
    httpd_resp_sendstr(r, "ok, reiniciando");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

int64_t web_last_poll_us(void) { return s_last_poll_us; }

void web_start(const web_api_t *api)
{
    s_api = *api;
    s_json = malloc(JSON_CAP);
    s_img = malloc(IMG_CAP);
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 6144;
    cfg.max_uri_handlers = 16;
    cfg.recv_wait_timeout = 15;
    httpd_handle_t srv;
    if (!s_json || !s_img || httpd_start(&srv, &cfg) != ESP_OK) { ESP_LOGE(TAG, "no se pudo iniciar el servidor web"); return; }
    const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = h_root},
        {.uri = "/api/state", .method = HTTP_GET, .handler = h_state},
        {.uri = "/cam.jpg", .method = HTTP_GET, .handler = h_cam},
        {.uri = "/api/start", .method = HTTP_POST, .handler = h_start},
        {.uri = "/api/stop", .method = HTTP_POST, .handler = h_stop},
        {.uri = "/api/cal_front", .method = HTTP_POST, .handler = h_cal_front},
        {.uri = "/api/cal_right", .method = HTTP_POST, .handler = h_cal_right},
        {.uri = "/api/cal_pitch", .method = HTTP_POST, .handler = h_cal_pitch},
        {.uri = "/api/motor1", .method = HTTP_POST, .handler = h_motor1},
        {.uri = "/api/motor2", .method = HTTP_POST, .handler = h_motor2},
        {.uri = "/api/motor_seq", .method = HTTP_POST, .handler = h_motor_seq},
        {.uri = "/ota", .method = HTTP_POST, .handler = h_ota},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); ++i) httpd_register_uri_handler(srv, &uris[i]);
    ESP_LOGI(TAG, "panel web en el puerto 80");
}

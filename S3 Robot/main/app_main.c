/* Firmware del ESP32-S3 CAM del robot: camara + puente WiFi con el P4.
 *   - Vision: detecta letreros y manda la tabla al P4 por UART0 (921600), como siempre.
 *   - Enlace de control (shared/link_protocol.h): recibe del P4 el estado y el barrido del LiDAR, y le manda las
 *     ordenes de iniciar/parar y el latido de la pagina web.
 *   - WiFi + pagina web (INICIAR / PARAR, zona del LiDAR, estado) + actualizacion por WiFi (OTA).
 * Los logs salen por el USB nativo (USB-Serial-JTAG): la UART0 es solo del cable al P4. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_source.h"
#include "img_converters.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link_protocol.h"
#include "flip.h"
#include "nvs_flash.h"
#include "sensor_table_protocol.h"
#include "vision.h"
#include "vision_settings.h"
#include "vision_table.h"
#include "web.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#define WRO_WIFI_SSID ""
#define WRO_WIFI_PASS ""
#endif

static const char *TAG = "wro_s3";
#define LINK_UART UART_NUM_0
#define LINK_BAUD 921600
#define STATE_TIMEOUT_US 1000000
/* Giro de la imagen de la camara: X = espejo izquierda-derecha, Y = arriba-abajo; los dos juntos = 180 grados.
 * Constantes de codigo (no de Kconfig) para que siempre se compilen. Un espejo suelto cambiaria el lado de los letreros. */
#define CAM_FLIP_X 0
#define CAM_FLIP_Y 1

static SemaphoreHandle_t s_tx_lock, s_state_lock;
static EXT_RAM_BSS_ATTR link_state_t s_state;
static int64_t s_state_us;                       /* instante del ultimo estado valido del P4 */
static uint32_t s_frames_ok;
static volatile uint8_t s_cmd;                   /* orden pendiente (se repite unos instantes) */
static volatile int64_t s_cmd_until_us;
static volatile uint8_t s_seq;                   /* cambia con cada orden nueva del usuario */
static char s_ip[16] = "-";
static SemaphoreHandle_t s_cam_lock;             /* protege el ultimo JPEG y la ultima deteccion de la camara */
static uint8_t *s_jpg;
static size_t s_jpg_len;
static wro_sensor_table_t s_vis;
static int64_t s_vis_us;
static volatile float s_cam_fps, s_vis_ms, s_jpg_ms, s_fb_ms;   /* rendimiento de la camara (ventana de 60 cuadros) */
static volatile bool s_wifi_pause;               /* diagnostico: sin reintentos mientras se escanea */

static const char *const MSG_TEXT[LINK_MSG_COUNT] = {
    [LINK_MSG_WAITING] = "En espera", [LINK_MSG_CALIBRATING] = "Calibrando giroscopio...",
    [LINK_MSG_DONE] = "Mision terminada", [LINK_MSG_FAULT] = "FALLA: parado", [LINK_MSG_STOPPED] = "Parado",
    [LINK_MSG_PIVOT] = "Giro en el sitio", [LINK_MSG_REVERSE] = "Voy en reversa",
    [LINK_MSG_CORNER_RIGHT] = "Esquina: giro a la derecha", [LINK_MSG_CORNER_LEFT] = "Esquina: giro a la izquierda",
    [LINK_MSG_STRAIGHT] = "Voy recto",
    [LINK_MSG_RED_RIGHT] = "Esquivando letrero rojo: derecha", [LINK_MSG_RED_LEFT] = "Esquivando letrero rojo: izquierda",
    [LINK_MSG_GREEN_RIGHT] = "Esquivando letrero verde: derecha", [LINK_MSG_GREEN_LEFT] = "Esquivando letrero verde: izquierda",
    [LINK_MSG_TURN_RIGHT] = "Giro a la derecha", [LINK_MSG_TURN_LEFT] = "Giro a la izquierda",
};

/* ---------- Enlace con el P4 (UART0) ---------- */
static void link_write(const uint8_t *data, size_t len)
{
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);       /* una trama entera a la vez */
    uart_write_bytes(LINK_UART, data, len);
    xSemaphoreGive(s_tx_lock);
}

static void link_rx_task(void *arg)
{
    (void)arg;
    static link_decoder_t dec;
    static EXT_RAM_BSS_ATTR link_state_t tmp;
    uint8_t buf[256];
    for (;;) {
        const int n = uart_read_bytes(LINK_UART, buf, sizeof buf, pdMS_TO_TICKS(20));
        for (int i = 0; i < n; ++i) {
            if (link_decoder_feed(&dec, buf[i]) != LINK_TYPE_STATE) continue;
            if (!link_decode_state(dec.payload, dec.payload_len, &tmp)) continue;
            xSemaphoreTake(s_state_lock, portMAX_DELAY);
            memcpy(&s_state, &tmp, sizeof tmp);
            s_state_us = esp_timer_get_time();
            ++s_frames_ok;
            xSemaphoreGive(s_state_lock);
        }
    }
}

/* Cada 100 ms: latido (la pagina esta viva) y la orden pendiente. */
static void link_cmd_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const int64_t now = esp_timer_get_time();
        link_cmd_t c = {
            .cmd = now < s_cmd_until_us ? s_cmd : LINK_CMD_NONE,
            .seq = s_seq,
            .web_alive = web_last_poll_us() && now - web_last_poll_us() < 3000000,
        };
        uint8_t frame[16];
        const size_t n = link_encode_cmd(&c, frame, sizeof frame);
        if (n) link_write(frame, n);
    }
}

static int cam_get_jpeg(uint8_t *out, int cap)
{
    int n = 0;
    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
    if (s_jpg && (int)s_jpg_len <= cap) { memcpy(out, s_jpg, s_jpg_len); n = (int)s_jpg_len; }
    xSemaphoreGive(s_cam_lock);
    return n;
}

static void send_cmd(uint8_t cmd)
{
    s_cmd = cmd;
    ++s_seq;
    s_cmd_until_us = esp_timer_get_time() + 600000;   /* se repite 0,6 s: el P4 la toma una vez aunque se pierda una trama */
}

static void api_start(void) { send_cmd(LINK_CMD_START); ESP_LOGI(TAG, "INICIAR desde la pagina"); }
static void api_cal(int kind)
{
    send_cmd(kind == 0 ? LINK_CMD_CAL_FRONT : kind == 1 ? LINK_CMD_CAL_RIGHT : kind == 2 ? LINK_CMD_SET_NORTH : LINK_CMD_CAL_PITCH);
    ESP_LOGI(TAG, "orden: %s", kind == 0 ? "calibrar frente" : kind == 1 ? "calibrar derecha" : kind == 2 ? "fijar norte" : "calibrar inclinacion");
}
static void api_stop(void) { send_cmd(LINK_CMD_STOP); ESP_LOGW(TAG, "PARAR desde la pagina"); }
static void api_motor(int which)
{
    send_cmd(which == 1 ? LINK_CMD_MOTOR_M1 : which == 2 ? LINK_CMD_MOTOR_M2 : LINK_CMD_MOTOR_SEQ);
    ESP_LOGI(TAG, "prueba de motores desde la pagina: %s", which == 1 ? "M1" : which == 2 ? "M2" : "M1 y luego M2");
}

/* ---------- Estado para la pagina ---------- */
static bool state_json(char *out, int cap, int *len)
{
    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    const int64_t now = esp_timer_get_time();
    const bool linked = s_state_us && now - s_state_us < STATE_TIMEOUT_US;
    const link_state_t *s = &s_state;
    const char *msg = !s_state_us ? "Sin enlace con el P4 (revisa el cable TX/RX)" :
                      !linked ? "Se perdio el enlace con el P4" :
                      s->msg < LINK_MSG_COUNT ? MSG_TEXT[s->msg] : "?";
    int n = snprintf(out, cap,
        "{\"msg\":\"%s\",\"link\":%d,\"phase\":%d,\"armed\":%d,\"speed\":%.1f,\"yawrate\":%.1f,\"yaw\":%.1f,"
        "\"front\":%u,\"left\":%u,\"right\":%u,\"corners\":%u,\"laps\":%u,\"dir\":%d,\"elapsed\":%.1f,"
        "\"rev\":%lu,\"age\":%d,\"imu\":%.2f,\"encl\":%.3f,\"encr\":%.3f,\"tables\":%lu,\"cf\":%.1f,\"cv\":%.1f,\"cj\":%.1f,\"cb\":%.1f,\"gb\":%.2f,\"hh\":%.1f,\"lv\":%u,\"pit\":%.1f,\"pcal\":%d,\"mh\":%.1f,\"me\":%.0f,\"mn\":%.0f,\"mep\":%u,\"camflip\":%d,\"calmsg\":%u,\"calsign\":%d,\"caloff\":%.1f,\"mt\":%u,\"cam\":[",
        msg, linked, s->phase, s->armed, s->speed_cm_s, s->yaw_rate_dps, s->yaw_deg,
        s->front_cm, s->left_cm, s->right_cm, s->corners, s->laps, s->direction, s->elapsed_s,
        (unsigned long)s->lidar_revs, s->lidar_age_ms == 0xFFFF ? -1 : (int)s->lidar_age_ms, s->imu_dps,
        s->enc_left_m, s->enc_right_m, (unsigned long)s->tables_ok, (double)s_cam_fps, (double)s_vis_ms, (double)s_jpg_ms, (double)s_fb_ms, s->gyro_bias_dps, s->height_cm, s->level, s->pitch_deg, s->pitch_cal, s->map_heading_deg, s->map_e_cm, s->map_n_cm, s->map_epoch, CAM_FLIP_X * 2 + CAM_FLIP_Y, s->cal_msg, s->cal_sign, s->cal_offset_deg, s->motor_test);
    for (uint8_t i = 0; i < s->cam_count && n < cap - 256; ++i)
        n += snprintf(out + n, cap - n, "%s[%.0f,%u,%u]", i ? "," : "", s->cam[i].bearing_deg, s->cam[i].distance_cm, s->cam[i].color);
    n += snprintf(out + n, cap - n, "],\"pts\":[");
    for (uint16_t i = 0; i < s->scan_count && n < cap - 32; ++i)
        n += snprintf(out + n, cap - n, "%s[%.0f,%u]", i ? "," : "", s->scan[i].angle_deg, s->scan[i].distance_cm);
    xSemaphoreGive(s_state_lock);
    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
    n += snprintf(out + n, cap - n, "],\"vis\":{\"age\":%d,\"or\":%u,\"bl\":%u,\"o\":[",
                  s_vis_us ? (int)((esp_timer_get_time() - s_vis_us) / 1000) : -1, s_vis.line_orange_row, s_vis.line_blue_row);
    for (uint8_t i = 0; i < s_vis.object_count && n < cap - 64; ++i)
        n += snprintf(out + n, cap - n, "%s[%u,%u,%u,%u]", i ? "," : "", s_vis.objects[i].color,
                      s_vis.objects[i].center_x_px, s_vis.objects[i].width_px, s_vis.objects[i].distance_mm);
    xSemaphoreGive(s_cam_lock);
    n += snprintf(out + n, cap - n, "]}}");
    *len = n < cap ? n : cap - 1;
    return n < cap;
}

/* ---------- Vision ---------- */
static void cam_task(void *arg)
{
    (void)arg;
    static EXT_RAM_BSS_ATTR vision_workspace_t workspace;
    vision_config_t config;
    vision_settings_current(&config);
    if (vision_camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "camara no disponible");
        vTaskDelete(NULL);
    }
    vision_camera_apply_settings(&config.camera);
    uint16_t seq = 0;
    int64_t last_jpg_us = 0, win_t0 = esp_timer_get_time();
    int win_frames = 0;
    double win_vis = 0.0, win_jpg = 0.0, win_fb = 0.0;
    int win_jpgn = 0;
    for (;;) {
        const int64_t t_fb0 = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        win_fb += (double)(esp_timer_get_time() - t_fb0) / 1000.0;
        if (fb->format == PIXFORMAT_RGB565) {
            /* camara montada boca abajo: la vision y la pagina reciben la imagen ya corregida */
            flip_rgb565(fb->buf, fb->width, fb->height, CAM_FLIP_X, CAM_FLIP_Y);
            vision_result_t r = {.action = V_STOP};
            const int64_t t_v0 = esp_timer_get_time();
            vision_process_cfg(fb->buf, fb->len, fb->width, fb->height, false, &config, &workspace, &r);
            win_vis += (double)(esp_timer_get_time() - t_v0) / 1000.0;
            wro_sensor_table_t table;
            vision_table_build(&r, fb->width, seq++, (uint32_t)(esp_timer_get_time() / 1000), &table);
            xSemaphoreTake(s_cam_lock, portMAX_DELAY);
            s_vis = table;
            s_vis_us = esp_timer_get_time();
            xSemaphoreGive(s_cam_lock);
            if (esp_timer_get_time() - last_jpg_us > 250000) {             /* ~4 cuadros/s para la pagina */
                last_jpg_us = esp_timer_get_time();
                uint8_t *jpg = NULL;
                size_t jpg_len = 0;
                const int64_t t_j0 = esp_timer_get_time();
                const bool jpg_ok = frame2jpg(fb, 85, &jpg, &jpg_len);
                win_jpg += (double)(esp_timer_get_time() - t_j0) / 1000.0;
                ++win_jpgn;
                if (jpg_ok) {
                    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
                    free(s_jpg);
                    s_jpg = jpg;
                    s_jpg_len = jpg_len;
                    xSemaphoreGive(s_cam_lock);
                }
            }
            uint8_t pkt[WRO_TABLE_MAX_SIZE];
            const size_t len = wro_sensor_table_encode(&table, pkt, sizeof pkt);
            if (len) link_write(pkt, len);
        }
        esp_camera_fb_return(fb);
        if (++win_frames >= 60) {
            const int64_t now = esp_timer_get_time();
            s_cam_fps = (float)win_frames * 1e6f / (float)(now - win_t0);
            s_vis_ms = (float)(win_vis / win_frames);
            s_fb_ms = (float)(win_fb / win_frames);
            s_jpg_ms = win_jpgn ? (float)(win_jpg / win_jpgn) : 0.0f;
            win_t0 = now; win_frames = 0; win_vis = win_jpg = win_fb = 0.0; win_jpgn = 0;
        }
    }
}

/* ---------- WiFi ---------- */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        ESP_LOGW(TAG, "WiFi sin conexion con \"%s\" (motivo %d)", WRO_WIFI_SSID, d->reason);
        if (!s_wifi_pause) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        snprintf(s_ip, sizeof s_ip, IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "WiFi conectado: abre http://%s/", s_ip);
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, CONFIG_WRO_WIFI_HOSTNAME);
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, WRO_WIFI_SSID, sizeof wc.sta.ssid);
    strlcpy((char *)wc.sta.password, WRO_WIFI_PASS, sizeof wc.sta.password);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable = true;
    wc.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (!WRO_WIFI_SSID[0]) ESP_LOGW(TAG, "sin SSID: crea main/wifi_secrets.h a partir del ejemplo");
}

/* Diagnostico: si no hay IP a los 8 s, lista las redes de 2,4 GHz que ve el S3. */
static void wifi_diag_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(8000));
    if (s_ip[0] != '-') vTaskDelete(NULL);
    s_wifi_pause = true;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(1500));
    const esp_err_t se = esp_wifi_scan_start(NULL, true);
    if (se != ESP_OK) ESP_LOGW(TAG, "escaneo fallo: %s", esp_err_to_name(se));
    else {
        uint16_t n = 20;
        wifi_ap_record_t ap[20];
        esp_wifi_scan_get_ap_records(&n, ap);
        for (uint16_t i = 0; i < n; ++i)
            ESP_LOGI(TAG, "red visible: \"%s\" canal %d senal %d dBm seguridad %d", (char *)ap[i].ssid, ap[i].primary, ap[i].rssi, ap[i].authmode);
        if (!n) ESP_LOGW(TAG, "no se ve ninguna red de 2,4 GHz");
    }
    s_wifi_pause = false;
    esp_wifi_connect();
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "WRO S3: camara + puente WiFi con el P4 (cable en UART0, %d baudios)", LINK_BAUD);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    s_tx_lock = xSemaphoreCreateMutex();
    s_state_lock = xSemaphoreCreateMutex();
    s_cam_lock = xSemaphoreCreateMutex();

    const uart_config_t uc = {
        .baud_rate = LINK_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 8192, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &uc));
    /* Con la consola en el USB nativo, GPIO43/44 no quedan como UART0 solos: hay que enrutarlos. */
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vision_settings_init();
    wifi_start();
    static const web_api_t api = {.get_state_json = state_json, .get_cam_jpeg = cam_get_jpeg, .request_start = api_start, .request_cal = api_cal, .request_stop = api_stop, .request_motor = api_motor};
    web_start(&api);
    xTaskCreate(wifi_diag_task, "wifi_diag", 4096, NULL, 3, NULL);
    xTaskCreatePinnedToCore(link_rx_task, "link_rx", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(link_cmd_task, "link_cmd", 3072, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(cam_task, "vision", 8192, NULL, 5, NULL, 1);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        const int64_t age_ms = s_state_us ? (esp_timer_get_time() - s_state_us) / 1000 : -1;
        ESP_LOGI(TAG, "IP %s | enlace P4: %lu tramas, ultima hace %lld ms | fase=%u armado=%d msg=%u",
                 s_ip, (unsigned long)s_frames_ok, (long long)age_ms, s_state.phase, s_state.armed, s_state.msg);
        xSemaphoreGive(s_state_lock);
    }
}

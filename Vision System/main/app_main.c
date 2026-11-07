#include "camera_source.h"
#include "vision.h"
#include "vision_config_rx.h"
#include "vision_settings.h"
#include "vision_uart_link.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#define PERF_WINDOW_FRAMES 100
static vision_workspace_t workspace;
static vision_config_t config;
static const char *TAG = "wro_vision";
void app_main(void)
{
    vision_settings_init();
    vision_settings_current(&config);
#if CONFIG_WRO_LOCK_AUTO_AFTER_FRAMES > 0
    if (config.camera.lock_after_frames == 0)
        config.camera.lock_after_frames = CONFIG_WRO_LOCK_AUTO_AFTER_FRAMES;
#endif
    esp_err_t err = vision_camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STOP: camera init failed: %s", esp_err_to_name(err));
        return;
    }
    vision_camera_apply_settings(&config.camera);
    err = vision_uart_link_init();
    const bool link_ready = err == ESP_OK;
    if (err != ESP_OK) ESP_LOGE(TAG, "Camera data link unavailable: %s", esp_err_to_name(err));
    err = vision_config_rx_start();
    if (err != ESP_OK) ESP_LOGW(TAG, "Live configuration unavailable: %s", esp_err_to_name(err));
    bool parking = false;
#ifdef CONFIG_WRO_PARKING_MODE
    parking = true;
#endif
    unsigned perf_frames = 0, perf_processed = 0;
    unsigned period_samples = 0;
    unsigned sensor_samples = 0;
    int64_t previous_cycle_us = 0, previous_sensor_us = 0;
    int64_t period_total_us = 0, capture_total_us = 0, vision_total_us = 0;
    int64_t sensor_period_total_us = 0;
    int64_t sensor_period_min_us = INT64_MAX, sensor_period_max_us = 0;
    uint16_t table_sequence = 0;
    uint32_t frames_since_apply = 0;
    bool auto_locked = false;
    while (true) {
        /* A configuration change lands between frames, never inside one. */
        vision_config_t incoming;
        if (vision_settings_poll(&incoming)) {
            config = incoming;
            vision_camera_apply_settings(&config.camera);
            frames_since_apply = 0;
            auto_locked = false;
        }
        const int64_t cycle_start_us = esp_timer_get_time();
        if (previous_cycle_us) {
            period_total_us += cycle_start_us - previous_cycle_us;
            ++period_samples;
        }
        previous_cycle_us = cycle_start_us;
        vision_result_t r = {.action = V_STOP};
        const int64_t capture_start_us = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        const int64_t capture_us = esp_timer_get_time() - capture_start_us;
        int64_t vision_us = 0;
        int frame_width = VISION_WIDTH;
        uint32_t frame_timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        capture_total_us += capture_us;
        if (fb) {
            const int64_t sensor_frame_us = (int64_t)fb->timestamp.tv_sec * 1000000LL +
                                            fb->timestamp.tv_usec;
            if (previous_sensor_us && sensor_frame_us > previous_sensor_us) {
                const int64_t sensor_period_us = sensor_frame_us - previous_sensor_us;
                sensor_period_total_us += sensor_period_us;
                if (sensor_period_us < sensor_period_min_us) sensor_period_min_us = sensor_period_us;
                if (sensor_period_us > sensor_period_max_us) sensor_period_max_us = sensor_period_us;
                ++sensor_samples;
            }
            previous_sensor_us = sensor_frame_us;
            frame_width = fb->width;
            frame_timestamp_ms = (uint32_t)(sensor_frame_us / 1000);
            if (fb->format == PIXFORMAT_RGB565) {
                const int64_t vision_start_us = esp_timer_get_time();
                const bool valid = vision_process_cfg(fb->buf, fb->len, fb->width, fb->height,
                    parking, &config, &workspace, &r);
                if (valid) {
                    /* El S3 envia una propuesta basada en este cuadro. El P4
                     * es el unico que orquesta la maniobra con LiDAR/IMU y
                     * controla el movimiento; no aplicar aqui otra secuencia
                     * temporizada que pueda competir con la decision del P4. */
                    ++perf_processed;
                }
                vision_us = esp_timer_get_time() - vision_start_us;
            }
            esp_camera_fb_return(fb);
            if (!auto_locked && config.camera.lock_after_frames &&
                ++frames_since_apply >= config.camera.lock_after_frames) {
                vision_camera_lock_auto();
                auto_locked = true;
            }
        }
        if (link_ready) {
            const esp_err_t link_err = vision_uart_link_send(&r, frame_width,
                table_sequence++, frame_timestamp_ms);
            if (link_err != ESP_OK) ESP_LOGW(TAG, "Camera table send failed: %s", esp_err_to_name(link_err));
        }
        vision_total_us += vision_us;
        if (++perf_frames == PERF_WINDOW_FRAMES) {
            const double avg_period_ms = period_samples
                ? (double)period_total_us / period_samples / 1000.0 : 0.0;
            const double avg_capture_ms = (double)capture_total_us / perf_frames / 1000.0;
            const double avg_vision_ms = (double)vision_total_us / perf_frames / 1000.0;
            const double avg_sensor_ms = sensor_samples
                ? (double)sensor_period_total_us / sensor_samples / 1000.0 : 0.0;
            const double sensor_fps = avg_sensor_ms > 0.0 ? 1000.0 / avg_sensor_ms : 0.0;
            const double fps = avg_period_ms > 0.0 ? 1000.0 / avg_period_ms : 0.0;
            ESP_LOGI(TAG, "PERF frames=%u processed=%u loop_fps=%.2f loop_ms=%.2f sensor_fps=%.2f sensor_ms=%.2f sensor_min_ms=%.2f sensor_max_ms=%.2f fb_get_ms=%.2f vision_ms=%.2f",
                perf_frames, perf_processed, fps, avg_period_ms, sensor_fps, avg_sensor_ms,
                (double)sensor_period_min_us / 1000.0, (double)sensor_period_max_us / 1000.0,
                avg_capture_ms, avg_vision_ms);
            perf_frames = 0;
            perf_processed = 0;
            period_samples = 0;
            sensor_samples = 0;
            period_total_us = 0;
            capture_total_us = 0;
            vision_total_us = 0;
            sensor_period_total_us = 0;
            sensor_period_min_us = INT64_MAX;
            sensor_period_max_us = 0;
        }
    }
}

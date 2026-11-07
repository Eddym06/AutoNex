#include "camera_source.h"
#include "camera_pins.h"
#include "esp_log.h"

static const char *TAG = "wro_camera";

esp_err_t vision_camera_init(void)
{
    camera_config_t c = {
        .pin_pwdn = CAM_PWDN, .pin_reset = CAM_RESET, .pin_xclk = CAM_XCLK,
        .pin_sccb_sda = CAM_SIOD, .pin_sccb_scl = CAM_SIOC,
        .pin_d0 = CAM_D0, .pin_d1 = CAM_D1, .pin_d2 = CAM_D2, .pin_d3 = CAM_D3,
        .pin_d4 = CAM_D4, .pin_d5 = CAM_D5, .pin_d6 = CAM_D6, .pin_d7 = CAM_D7,
        .pin_vsync = CAM_VSYNC, .pin_href = CAM_HREF, .pin_pclk = CAM_PCLK,
        .xclk_freq_hz = 16000000, .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0, .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QQVGA, .jpeg_quality = 12, .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_LATEST,
    };
    return esp_camera_init(&c);
}

esp_err_t vision_camera_apply_settings(const vision_camera_settings_t *k)
{
    if (!k || !k->apply) return ESP_OK;
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return ESP_ERR_INVALID_STATE;
    /* A driver that lacks a setter returns -1; the others are still applied. */
    int failures = 0;
    failures += s->set_exposure_ctrl(s, k->auto_exposure ? 1 : 0) != 0;
    if (!k->auto_exposure) failures += s->set_aec_value(s, k->exposure) != 0;
    else failures += s->set_ae_level(s, k->ae_level) != 0;
    failures += s->set_gain_ctrl(s, k->auto_gain ? 1 : 0) != 0;
    if (!k->auto_gain) failures += s->set_agc_gain(s, k->gain) != 0;
    failures += s->set_whitebal(s, k->auto_wb ? 1 : 0) != 0;
    if (!k->auto_wb) failures += s->set_awb_gain(s, 0) != 0;
    failures += s->set_wb_mode(s, k->wb_mode) != 0;
    failures += s->set_brightness(s, k->brightness) != 0;
    failures += s->set_contrast(s, k->contrast) != 0;
    failures += s->set_saturation(s, k->saturation) != 0;
    failures += s->set_hmirror(s, k->hmirror ? 1 : 0) != 0;
    failures += s->set_vflip(s, k->vflip ? 1 : 0) != 0;
    if (failures) ESP_LOGW(TAG, "%d camera setting(s) were rejected by the driver", failures);
    else ESP_LOGI(TAG, "camera settings applied (auto exposure=%d gain=%d wb=%d)",
                  k->auto_exposure, k->auto_gain, k->auto_wb);
    return ESP_OK;
}

esp_err_t vision_camera_lock_auto(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return ESP_ERR_INVALID_STATE;
    /* Switching the automatic controls off leaves the sensor running with the
     * values the controllers had reached. */
    int failures = 0;
    failures += s->set_exposure_ctrl(s, 0) != 0;
    failures += s->set_gain_ctrl(s, 0) != 0;
    failures += s->set_whitebal(s, 0) != 0;
    ESP_LOGI(TAG, "auto exposure / gain / white balance locked (%d failure(s))", failures);
    return failures ? ESP_FAIL : ESP_OK;
}

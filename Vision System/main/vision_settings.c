#include "vision_settings.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include "nvs_flash.h"

#define SETTINGS_NAMESPACE "wro_vision"
#define SETTINGS_KEY "config"

static const char *TAG = "wro_settings";
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static vision_config_t active;
static uint32_t revision, polled_revision;
static bool nvs_ready;

static esp_err_t load_saved(vision_config_t *out)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    uint8_t blob[VISION_CONFIG_WIRE_MAX];
    size_t size = sizeof(blob);
    err = nvs_get_blob(h, SETTINGS_KEY, blob, &size);
    nvs_close(h);
    if (err != ESP_OK) return err;
    return vision_config_decode(blob, size, out) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t vision_settings_init(void)
{
    vision_config_default(&active);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    nvs_ready = err == ESP_OK;
    if (nvs_ready) {
        vision_config_t saved;
        if (load_saved(&saved) == ESP_OK) {
            active = saved;
            ESP_LOGI(TAG, "using the configuration saved in flash");
        } else {
            ESP_LOGI(TAG, "no saved configuration: built-in defaults");
        }
    } else {
        ESP_LOGW(TAG, "NVS unavailable (%s): built-in defaults, nothing can be saved",
                 esp_err_to_name(err));
    }
    revision = polled_revision + 1u;
    return err;
}

void vision_settings_current(vision_config_t *out)
{
    portENTER_CRITICAL(&lock);
    *out = active;
    portEXIT_CRITICAL(&lock);
}

bool vision_settings_poll(vision_config_t *out)
{
    bool changed;
    portENTER_CRITICAL(&lock);
    changed = polled_revision != revision;
    if (changed) { *out = active; polled_revision = revision; }
    portEXIT_CRITICAL(&lock);
    return changed;
}

esp_err_t vision_settings_save(void)
{
    if (!nvs_ready) return ESP_ERR_INVALID_STATE;
    vision_config_t copy;
    vision_settings_current(&copy);
    uint8_t blob[VISION_CONFIG_WIRE_MAX];
    const size_t n = vision_config_encode(&copy, blob, sizeof(blob));
    if (!n) return ESP_ERR_INVALID_SIZE;
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, SETTINGS_KEY, blob, n);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) ESP_LOGI(TAG, "configuration saved (%u bytes)", (unsigned)n);
    return err;
}

void vision_settings_submit(const vision_config_t *cfg, bool persist)
{
    if (!cfg) return;
    vision_config_t clean = *cfg;
    vision_config_sanitize(&clean);
    portENTER_CRITICAL(&lock);
    active = clean;
    ++revision;
    portEXIT_CRITICAL(&lock);
    if (persist) {
        const esp_err_t err = vision_settings_save();
        if (err != ESP_OK) ESP_LOGW(TAG, "save failed: %s", esp_err_to_name(err));
    }
}

void vision_settings_reset(void)
{
    vision_config_t defaults;
    vision_config_default(&defaults);
    vision_settings_submit(&defaults, false);
    if (nvs_ready) {
        nvs_handle_t h;
        if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, SETTINGS_KEY);
            nvs_commit(h);
            nvs_close(h);
        }
    }
}

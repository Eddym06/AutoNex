#ifndef WRO_VISION_SETTINGS_H
#define WRO_VISION_SETTINGS_H

#include "esp_err.h"
#include "vision.h"

/* Active vision configuration. Defaults come from vision_config_default(); the
 * values last saved to flash (NVS) override them at boot, and the console / the
 * P4 can replace them at run time (see vision_config_rx.h). */
esp_err_t vision_settings_init(void);
/* Copy of the active configuration. */
void vision_settings_current(vision_config_t *out);
/* Returns true and fills `out` when the configuration changed since the last
 * call. The vision loop calls it once per frame, so a change never lands in
 * the middle of a frame. */
bool vision_settings_poll(vision_config_t *out);
/* Replaces the configuration (sanitized). persist == true also writes flash. */
void vision_settings_submit(const vision_config_t *cfg, bool persist);
/* Drops the saved copy and returns to the built-in defaults. */
void vision_settings_reset(void);
/* Writes the active configuration to flash. */
esp_err_t vision_settings_save(void);

#endif

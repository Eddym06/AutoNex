#pragma once
#include "esp_camera.h"
#include "vision.h"

esp_err_t vision_camera_init(void);

/* Writes the sensor registers described by `settings`. With settings->apply ==
 * false nothing is touched and the sensor keeps its driver defaults. */
esp_err_t vision_camera_apply_settings(const vision_camera_settings_t *settings);

/* Freezes exposure, gain and white balance at their current automatic values.
 * Call it after the auto controls have converged so a bright wall or a flicker
 * entering the frame cannot shift the color thresholds during a run. */
esp_err_t vision_camera_lock_auto(void);

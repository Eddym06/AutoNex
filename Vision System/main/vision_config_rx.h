#ifndef WRO_VISION_CONFIG_RX_H
#define WRO_VISION_CONFIG_RX_H

#include "esp_err.h"

/* Starts the task that listens for live configuration on the console UART
 * (a serial monitor or a host tool) and, when CONFIG_WRO_LINK_RX_GPIO >= 0, on
 * the UART that links to the P4. See vision_config_cmd.h for the commands. */
esp_err_t vision_config_rx_start(void);

#endif

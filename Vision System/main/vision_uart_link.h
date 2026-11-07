#ifndef WRO_VISION_UART_LINK_H
#define WRO_VISION_UART_LINK_H

#include "esp_err.h"
#include "vision.h"

esp_err_t vision_uart_link_init(void);
esp_err_t vision_uart_link_send(const vision_result_t *result, int width,
                                uint16_t sequence, uint32_t timestamp_ms);

#endif

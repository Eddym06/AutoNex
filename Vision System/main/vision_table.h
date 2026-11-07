#ifndef WRO_VISION_TABLE_H
#define WRO_VISION_TABLE_H

#include "vision.h"
#include "sensor_table_protocol.h"

void vision_table_build(const vision_result_t *result, int image_width,
                        uint16_t sequence, uint32_t timestamp_ms,
                        wro_sensor_table_t *table);

#endif

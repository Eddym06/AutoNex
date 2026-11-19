#include "sensor_table_protocol.h"

uint16_t wro_sensor_table_crc16(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xFFFFu;
    if (!bytes) return 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t wro_sensor_table_encode(const wro_sensor_table_t *table,
                               uint8_t *output, size_t capacity)
{
    if (!table || !output || table->object_count > WRO_TABLE_MAX_OBJECTS) return 0;
    const size_t length = WRO_TABLE_HEADER_SIZE +
                          (size_t)table->object_count * WRO_TABLE_OBJECT_SIZE + 2u;
    if (capacity < length) return 0;
    output[0] = WRO_TABLE_MAGIC_0;
    output[1] = WRO_TABLE_MAGIC_1;
    output[2] = WRO_TABLE_VERSION;
    output[3] = (uint8_t)table->sequence;
    output[4] = (uint8_t)(table->sequence >> 8);
    output[5] = (uint8_t)table->frame_timestamp_ms;
    output[6] = (uint8_t)(table->frame_timestamp_ms >> 8);
    output[7] = (uint8_t)(table->frame_timestamp_ms >> 16);
    output[8] = (uint8_t)(table->frame_timestamp_ms >> 24);
    output[9] = table->valid ? 1u : 0u;
    output[10] = table->action;
    output[11] = table->object_count;
    output[12] = table->line_orange_row;
    output[13] = table->line_blue_row;
    size_t offset = WRO_TABLE_HEADER_SIZE;
    for (size_t i = 0; i < table->object_count; ++i) {
        const wro_table_object_t *object = &table->objects[i];
        output[offset++] = object->color;
        output[offset++] = object->center_x_px;
        output[offset++] = object->width_px;
        output[offset++] = (uint8_t)object->distance_mm;
        output[offset++] = (uint8_t)(object->distance_mm >> 8);
        output[offset++] = object->flags;
    }
    const uint16_t crc = wro_sensor_table_crc16(output + 2, length - 4);
    output[offset++] = (uint8_t)crc;
    output[offset++] = (uint8_t)(crc >> 8);
    return offset;
}

#ifndef WRO_SENSOR_TABLE_PROTOCOL_H
#define WRO_SENSOR_TABLE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* S3 -> P4 detection table, version 2.
 *
 * Frame: A5 5A | version | seq(2) | frame_ms(4) | valid | action | count |
 *        orange_row | blue_row | count * object(6) | crc16(2)
 * CRC16-CCITT covers everything between the magic bytes and the CRC. */
#define WRO_TABLE_MAGIC_0 0xA5u
#define WRO_TABLE_MAGIC_1 0x5Au
#define WRO_TABLE_VERSION 2u
#define WRO_TABLE_MAX_OBJECTS 12u
#define WRO_TABLE_HEADER_SIZE 14u
#define WRO_TABLE_OBJECT_SIZE 6u
#define WRO_TABLE_MAX_SIZE (WRO_TABLE_HEADER_SIZE + WRO_TABLE_MAX_OBJECTS * WRO_TABLE_OBJECT_SIZE + 2u)

/* Per-object quality flags. They let the P4 weigh a detection instead of
 * trusting a distance that the camera could not measure well. */
#define WRO_OBJ_CLIPPED   0x01u /* touches the image border: width is a lower bound */
#define WRO_OBJ_GROUND    0x02u /* distance comes from the ground-contact row */
#define WRO_OBJ_OCCLUDED  0x04u /* another colored object hides the base */
#define WRO_OBJ_MERGED    0x08u /* box is much wider than one pillar: touching pillars */

typedef struct {
    uint8_t color;         /* 1 red, 2 green, 3 parking (magenta) */
    uint8_t center_x_px;   /* QQVGA image coordinate, 0..159 */
    uint8_t width_px;
    uint16_t distance_mm;  /* optical depth (along the camera axis), not range */
    uint8_t flags;         /* WRO_OBJ_* */
} wro_table_object_t;

typedef struct {
    uint16_t sequence;
    uint32_t frame_timestamp_ms; /* S3 clock: diagnostics only */
    bool valid;
    uint8_t action;
    uint8_t object_count;
    /* Lowest image row (QQVGA coordinates, 1..255) where an orange/blue floor
     * line is seen; 0 means no line. A line gets closer as the row grows. */
    uint8_t line_orange_row;
    uint8_t line_blue_row;
    wro_table_object_t objects[WRO_TABLE_MAX_OBJECTS];
} wro_sensor_table_t;

/* Returns zero for invalid input/capacity; otherwise writes one binary frame. */
size_t wro_sensor_table_encode(const wro_sensor_table_t *table,
                               uint8_t *output, size_t capacity);
uint16_t wro_sensor_table_crc16(const uint8_t *bytes, size_t length);

#endif

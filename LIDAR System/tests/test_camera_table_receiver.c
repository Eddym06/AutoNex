#include "camera_table_receiver.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    wro_sensor_table_t source = {
        .sequence = 65530,
        .frame_timestamp_ms = 1234567,
        .valid = true,
        .action = 4,
        .object_count = 2,
        .line_orange_row = 88,
        .objects = {
            {.color = 1, .center_x_px = 80, .width_px = 20, .distance_mm = 300},
            {.color = 2, .center_x_px = 120, .width_px = 12, .distance_mm = 450,
             .flags = WRO_OBJ_GROUND | WRO_OBJ_CLIPPED}
        }
    };
    uint8_t bytes[WRO_TABLE_MAX_SIZE];
    const size_t length = wro_sensor_table_encode(&source, bytes, sizeof(bytes));
    assert(length == WRO_TABLE_HEADER_SIZE + 2 * WRO_TABLE_OBJECT_SIZE + 2);

    camera_table_decoder_t decoder = {0};
    camera_table_t table;
    assert(!camera_table_decoder_feed(&decoder, bytes, 7, 1000000, &table));
    assert(camera_table_decoder_feed(&decoder, bytes + 7, length - 7, 1000000, &table));
    assert(table.valid && table.sequence == source.sequence);
    assert(table.frame_timestamp_ms == source.frame_timestamp_ms);
    assert(table.received_timestamp_us == 1000000);
    assert(table.object_count == 2);
    assert(table.objects[0].color == 1 && fabsf(table.objects[0].bearing_deg) < 0.3f);
    assert(fabsf(table.objects[0].distance_cm - 30.0f) < 0.01f);
    assert(table.objects[1].color == 2 && table.objects[1].bearing_deg > 0.0f);
    /* Pinhole: tan(bearing) = 40 px / 118.6 px, and the table carries optical
     * depth, so the range along the ray is depth / cos(bearing). */
    assert(fabsf(table.objects[1].bearing_deg - 18.64f) < 0.05f);
    assert(fabsf(table.objects[1].distance_cm - 45.0f / cosf(table.objects[1].bearing_deg * 0.0174533f)) < 0.05f);
    assert(table.objects[1].flags == (WRO_OBJ_GROUND | WRO_OBJ_CLIPPED));
    assert(table.line_orange_row == 88 && table.line_blue_row == 0);
    /* A different lens changes the angle without touching the wire format. */
    camera_table_decoder_set_focal(&decoder, 160.0f);
    camera_table_decoder_reset(&decoder);
    assert(camera_table_decoder_feed(&decoder, bytes, length, 1000000, &table));
    assert(fabsf(table.objects[1].bearing_deg - 14.04f) < 0.05f);
    camera_table_decoder_set_focal(&decoder, 0.0f);

    bytes[length - 1] ^= 0x80;
    camera_table_decoder_reset(&decoder);
    assert(!camera_table_decoder_feed(&decoder, bytes, length, 1000000, &table));
    assert(camera_table_decoder_feed(&decoder, bytes, 0, 1000000, &table) == false);
    bytes[length - 1] ^= 0x80;
    assert(camera_table_decoder_feed(&decoder, bytes, length, 1100000, &table));
    assert(table.received_timestamp_us == 1100000);

    /* A damaged object count makes the receiver consume part of the next
     * frame; its embedded magic must still be recovered. */
    wro_sensor_table_t short_source = {.sequence = 7, .valid = true, .action = 1};
    uint8_t short_frame[WRO_TABLE_MAX_SIZE];
    const size_t short_length = wro_sensor_table_encode(&short_source,
                                                         short_frame, sizeof(short_frame));
    assert(short_length == WRO_TABLE_HEADER_SIZE + 2u);
    uint8_t damaged[WRO_TABLE_MAX_SIZE];
    memcpy(damaged, short_frame, short_length);
    damaged[11] = 2;
    camera_table_decoder_reset(&decoder);
    assert(!camera_table_decoder_feed(&decoder, damaged, short_length, 1200000, &table));
    assert(camera_table_decoder_feed(&decoder, short_frame, short_length, 1300000, &table));
    assert(table.sequence == 7 && table.received_timestamp_us == 1300000);
    puts("PASS: S3 detection-table framing, CRC, chunked UART input and P4 timestamps");
    return 0;
}

#include "vision_table.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    vision_result_t result = {.valid = true, .action = V_LEFT_SOFT, .object_count = 1,
                              .line_orange_row = 91, .line_blue_row = 0};
    result.objects[0] = (vision_object_t){
        .color = 2,
        .box = {.min_x = 140, .max_x = 179, .distance_cm = 42.5f, .detected = true,
                .flags = VISION_FLAG_GROUND | VISION_FLAG_CLIPPED}
    };
    wro_sensor_table_t table;
    vision_table_build(&result, 320, 99, 1234, &table);
    assert(table.valid && table.sequence == 99 && table.action == V_LEFT_SOFT);
    assert(table.frame_timestamp_ms == 1234 && table.object_count == 1);
    assert(table.objects[0].color == 2);
    assert(table.objects[0].center_x_px == 80);
    assert(table.objects[0].width_px == 20);
    assert(table.objects[0].distance_mm == 425);
    assert(table.objects[0].flags == (WRO_OBJ_GROUND | WRO_OBJ_CLIPPED));
    assert(table.line_orange_row == 91 && table.line_blue_row == 0);
    result.parking = (vision_box_t){.min_x = 150, .max_x = 169,
        .distance_cm = 25.0f, .detected = true};
    result.action = V_PARK_APPROACH;
    vision_table_build(&result, 320, 100, 1250, &table);
    assert(table.action == V_PARK_APPROACH && table.object_count == 2);
    assert(table.objects[1].color == 3 && table.objects[1].distance_mm == 250);
    result.parking.distance_cm = 0.0f;
    vision_table_build(&result, 320, 101, 1260, &table);
    assert(table.object_count == 1);

    /* The encoded frame carries the new fields. */
    uint8_t bytes[WRO_TABLE_MAX_SIZE];
    const size_t length = wro_sensor_table_encode(&table, bytes, sizeof(bytes));
    assert(length == WRO_TABLE_HEADER_SIZE + WRO_TABLE_OBJECT_SIZE + 2u);
    assert(bytes[2] == 2u && bytes[12] == 91u && bytes[13] == 0u);
    assert(bytes[WRO_TABLE_HEADER_SIZE + 5] == (WRO_OBJ_GROUND | WRO_OBJ_CLIPPED));
    puts("PASS: camera detections scale to the shared P4 sensor-table format (v2)");
    return 0;
}

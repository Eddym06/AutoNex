#include "vision_table.h"

#include <math.h>
#include <string.h>

static void add_box(wro_sensor_table_t *table, const vision_box_t *box,
                    uint8_t color, float scale)
{
    if (!box->detected || table->object_count >= WRO_TABLE_MAX_OBJECTS ||
        !isfinite(box->distance_cm) || box->distance_cm <= 0.0f) return;
    wro_table_object_t *target = &table->objects[table->object_count++];
    const float center = (float)(box->min_x + box->max_x + 1) * 0.5f;
    const int width = box->max_x - box->min_x + 1;
    target->color = color;
    target->center_x_px = (uint8_t)fminf(159.0f, fmaxf(0.0f, roundf(center * scale)));
    target->width_px = (uint8_t)fminf(255.0f, fmaxf(1.0f, roundf((float)width * scale)));
    target->distance_mm = (uint16_t)fminf(65535.0f,
        fmaxf(1.0f, roundf(box->distance_cm * 10.0f)));
    target->flags = 0;
    if (box->flags & VISION_FLAG_CLIPPED) target->flags |= WRO_OBJ_CLIPPED;
    if (box->flags & VISION_FLAG_GROUND) target->flags |= WRO_OBJ_GROUND;
    if (box->flags & VISION_FLAG_OCCLUDED) target->flags |= WRO_OBJ_OCCLUDED;
    if (box->flags & VISION_FLAG_MERGED) target->flags |= WRO_OBJ_MERGED;
}

void vision_table_build(const vision_result_t *result, int image_width,
                        uint16_t sequence, uint32_t timestamp_ms,
                        wro_sensor_table_t *table)
{
    if (!table) return;
    memset(table, 0, sizeof(*table));
    if (!result || image_width <= 0) return;
    table->sequence = sequence;
    table->frame_timestamp_ms = timestamp_ms;
    table->valid = result->valid;
    table->action = (uint8_t)result->action;
    table->line_orange_row = result->line_orange_row;
    table->line_blue_row = result->line_blue_row;
    const size_t count = result->object_count < WRO_TABLE_MAX_OBJECTS
        ? result->object_count : WRO_TABLE_MAX_OBJECTS;
    const float scale = 160.0f / (float)image_width;
    for (size_t i = 0; i < count; ++i) {
        const vision_object_t *source = &result->objects[i];
        add_box(table, &source->box, source->color, scale);
    }
    add_box(table, &result->parking, 3u, scale);
}

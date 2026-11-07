#include "camera_table_receiver.h"

#include <math.h>
#include <string.h>

#define DEFAULT_FOCAL_PX 118.6f
#define QQVGA_WIDTH 160.0f
#define RAD_TO_DEG 57.29577951308232f

void camera_table_decoder_reset(camera_table_decoder_t *decoder)
{
    if (!decoder) return;
    const float focal = decoder->focal_px, pitch_cos = decoder->pitch_cos;
    memset(decoder, 0, sizeof(*decoder));
    decoder->focal_px = focal;
    decoder->pitch_cos = pitch_cos;
}

void camera_table_decoder_set_pitch_deg(camera_table_decoder_t *decoder, float pitch_deg)
{
    if (!decoder) return;
    decoder->pitch_cos = (isfinite(pitch_deg) && fabsf(pitch_deg) < 80.0f)
        ? cosf(pitch_deg / RAD_TO_DEG) : 0.0f;
}

void camera_table_decoder_set_focal(camera_table_decoder_t *decoder, float focal_px)
{
    if (decoder) decoder->focal_px = (isfinite(focal_px) && focal_px > 20.0f) ? focal_px : 0.0f;
}

static bool decode(camera_table_decoder_t *decoder, uint64_t arrival_us,
                   camera_table_t *output)
{
    const uint8_t *b = decoder->bytes;
    if (b[2] != WRO_TABLE_VERSION) return false;
    const uint8_t count = b[11];
    if (count > WRO_TABLE_MAX_OBJECTS) return false;
    const size_t frame_length = WRO_TABLE_HEADER_SIZE +
                                (size_t)count * WRO_TABLE_OBJECT_SIZE + 2u;
    const uint16_t expected_crc = (uint16_t)b[frame_length - 2] |
                                  (uint16_t)b[frame_length - 1] << 8;
    if (expected_crc != wro_sensor_table_crc16(b + 2, frame_length - 4)) return false;
    memset(output, 0, sizeof(*output));
    output->sequence = (uint16_t)b[3] | (uint16_t)b[4] << 8;
    output->frame_timestamp_ms = (uint32_t)b[5] | (uint32_t)b[6] << 8 |
                                 (uint32_t)b[7] << 16 | (uint32_t)b[8] << 24;
    output->received_timestamp_us = arrival_us;
    output->valid = (b[9] & 1u) != 0;
    output->action = b[10];
    output->line_orange_row = b[12];
    output->line_blue_row = b[13];
    if (!output->valid) return true;
    const float focal = decoder->focal_px > 0.0f ? decoder->focal_px : DEFAULT_FOCAL_PX;
    const float pitch_cos = decoder->pitch_cos > 0.0f ? decoder->pitch_cos : 1.0f;
    size_t offset = WRO_TABLE_HEADER_SIZE;
    for (size_t i = 0; i < count; ++i) {
        const uint8_t color = b[offset++];
        const uint8_t center_x = b[offset++];
        const uint8_t width = b[offset++];
        const uint8_t distance_lo = b[offset++];
        const uint8_t distance_hi = b[offset++];
        const uint8_t flags = b[offset++];
        const uint16_t distance_mm = (uint16_t)distance_lo | (uint16_t)distance_hi << 8;
        if (color == 0 || center_x >= (uint8_t)QQVGA_WIDTH || width == 0 || distance_mm == 0 ||
            output->object_count >= LOCAL_MAP_MAX_CAMERA_OBJECTS)
            continue;
        local_camera_object_t *object = &output->objects[output->object_count++];
        object->color = color;
        /* Pinhole: la tangente del angulo es el desplazamiento entre la focal.
         * Con un FOV lineal el error crece hacia los bordes de la imagen. */
        object->bearing_deg = atanf(((float)center_x - QQVGA_WIDTH * 0.5f) / focal * pitch_cos) * RAD_TO_DEG;
        /* El S3 mide la profundidad sobre el eje optico; el mapa quiere el
         * alcance a lo largo del rayo. */
        const float depth_cm = (float)distance_mm / 10.0f;
        object->distance_cm = depth_cm / cosf(object->bearing_deg / RAD_TO_DEG);
        object->valid = true;
        object->flags = flags;
    }
    return true;
}

bool camera_table_decoder_feed(camera_table_decoder_t *decoder,
                               const uint8_t *bytes, size_t length,
                               uint64_t arrival_timestamp_us,
                               camera_table_t *output)
{
    if (!decoder || (!bytes && length) || !output) return false;
    bool produced = false;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t byte = bytes[i];
        if (decoder->used == 0) {
            if (byte == WRO_TABLE_MAGIC_0) decoder->bytes[decoder->used++] = byte;
            continue;
        }
        decoder->bytes[decoder->used++] = byte;
        for (;;) {
            if (decoder->used == 1) break;
            if (decoder->bytes[1] != WRO_TABLE_MAGIC_1) goto discard_prefix;
            if (decoder->used < WRO_TABLE_HEADER_SIZE) break;
            if (decoder->bytes[2] != WRO_TABLE_VERSION ||
                decoder->bytes[11] > WRO_TABLE_MAX_OBJECTS) goto discard_prefix;
            decoder->expected = WRO_TABLE_HEADER_SIZE +
                (size_t)decoder->bytes[11] * WRO_TABLE_OBJECT_SIZE + 2u;
            if (decoder->used < decoder->expected) break;
            if (decode(decoder, arrival_timestamp_us, output)) {
                produced = true;
                decoder->used = 0;
                decoder->expected = 0;
                break;
            }
discard_prefix:
            ;
            /* A corrupt length or CRC may include the start of another frame.
             * Keep its suffix and inspect it before consuming another byte. */
            size_t start = 1;
            while (start < decoder->used && decoder->bytes[start] != WRO_TABLE_MAGIC_0)
                ++start;
            decoder->used -= start;
            if (decoder->used)
                memmove(decoder->bytes, decoder->bytes + start, decoder->used);
            decoder->expected = 0;
            if (!decoder->used) break;
        }
    }
    return produced;
}

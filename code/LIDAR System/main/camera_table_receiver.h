#ifndef CAMERA_TABLE_RECEIVER_H
#define CAMERA_TABLE_RECEIVER_H

#include "local_obstacle_map.h"
#include "../../shared/sensor_table_protocol.h"

typedef struct {
    uint8_t bytes[WRO_TABLE_MAX_SIZE];
    size_t used;
    size_t expected;
    /* Distancia focal de la camara en pixeles de una imagen de 160 de ancho: la
     * misma que usa el S3 (vision_config_t.focal_px). 0 = valor por defecto. */
    float focal_px;
    /* Cabeceo de la camara hacia abajo (coseno; 0 = sin inclinacion). */
    float pitch_cos;
} camera_table_decoder_t;

typedef struct {
    uint16_t sequence;
    uint32_t frame_timestamp_ms;
    uint64_t received_timestamp_us;
    uint8_t action;
    bool valid;
    size_t object_count;
    uint8_t line_orange_row, line_blue_row; /* 0 = sin linea; mas grande = mas cerca */
    local_camera_object_t objects[LOCAL_MAP_MAX_CAMERA_OBJECTS];
} camera_table_t;

void camera_table_decoder_reset(camera_table_decoder_t *decoder);
void camera_table_decoder_set_focal(camera_table_decoder_t *decoder, float focal_px);
/* Una camara inclinada ve el angulo horizontal de un pixel acortado por cos(cabeceo). */
void camera_table_decoder_set_pitch_deg(camera_table_decoder_t *decoder, float pitch_deg);
/* Feed UART bytes. Returns true once a CRC-checked camera table was decoded.
 * arrival_timestamp_us is from the P4 monotonic clock and is used for fusion. */
bool camera_table_decoder_feed(camera_table_decoder_t *decoder,
                               const uint8_t *bytes, size_t length,
                               uint64_t arrival_timestamp_us,
                               camera_table_t *output);

#endif

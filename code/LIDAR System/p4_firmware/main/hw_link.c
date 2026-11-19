#include "hw_link.h"

#include "esp_timer.h"
#include "hw_camera.h"

static link_decoder_t s_dec;
static volatile uint8_t s_cmd;
static volatile int64_t s_alive_us;
static uint8_t s_last_cal_seq;
static bool s_cal_seq_valid;

size_t hw_link_read(void *ctx, uint8_t *buffer, size_t capacity)
{
    const size_t n = hw_camera_read(ctx, buffer, capacity);
    for (size_t i = 0; i < n; ++i) {
        if (link_decoder_feed(&s_dec, buffer[i]) != LINK_TYPE_CMD) continue;
        link_cmd_t c;
        if (!link_decode_cmd(s_dec.payload, s_dec.payload_len, &c)) continue;
        if (c.cmd == LINK_CMD_CAL_FRONT || c.cmd == LINK_CMD_CAL_RIGHT || c.cmd == LINK_CMD_SET_NORTH || c.cmd == LINK_CMD_CAL_PITCH ||
            c.cmd == LINK_CMD_MOTOR_M1 || c.cmd == LINK_CMD_MOTOR_M2 || c.cmd == LINK_CMD_MOTOR_SEQ) {   /* una vez por clic, aunque la orden se repita */
            if (!s_cal_seq_valid || c.seq != s_last_cal_seq) {   /* una calibracion por clic, aunque la orden se repita */
                s_cmd = c.cmd;
                s_last_cal_seq = c.seq;
                s_cal_seq_valid = true;
            }
        } else if (c.cmd != LINK_CMD_NONE) s_cmd = c.cmd;
        if (c.web_alive) s_alive_us = esp_timer_get_time();
    }
    return n;   /* los bytes siguen su camino al decodificador de la tabla: tienen otra magia y se ignoran */
}

uint8_t hw_link_take_cmd(void)
{
    const uint8_t c = s_cmd;
    s_cmd = LINK_CMD_NONE;
    return c;
}

int64_t hw_link_last_alive_us(void) { return s_alive_us; }

bool hw_link_send_state(const link_state_t *state)
{
    static uint8_t frame[LINK_MAX_FRAME];
    const size_t n = link_encode_state(state, frame, sizeof frame);
    return n && hw_camera_write(frame, n);
}

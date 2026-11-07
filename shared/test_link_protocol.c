#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "link_protocol.h"

static int feed_all(link_decoder_t *d, const uint8_t *b, size_t n)
{
    int t = 0;
    for (size_t i = 0; i < n; ++i) { int r = link_decoder_feed(d, b[i]); if (r) t = r; }
    return t;
}

int main(void)
{
    static link_state_t s, o;
    static uint8_t buf[LINK_MAX_FRAME + 16];
    s.phase = 1; s.msg = LINK_MSG_RED_RIGHT; s.armed = true; s.direction = -1; s.laps = 2; s.corners = 5;
    s.speed_cm_s = 23.4f; s.yaw_rate_dps = -12.5f; s.yaw_deg = -91.2f;
    s.front_cm = 80; s.left_cm = 40; s.right_cm = 55; s.elapsed_s = 61.3f;
    s.enc_left_m = 1.234f; s.enc_right_m = -0.5f; s.imu_dps = 0.37f;
    s.lidar_age_ms = 90; s.lidar_revs = 4000000000u; s.tables_ok = 77;
    s.cal_msg = LINK_CAL_SIGN_FLIPPED; s.cal_sign = -1; s.cal_offset_deg = -37.5f;
    s.map_heading_deg = -123.4f; s.map_e_cm = -250.0f; s.map_n_cm = 417.0f; s.map_epoch = 9; s.gyro_bias_dps = -0.87f;
    s.height_cm = -12.5f; s.pitch_deg = 7.5f; s.level = 3; s.pitch_cal = true;
    s.cam_count = 1; s.cam[0].bearing_deg = -12.3f; s.cam[0].distance_cm = 150; s.cam[0].color = 2;
    s.scan_count = LINK_MAX_SCAN_POINTS;
    for (int i = 0; i < (int)LINK_MAX_SCAN_POINTS; ++i) { s.scan[i].angle_deg = -179.0f + i * 0.7f; s.scan[i].distance_cm = (uint16_t)(10 + i); }
    size_t n = link_encode_state(&s, buf, sizeof buf);
    assert(n > 0 && n <= LINK_MAX_FRAME);

    link_decoder_t d; link_decoder_reset(&d);
    uint8_t noise[] = {0x00, 0xA6, 0xA6, 0x12, 0xA6, 0x5B, 0xFF};      /* ruido y un falso arranque */
    assert(feed_all(&d, noise, sizeof noise) == 0);
    assert(feed_all(&d, buf, n) == LINK_TYPE_STATE);
    assert(link_decode_state(d.payload, d.payload_len, &o));
    assert(o.msg == LINK_MSG_RED_RIGHT && o.armed && o.direction == -1 && o.laps == 2 && o.corners == 5);
    assert(fabsf(o.speed_cm_s - 23.4f) < 0.06f && fabsf(o.yaw_rate_dps + 12.5f) < 0.06f && fabsf(o.yaw_deg + 91.2f) < 0.06f);
    assert(o.front_cm == 80 && o.left_cm == 40 && o.right_cm == 55 && fabsf(o.elapsed_s - 61.3f) < 0.06f);
    assert(fabsf(o.enc_left_m - 1.234f) < 0.001f && fabsf(o.enc_right_m + 0.5f) < 0.001f && fabsf(o.imu_dps - 0.37f) < 0.006f);
    assert(o.lidar_age_ms == 90 && o.lidar_revs == 4000000000u && o.tables_ok == 77);
    assert(o.cal_msg == LINK_CAL_SIGN_FLIPPED && o.cal_sign == -1 && fabsf(o.cal_offset_deg + 37.5f) < 0.06f);
    assert(fabsf(o.map_heading_deg + 123.4f) < 0.06f && o.map_e_cm == -250.0f && o.map_n_cm == 417.0f && o.map_epoch == 9 && fabsf(o.gyro_bias_dps + 0.87f) < 0.006f);
    assert(fabsf(o.height_cm + 12.5f) < 0.06f && fabsf(o.pitch_deg - 7.5f) < 0.06f && o.level == 3 && o.pitch_cal);
    assert(o.cam_count == 1 && fabsf(o.cam[0].bearing_deg + 12.3f) < 0.06f && o.cam[0].distance_cm == 150 && o.cam[0].color == 2);
    assert(o.scan_count == LINK_MAX_SCAN_POINTS && fabsf(o.scan[10].angle_deg - (-179.0f + 7.0f)) < 0.06f && o.scan[10].distance_cm == 20);

    buf[40] ^= 0x01;                                                    /* un bit dañado: el CRC debe rechazarlo */
    assert(feed_all(&d, buf, n) == 0);

    link_cmd_t c = {LINK_CMD_CAL_FRONT, 7, true}, oc;
    n = link_encode_cmd(&c, buf, sizeof buf);
    assert(feed_all(&d, buf, n) == LINK_TYPE_CMD && link_decode_cmd(d.payload, d.payload_len, &oc));
    assert(oc.cmd == LINK_CMD_CAL_FRONT && oc.seq == 7 && oc.web_alive);
    puts("link_protocol OK");
    return 0;
}

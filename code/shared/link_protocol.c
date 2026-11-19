#include "link_protocol.h"

#include <math.h>
#include <string.h>

#include "sensor_table_protocol.h"   /* wro_sensor_table_crc16 */

typedef struct { uint8_t *p; size_t cap, n; bool ok; } wr_t;

static void put(wr_t *w, uint32_t v, int bytes)
{
    if (w->n + (size_t)bytes > w->cap) { w->ok = false; return; }
    for (int i = 0; i < bytes; ++i) w->p[w->n++] = (uint8_t)(v >> (8 * i));
}

static int32_t clampi(float v, int32_t lo, int32_t hi)
{
    const long r = lroundf(v);
    return r < lo ? lo : r > hi ? hi : (int32_t)r;
}

static size_t finish(wr_t *w, size_t payload_start, uint8_t type)
{
    if (!w->ok) return 0;
    const size_t len = w->n - payload_start;
    w->p[2] = type;
    w->p[3] = (uint8_t)len;
    w->p[4] = (uint8_t)(len >> 8);
    put(w, wro_sensor_table_crc16(w->p + 2, w->n - 2), 2);
    return w->ok ? w->n : 0;
}

static void header(wr_t *w) { put(w, LINK_MAGIC_0, 1); put(w, LINK_MAGIC_1, 1); put(w, 0, 3); }

size_t link_encode_cmd(const link_cmd_t *c, uint8_t *out, size_t capacity)
{
    wr_t w = {out, capacity, 0, true};
    header(&w);
    put(&w, c->cmd, 1);
    put(&w, c->web_alive, 1);
    put(&w, c->seq, 1);
    return finish(&w, 5, LINK_TYPE_CMD);
}

size_t link_encode_state(const link_state_t *s, uint8_t *out, size_t capacity)
{
    wr_t w = {out, capacity, 0, true};
    header(&w);
    put(&w, s->phase, 1); put(&w, s->msg, 1); put(&w, s->armed, 1); put(&w, (uint8_t)s->direction, 1);
    put(&w, s->laps, 1); put(&w, s->corners, 1);
    put(&w, (uint32_t)clampi(s->speed_cm_s * 10.0f, -32768, 32767), 2);
    put(&w, (uint32_t)clampi(s->yaw_rate_dps * 10.0f, -32768, 32767), 2);
    put(&w, (uint32_t)clampi(s->yaw_deg * 10.0f, -32768, 32767), 2);
    put(&w, s->front_cm, 2); put(&w, s->left_cm, 2); put(&w, s->right_cm, 2);
    put(&w, (uint32_t)clampi(s->elapsed_s * 10.0f, 0, 65535), 2);
    put(&w, (uint32_t)clampi(s->enc_left_m * 1000.0f, INT32_MIN / 2, INT32_MAX / 2), 4);
    put(&w, (uint32_t)clampi(s->enc_right_m * 1000.0f, INT32_MIN / 2, INT32_MAX / 2), 4);
    put(&w, (uint32_t)clampi(s->imu_dps * 100.0f, -32768, 32767), 2);
    put(&w, s->lidar_age_ms, 2);
    put(&w, s->lidar_revs, 4); put(&w, s->tables_ok, 4);
    put(&w, s->cal_msg, 1); put(&w, (uint8_t)s->cal_sign, 1);
    put(&w, (uint32_t)clampi(s->cal_offset_deg * 10.0f, -32768, 32767), 2);
    put(&w, (uint32_t)clampi(s->map_heading_deg * 10.0f, -1800, 1800), 2);
    put(&w, (uint32_t)clampi(s->map_e_cm, -32768, 32767), 2);
    put(&w, (uint32_t)clampi(s->map_n_cm, -32768, 32767), 2);
    put(&w, s->map_epoch, 1);
    put(&w, (uint32_t)clampi(s->gyro_bias_dps * 100.0f, -32768, 32767), 2);
    put(&w, (uint32_t)clampi(s->height_cm * 10.0f, -32768, 32767), 2);
    put(&w, (uint32_t)clampi(s->pitch_deg * 10.0f, -900, 900), 2);
    put(&w, s->level, 1); put(&w, s->pitch_cal, 1); put(&w, s->motor_test, 1);
    const uint8_t nc = s->cam_count < LINK_MAX_CAM_OBJECTS ? s->cam_count : LINK_MAX_CAM_OBJECTS;
    put(&w, nc, 1);
    for (uint8_t i = 0; i < nc; ++i) {
        put(&w, (uint32_t)clampi(s->cam[i].bearing_deg * 10.0f, -32768, 32767), 2);
        put(&w, s->cam[i].distance_cm, 2); put(&w, s->cam[i].color, 1);
    }
    const uint16_t ns = s->scan_count < LINK_MAX_SCAN_POINTS ? s->scan_count : LINK_MAX_SCAN_POINTS;
    put(&w, ns, 2);
    for (uint16_t i = 0; i < ns; ++i) {
        float a = fmodf(s->scan[i].angle_deg, 360.0f);
        if (a < 0.0f) a += 360.0f;
        put(&w, (uint32_t)clampi(a * 10.0f, 0, 3599), 2);
        put(&w, s->scan[i].distance_cm, 2);
    }
    return finish(&w, 5, LINK_TYPE_STATE);
}

typedef struct { const uint8_t *p; size_t len, n; bool ok; } rd_t;

static uint32_t get(rd_t *r, int bytes)
{
    if (r->n + (size_t)bytes > r->len) { r->ok = false; return 0; }
    uint32_t v = 0;
    for (int i = 0; i < bytes; ++i) v |= (uint32_t)r->p[r->n++] << (8 * i);
    return v;
}

bool link_decode_cmd(const uint8_t *payload, size_t len, link_cmd_t *out)
{
    rd_t r = {payload, len, 0, true};
    out->cmd = (uint8_t)get(&r, 1);
    out->web_alive = get(&r, 1) != 0;
    out->seq = (uint8_t)get(&r, 1);
    return r.ok;
}

bool link_decode_state(const uint8_t *payload, size_t len, link_state_t *s)
{
    rd_t r = {payload, len, 0, true};
    s->phase = (uint8_t)get(&r, 1); s->msg = (uint8_t)get(&r, 1); s->armed = get(&r, 1) != 0;
    s->direction = (int8_t)get(&r, 1);
    s->laps = (uint8_t)get(&r, 1); s->corners = (uint8_t)get(&r, 1);
    s->speed_cm_s = (int16_t)get(&r, 2) / 10.0f;
    s->yaw_rate_dps = (int16_t)get(&r, 2) / 10.0f;
    s->yaw_deg = (int16_t)get(&r, 2) / 10.0f;
    s->front_cm = (uint16_t)get(&r, 2); s->left_cm = (uint16_t)get(&r, 2); s->right_cm = (uint16_t)get(&r, 2);
    s->elapsed_s = (uint16_t)get(&r, 2) / 10.0f;
    s->enc_left_m = (int32_t)get(&r, 4) / 1000.0f;
    s->enc_right_m = (int32_t)get(&r, 4) / 1000.0f;
    s->imu_dps = (int16_t)get(&r, 2) / 100.0f;
    s->lidar_age_ms = (uint16_t)get(&r, 2);
    s->lidar_revs = get(&r, 4); s->tables_ok = get(&r, 4);
    s->cal_msg = (uint8_t)get(&r, 1); s->cal_sign = (int8_t)get(&r, 1);
    s->cal_offset_deg = (int16_t)get(&r, 2) / 10.0f;
    s->map_heading_deg = (int16_t)get(&r, 2) / 10.0f;
    s->map_e_cm = (int16_t)get(&r, 2);
    s->map_n_cm = (int16_t)get(&r, 2);
    s->map_epoch = (uint8_t)get(&r, 1);
    s->gyro_bias_dps = (int16_t)get(&r, 2) / 100.0f;
    s->height_cm = (int16_t)get(&r, 2) / 10.0f;
    s->pitch_deg = (int16_t)get(&r, 2) / 10.0f;
    s->level = (uint8_t)get(&r, 1); s->pitch_cal = get(&r, 1) != 0;
    s->motor_test = (uint8_t)get(&r, 1);
    s->cam_count = (uint8_t)get(&r, 1);
    if (s->cam_count > LINK_MAX_CAM_OBJECTS) return false;
    for (uint8_t i = 0; i < s->cam_count; ++i) {
        s->cam[i].bearing_deg = (int16_t)get(&r, 2) / 10.0f;
        s->cam[i].distance_cm = (uint16_t)get(&r, 2); s->cam[i].color = (uint8_t)get(&r, 1);
    }
    s->scan_count = (uint16_t)get(&r, 2);
    if (s->scan_count > LINK_MAX_SCAN_POINTS) return false;
    for (uint16_t i = 0; i < s->scan_count; ++i) {
        float a = (uint16_t)get(&r, 2) / 10.0f;
        s->scan[i].angle_deg = a > 180.0f ? a - 360.0f : a;     /* 0 al frente, positivo a la derecha */
        s->scan[i].distance_cm = (uint16_t)get(&r, 2);
    }
    return r.ok && r.n == r.len;
}

void link_decoder_reset(link_decoder_t *d) { d->used = 0; }

/* Descarta el primer byte y deja el buffer empezando en la siguiente posible cabecera (ya recibida). */
static void resync(link_decoder_t *d)
{
    size_t i = 1;
    while (i < d->used && d->buf[i] != LINK_MAGIC_0) ++i;
    if (i >= d->used) { d->used = 0; return; }
    memmove(d->buf, d->buf + i, d->used - i);
    d->used -= i;
}

int link_decoder_feed(link_decoder_t *d, uint8_t b)
{
    if (d->used == 0 && b != LINK_MAGIC_0) return 0;
    d->buf[d->used++] = b;
    for (;;) {
        if (d->used >= 2 && d->buf[1] != LINK_MAGIC_1) { resync(d); continue; }
        if (d->used < 5) return 0;
        const size_t len = (size_t)d->buf[3] | ((size_t)d->buf[4] << 8);
        if (len > LINK_MAX_PAYLOAD) { resync(d); continue; }
        if (d->used < 5 + len + 2) return 0;
        const uint16_t crc = (uint16_t)(d->buf[5 + len] | (d->buf[6 + len] << 8));
        if (crc != wro_sensor_table_crc16(d->buf + 2, 3 + len)) { resync(d); continue; }
        d->used = 0;
        d->payload = d->buf + 5;
        d->payload_len = len;
        return d->buf[2];
    }
}

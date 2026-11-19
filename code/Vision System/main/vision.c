#include "vision.h"
#include "config_fields.h"
#include "sensor_table_protocol.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------- configuration */

void vision_config_default(vision_config_t *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    /* Official WRO colors: red RGB(238,39,55) = H356 S84 V93, green
     * RGB(68,214,44) = H111 S79 V84, magenta RGB(255,0,255) = H300, orange
     * CMYK(0,60,100,0) = H24, blue CMYK(100,80,0,0) = H228. The windows are wide
     * enough to survive a white-balance shift but keep orange away from red. */
    c->hsv[VISION_CLASS_RED]     = (vision_hsv_window_t){335, 14, 50, 40, true};
    c->hsv[VISION_CLASS_GREEN]   = (vision_hsv_window_t){80, 160, 40, 35, true};
    c->hsv[VISION_CLASS_MAGENTA] = (vision_hsv_window_t){280, 330, 40, 35, true};
    c->hsv[VISION_CLASS_ORANGE]  = (vision_hsv_window_t){16, 45, 60, 50, true};
    c->hsv[VISION_CLASS_BLUE]    = (vision_hsv_window_t){200, 260, 50, 30, true};
    c->pass_side_red = 1;
    c->pass_side_green = -1;
    c->enforce_color_side = true;
    c->focal_px = 118.6f;
    c->camera_height_cm = 5.0f;
    c->horizon_row = 30.0f;
    c->pillar_width_cm = 5.0f;
    c->pillar_height_cm = 10.0f;
    c->use_ground_distance = false; /* needs camera_height_cm / horizon_row measured */
    c->merged_ratio = 0.62f;
    c->roi_start = 20;
    c->roi_end = 110;
    c->min_samples = 30;
    c->min_span_px = 4;
    c->line_row_start = 60;
    c->line_row_end = 118;
    c->line_min_samples = 24;
    c->max_objects = VISION_MAX_OBJECTS;
    c->act_distance_cm = 45.0f;
    c->corridor_left_px = 45;
    c->corridor_right_px = 115;
    c->open_left_px = 56;
    c->open_right_px = 104;
    c->park_left_px = 70;
    c->park_right_px = 90;
    c->park_ready_row = 100;
    c->hard_distance_cm = 15.0f;
    c->turn_radius_cm = 20.785f; /* 12 cm wheelbase at 30 degrees */
    c->turn_clearance_cm = 9.0f;
    c->turn_lead_cm = 13.5f;
    c->camera = (vision_camera_settings_t){
        .apply = false, .auto_exposure = true, .exposure = 300, .ae_level = 0,
        .auto_gain = true, .gain = 4, .auto_wb = true, .wb_mode = 0,
        .brightness = 0, .contrast = 0, .saturation = 0,
        .hmirror = false, .vflip = false, .lock_after_frames = 0};
}

static bool clamp_i(int *v, int lo, int hi)
{
    if (*v < lo) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}
static bool clamp_f(float *v, float lo, float hi)
{
    if (!isfinite(*v)) { *v = lo; return true; }
    if (*v < lo) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}

bool vision_config_sanitize(vision_config_t *c)
{
    if (!c) return false;
    bool changed = false;
    for (int i = 1; i < VISION_CLASS_COUNT; ++i) {
        vision_hsv_window_t *w = &c->hsv[i];
        int lo = w->hue_lo, hi = w->hue_hi, s = w->sat_min, v = w->val_min;
        changed |= clamp_i(&lo, 0, 359); changed |= clamp_i(&hi, 0, 359);
        changed |= clamp_i(&s, 0, 100); changed |= clamp_i(&v, 0, 100);
        w->hue_lo = (int16_t)lo; w->hue_hi = (int16_t)hi;
        w->sat_min = (uint8_t)s; w->val_min = (uint8_t)v;
    }
    if (c->pass_side_red > 1 || c->pass_side_red < -1) { c->pass_side_red = 0; changed = true; }
    if (c->pass_side_green > 1 || c->pass_side_green < -1) { c->pass_side_green = 0; changed = true; }
    changed |= clamp_f(&c->focal_px, 40.0f, 400.0f);
    changed |= clamp_f(&c->camera_height_cm, 1.0f, 40.0f);
    changed |= clamp_f(&c->horizon_row, -60.0f, 110.0f);
    changed |= clamp_f(&c->pillar_width_cm, 1.0f, 30.0f);
    changed |= clamp_f(&c->pillar_height_cm, 1.0f, 40.0f);
    changed |= clamp_f(&c->merged_ratio, 0.2f, 1.0f);
    changed |= clamp_i(&c->roi_start, 0, 118);
    changed |= clamp_i(&c->roi_end, c->roi_start + 2, 120);
    changed |= clamp_i(&c->min_samples, 4, 2000);
    changed |= clamp_i(&c->min_span_px, 2, 80);
    changed |= clamp_i(&c->line_row_start, 0, 118);
    changed |= clamp_i(&c->line_row_end, c->line_row_start + 2, 120);
    changed |= clamp_i(&c->line_min_samples, 4, 80);
    changed |= clamp_i(&c->max_objects, 1, VISION_MAX_OBJECTS);
    changed |= clamp_f(&c->act_distance_cm, 10.0f, 300.0f);
    changed |= clamp_i(&c->corridor_left_px, 0, 159);
    changed |= clamp_i(&c->corridor_right_px, c->corridor_left_px, 160);
    changed |= clamp_i(&c->open_left_px, 0, 159);
    changed |= clamp_i(&c->open_right_px, c->open_left_px, 160);
    changed |= clamp_i(&c->park_left_px, 0, 159);
    changed |= clamp_i(&c->park_right_px, c->park_left_px, 160);
    changed |= clamp_i(&c->park_ready_row, 0, 120);
    changed |= clamp_f(&c->hard_distance_cm, 3.0f, 100.0f);
    changed |= clamp_f(&c->turn_radius_cm, 5.0f, 200.0f);
    changed |= clamp_f(&c->turn_clearance_cm, 0.0f, 60.0f);
    changed |= clamp_f(&c->turn_lead_cm, 0.0f, 100.0f);
    vision_camera_settings_t *k = &c->camera;
    int e = k->exposure, g = k->gain, wb = k->wb_mode, ae = k->ae_level;
    int br = k->brightness, ct = k->contrast, sa = k->saturation;
    changed |= clamp_i(&e, 0, 1200); changed |= clamp_i(&g, 0, 30);
    changed |= clamp_i(&wb, 0, 4); changed |= clamp_i(&ae, -2, 2);
    changed |= clamp_i(&br, -2, 2); changed |= clamp_i(&ct, -2, 2);
    changed |= clamp_i(&sa, -2, 2);
    k->exposure = (int16_t)e; k->gain = (uint8_t)g; k->wb_mode = (int8_t)wb;
    k->ae_level = (int8_t)ae; k->brightness = (int8_t)br;
    k->contrast = (int8_t)ct; k->saturation = (int8_t)sa;
    return changed;
}

/* One routine describes the layout and serves both directions, so the encoder
 * and the decoder cannot drift apart. */
typedef struct { uint8_t *buf; size_t pos, cap; bool write, err; } codec_t;

static void io_u8(codec_t *c, uint8_t *v)
{
    if (c->pos + 1u > c->cap) { c->err = true; return; }
    if (c->write) c->buf[c->pos] = *v; else *v = c->buf[c->pos];
    ++c->pos;
}
static void io_bool(codec_t *c, bool *v)
{
    uint8_t t = *v ? 1u : 0u;
    io_u8(c, &t);
    if (!c->write) *v = t != 0u;
}
static void io_i8(codec_t *c, int8_t *v)
{
    uint8_t t = (uint8_t)*v;
    io_u8(c, &t);
    if (!c->write) *v = (int8_t)t;
}
static void io_u16(codec_t *c, uint16_t *v)
{
    uint8_t lo = (uint8_t)(*v & 0xFFu), hi = (uint8_t)(*v >> 8);
    io_u8(c, &lo); io_u8(c, &hi);
    if (!c->write) *v = (uint16_t)(lo | (hi << 8));
}
static void io_i16(codec_t *c, int16_t *v)
{
    uint16_t t = (uint16_t)*v;
    io_u16(c, &t);
    if (!c->write) *v = (int16_t)t;
}
static void io_int(codec_t *c, int *v)
{
    int16_t t = (int16_t)*v;
    io_i16(c, &t);
    if (!c->write) *v = t;
}
static void io_f32(codec_t *c, float *v)
{
    uint32_t bits = 0;
    if (c->write) memcpy(&bits, v, sizeof(bits));
    uint8_t b[4] = {(uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
    for (int i = 0; i < 4; ++i) io_u8(c, &b[i]);
    if (!c->write) {
        bits = (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
        memcpy(v, &bits, sizeof(bits));
    }
}

static void io_config(codec_t *c, vision_config_t *cfg)
{
    for (int i = 1; i < VISION_CLASS_COUNT; ++i) {
        vision_hsv_window_t *w = &cfg->hsv[i];
        io_i16(c, &w->hue_lo); io_i16(c, &w->hue_hi);
        io_u8(c, &w->sat_min); io_u8(c, &w->val_min); io_bool(c, &w->enabled);
    }
    io_i8(c, &cfg->pass_side_red); io_i8(c, &cfg->pass_side_green);
    io_bool(c, &cfg->enforce_color_side);
    io_f32(c, &cfg->focal_px); io_f32(c, &cfg->camera_height_cm);
    io_f32(c, &cfg->horizon_row); io_f32(c, &cfg->pillar_width_cm);
    io_f32(c, &cfg->pillar_height_cm); io_f32(c, &cfg->merged_ratio);
    io_bool(c, &cfg->use_ground_distance);
    io_int(c, &cfg->roi_start); io_int(c, &cfg->roi_end);
    io_int(c, &cfg->min_samples); io_int(c, &cfg->min_span_px);
    io_int(c, &cfg->line_row_start); io_int(c, &cfg->line_row_end);
    io_int(c, &cfg->line_min_samples); io_int(c, &cfg->max_objects);
    io_f32(c, &cfg->act_distance_cm);
    io_int(c, &cfg->corridor_left_px); io_int(c, &cfg->corridor_right_px);
    io_int(c, &cfg->open_left_px); io_int(c, &cfg->open_right_px);
    io_int(c, &cfg->park_left_px); io_int(c, &cfg->park_right_px);
    io_int(c, &cfg->park_ready_row);
    io_f32(c, &cfg->hard_distance_cm); io_f32(c, &cfg->turn_radius_cm);
    io_f32(c, &cfg->turn_clearance_cm); io_f32(c, &cfg->turn_lead_cm);
    vision_camera_settings_t *k = &cfg->camera;
    io_bool(c, &k->apply); io_bool(c, &k->auto_exposure); io_i16(c, &k->exposure);
    io_i8(c, &k->ae_level); io_bool(c, &k->auto_gain); io_u8(c, &k->gain);
    io_bool(c, &k->auto_wb); io_i8(c, &k->wb_mode); io_i8(c, &k->brightness);
    io_i8(c, &k->contrast); io_i8(c, &k->saturation); io_bool(c, &k->hmirror);
    io_bool(c, &k->vflip); io_u16(c, &k->lock_after_frames);
}

#define CONFIG_MAGIC_0 0xA6u
#define CONFIG_MAGIC_1 0x6Au
#define CONFIG_VERSION 1u

size_t vision_config_encode(const vision_config_t *cfg, uint8_t *out, size_t capacity)
{
    if (!cfg || !out || capacity < 8u) return 0;
    vision_config_t copy = *cfg;
    codec_t c = {out + 4, 0, capacity - 6u, true, false};
    io_config(&c, &copy);
    if (c.err) return 0;
    out[0] = CONFIG_MAGIC_0; out[1] = CONFIG_MAGIC_1;
    out[2] = CONFIG_VERSION; out[3] = (uint8_t)c.pos;
    const uint16_t crc = wro_sensor_table_crc16(out + 2, c.pos + 2u);
    out[4 + c.pos] = (uint8_t)crc; out[5 + c.pos] = (uint8_t)(crc >> 8);
    return c.pos + 6u;
}

bool vision_config_decode(const uint8_t *bytes, size_t length, vision_config_t *cfg)
{
    if (!bytes || !cfg || length < 8u) return false;
    if (bytes[0] != CONFIG_MAGIC_0 || bytes[1] != CONFIG_MAGIC_1 ||
        bytes[2] != CONFIG_VERSION) return false;
    const size_t payload = bytes[3];
    if (length != payload + 6u) return false;
    const uint16_t crc = (uint16_t)bytes[4 + payload] | (uint16_t)bytes[5 + payload] << 8;
    if (crc != wro_sensor_table_crc16(bytes + 2, payload + 2u)) return false;
    uint8_t scratch[VISION_CONFIG_WIRE_MAX];
    if (payload > sizeof(scratch)) return false;
    memcpy(scratch, bytes + 4, payload);
    vision_config_t parsed;
    vision_config_default(&parsed);
    codec_t c = {scratch, 0, payload, false, false};
    io_config(&c, &parsed);
    if (c.err || c.pos != payload) return false;
    vision_config_sanitize(&parsed);
    *cfg = parsed;
    return true;
}

/* ---------------------------------------------------------- text interface
 *
 * "key value" lines let a person edit a config in a text file (the host tool
 * turns it into the wire format) or type it in a console. One table describes
 * every field, so a new tunable is one line here. */

#define VF(n, t, member) CF_FIELD(n, t, vision_config_t, member)
#define HSV_FIELDS(prefix, cls) \
    VF(prefix ".hue_lo", CF_I16, hsv[cls].hue_lo), VF(prefix ".hue_hi", CF_I16, hsv[cls].hue_hi), \
    VF(prefix ".sat_min", CF_U8, hsv[cls].sat_min), VF(prefix ".val_min", CF_U8, hsv[cls].val_min), \
    VF(prefix ".enabled", CF_BOOL, hsv[cls].enabled)

static const cf_field_t FIELDS[] = {
    HSV_FIELDS("red", VISION_CLASS_RED), HSV_FIELDS("green", VISION_CLASS_GREEN),
    HSV_FIELDS("magenta", VISION_CLASS_MAGENTA), HSV_FIELDS("orange", VISION_CLASS_ORANGE),
    HSV_FIELDS("blue", VISION_CLASS_BLUE),
    VF("pass_side_red", CF_I8, pass_side_red), VF("pass_side_green", CF_I8, pass_side_green),
    VF("enforce_color_side", CF_BOOL, enforce_color_side),
    VF("focal_px", CF_F32, focal_px), VF("camera_height_cm", CF_F32, camera_height_cm),
    VF("horizon_row", CF_F32, horizon_row), VF("pillar_width_cm", CF_F32, pillar_width_cm),
    VF("pillar_height_cm", CF_F32, pillar_height_cm),
    VF("use_ground_distance", CF_BOOL, use_ground_distance),
    VF("merged_ratio", CF_F32, merged_ratio),
    VF("roi_start", CF_INT, roi_start), VF("roi_end", CF_INT, roi_end),
    VF("min_samples", CF_INT, min_samples), VF("min_span_px", CF_INT, min_span_px),
    VF("line_row_start", CF_INT, line_row_start), VF("line_row_end", CF_INT, line_row_end),
    VF("line_min_samples", CF_INT, line_min_samples), VF("max_objects", CF_INT, max_objects),
    VF("act_distance_cm", CF_F32, act_distance_cm),
    VF("corridor_left_px", CF_INT, corridor_left_px), VF("corridor_right_px", CF_INT, corridor_right_px),
    VF("open_left_px", CF_INT, open_left_px), VF("open_right_px", CF_INT, open_right_px),
    VF("park_left_px", CF_INT, park_left_px), VF("park_right_px", CF_INT, park_right_px),
    VF("park_ready_row", CF_INT, park_ready_row),
    VF("hard_distance_cm", CF_F32, hard_distance_cm), VF("turn_radius_cm", CF_F32, turn_radius_cm),
    VF("turn_clearance_cm", CF_F32, turn_clearance_cm), VF("turn_lead_cm", CF_F32, turn_lead_cm),
    VF("camera.apply", CF_BOOL, camera.apply), VF("camera.auto_exposure", CF_BOOL, camera.auto_exposure),
    VF("camera.exposure", CF_I16, camera.exposure), VF("camera.ae_level", CF_I8, camera.ae_level),
    VF("camera.auto_gain", CF_BOOL, camera.auto_gain), VF("camera.gain", CF_U8, camera.gain),
    VF("camera.auto_wb", CF_BOOL, camera.auto_wb), VF("camera.wb_mode", CF_I8, camera.wb_mode),
    VF("camera.brightness", CF_I8, camera.brightness), VF("camera.contrast", CF_I8, camera.contrast),
    VF("camera.saturation", CF_I8, camera.saturation), VF("camera.hmirror", CF_BOOL, camera.hmirror),
    VF("camera.vflip", CF_BOOL, camera.vflip),
    VF("camera.lock_after_frames", CF_U16, camera.lock_after_frames),
};
#define FIELD_COUNT (sizeof(FIELDS) / sizeof(FIELDS[0]))

bool vision_config_set(vision_config_t *cfg, const char *key, const char *value)
{
    return cf_set(cfg, FIELDS, FIELD_COUNT, key, value);
}

size_t vision_config_format(const vision_config_t *cfg, char *out, size_t capacity)
{
    return cf_format(cfg, FIELDS, FIELD_COUNT, out, capacity);
}

bool vision_config_parse_text(vision_config_t *cfg, const char *text, char *error, size_t error_size)
{
    vision_config_t work;
    if (!cfg || !cf_parse_text(cfg, &work, sizeof(work), FIELDS, FIELD_COUNT, text, error, error_size))
        return false;
    vision_config_sanitize(cfg);
    return true;
}

float vision_calibrate_focal(float pillar_width_cm, float width_px, float distance_cm)
{
    if (!(pillar_width_cm > 0.0f) || !(width_px > 0.0f) || !(distance_cm > 0.0f)) return -1.0f;
    return width_px * distance_cm / pillar_width_cm;
}

float vision_calibrate_horizon(float camera_height_cm, float focal_px,
                               float base_row, float distance_cm)
{
    if (!(camera_height_cm > 0.0f) || !(focal_px > 0.0f) || !(distance_cm > 0.0f))
        return -1000.0f;
    /* base_row = v0 + f tan(alpha - theta) with tan(alpha) = h / D: solve theta
     * and return the horizon row v0 - f tan(theta). v0 is the middle of the
     * 120-row QQVGA image. */
    const float v0 = 60.0f;
    const float alpha = atanf(camera_height_cm / distance_cm);
    const float beta = atanf((base_row - v0) / focal_px);
    const float theta = alpha - beta;
    return v0 - focal_px * tanf(theta);
}

/* ------------------------------------------------------------- classification */

static bool hue_in(int h, int lo, int hi)
{
    return lo <= hi ? (h >= lo && h <= hi) : (h >= lo || h <= hi);
}

static int classify(const vision_config_t *cfg, uint16_t p)
{
    int r = ((p >> 11) & 31) * 255 / 31;
    int g = ((p >> 5) & 63) * 255 / 63;
    int b = (p & 31) * 255 / 31;
    int max = r > g ? r : g; if (b > max) max = b;
    int min = r < g ? r : g; if (b < min) min = b;
    int delta = max - min;
    int v = max * 100 / 255, s = max ? delta * 100 / max : 0;
    int h = 0;
    if (delta) {
        if (max == r) h = 60 * (g - b) / delta;
        else if (max == g) h = 120 + 60 * (b - r) / delta;
        else h = 240 + 60 * (r - g) / delta;
        if (h < 0) h += 360;
    }
    if (delta == 0) return 0;
    static const uint8_t order[] = {VISION_CLASS_RED, VISION_CLASS_GREEN, VISION_CLASS_MAGENTA,
                                    VISION_CLASS_ORANGE, VISION_CLASS_BLUE};
    for (size_t i = 0; i < sizeof(order); ++i) {
        const vision_hsv_window_t *w = &cfg->hsv[order[i]];
        if (w->enabled && s >= w->sat_min && v >= w->val_min &&
            hue_in(h, w->hue_lo, w->hue_hi)) return order[i];
    }
    return 0;
}

static uint16_t pixel_at(const uint8_t *data, int width, int x, int y)
{
    const size_t i = ((size_t)y * (size_t)width + (size_t)x) * 2u;
    /* esp32-camera RGB565: high byte first, no unaligned uint16 cast. */
    return (uint16_t)((uint16_t)data[i] << 8 | data[i + 1]);
}

static void reset_box(vision_box_t *b)
{
    *b = (vision_box_t){.min_x = VISION_WIDTH, .min_y = VISION_HEIGHT,
                          .distance_cm = 999.0f};
}

/* ------------------------------------------------------------------ decision */

static float turn_trigger_cm(const vision_config_t *cfg, const vision_box_t *b,
                             int direction, int scale)
{
    /* Start only when the remaining distance is needed by the turn plus the
     * clearance and half of the nominal block depth. */
    float edge_px = (direction > 0 ? b->max_x : b->min_x) / (float)scale;
    float edge_cm = (edge_px - 80.0f) * b->distance_cm / cfg->focal_px;
    float clear = cfg->turn_clearance_cm;
    float shift = direction > 0 ? edge_cm + clear : clear - edge_cm;
    float radius = cfg->turn_radius_cm;
    if (shift < 0.0f) shift = 0.0f;
    if (shift > radius * 1.9f) shift = radius * 1.9f;
    return radius * acosf(1.0f - shift / radius) + cfg->turn_lead_cm;
}

static bool blocks_path(const vision_config_t *cfg, const vision_box_t *b, int scale)
{
    if (!b->detected || b->distance_cm > cfg->act_distance_cm) return false;
    /* Every obstacle color blocks when its box overlaps the driving corridor. */
    return b->max_x >= cfg->corridor_left_px * scale && b->min_x <= cfg->corridor_right_px * scale;
}

static int forced_direction(const vision_config_t *cfg, int color)
{
    if (!cfg->enforce_color_side) return 0;
    if (color == VISION_CLASS_RED) return cfg->pass_side_red;
    if (color == VISION_CLASS_GREEN) return cfg->pass_side_green;
    return 0;
}

static vision_action_t decide(const vision_config_t *cfg, const vision_result_t *r,
                              bool parking_mode, int scale)
{
    const vision_box_t *b = NULL;
    int color = 0;
    for (int i = 0; i < r->object_count; ++i) {
        const vision_object_t *candidate = &r->objects[i];
        if (blocks_path(cfg, &candidate->box, scale) &&
            (!b || candidate->box.distance_cm < b->distance_cm)) {
            b = &candidate->box;
            color = candidate->color;
        }
    }
    if (b) {
        bool left_open = b->min_x >= cfg->open_left_px * scale;
        bool right_open = b->max_x <= cfg->open_right_px * scale;
        for (int i = 0; i < r->object_count; ++i) {
            const vision_object_t *other = &r->objects[i];
            if (&other->box == b || other->box.distance_cm > b->distance_cm + 12.0f) continue;
            if (other->box.min_x < b->min_x && other->box.max_x >= cfg->open_left_px * scale)
                left_open = false;
            if (other->box.max_x > b->max_x && other->box.min_x <= cfg->open_right_px * scale)
                right_open = false;
        }
        if (!left_open && !right_open) return V_STOP;
        int direction = forced_direction(cfg, color);
        if (!direction) {
            if (left_open != right_open) direction = left_open ? -1 : 1;
            else if (b->max_x < 80 * scale) direction = 1;
            else if (b->min_x > 80 * scale) direction = -1;
            else direction = color == VISION_CLASS_RED ? 1 : -1;
        }
        if (b->distance_cm > turn_trigger_cm(cfg, b, direction, scale)) return V_FORWARD;
        if (b->distance_cm <= cfg->hard_distance_cm)
            return direction > 0 ? V_RIGHT_HARD : V_LEFT_HARD;
        return direction > 0 ? V_RIGHT_SOFT : V_LEFT_SOFT;
    }
    if (parking_mode) {
        if (!r->parking.detected) return V_STOP;
        int cx = (r->parking.min_x + r->parking.max_x) / 2;
        if (cx < cfg->park_left_px * scale) return V_PARK_LEFT;
        if (cx > cfg->park_right_px * scale) return V_PARK_RIGHT;
        /* Image cue only: controller must confirm distance with real sensors. */
        return r->parking.max_y >= cfg->park_ready_row * scale ? V_PARK_READY : V_PARK_APPROACH;
    }
    return V_FORWARD;
}

/* -------------------------------------------------------------------- detect */

/* The flood fill works on a 2-pixel grid. Once a component is accepted, the box
 * is re-measured on the full-resolution pixels so the width and the base row
 * are exact to one pixel: at 80 cm a pillar is ~7 px wide, and a 2-pixel
 * quantization alone is a 25 % distance error. */
static void refine_box(const uint8_t *data, int width, int height,
                       const vision_config_t *cfg, int color, vision_box_t *b)
{
    int x0 = b->min_x - 2, x1 = b->max_x + 3, y0 = b->min_y - 2, y1 = b->max_y + 3;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > width) x1 = width;
    if (y1 > height) y1 = height;
    int minx = width, maxx = -1, miny = height, maxy = -1, area = 0;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            if (classify(cfg, pixel_at(data, width, x, y)) != color) continue;
            ++area;
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
        }
    }
    if (area == 0) return;
    b->min_x = minx; b->max_x = maxx; b->min_y = miny; b->max_y = maxy;
    b->area_px = area;
}

static bool base_occluded(const uint8_t *data, int width, int height,
                          const vision_config_t *cfg, const vision_box_t *b)
{
    for (int dy = 1; dy <= 2; ++dy) {
        const int y = b->max_y + dy;
        if (y >= height) break;
        for (int x = b->min_x; x <= b->max_x; x += 2) {
            const int cls = classify(cfg, pixel_at(data, width, x, y));
            if (cls >= VISION_CLASS_RED && cls <= VISION_CLASS_MAGENTA) return true;
        }
    }
    return false;
}

/* Distancia horizontal al pie de un pilar.
 *
 * Sin geometria (use_ground_distance == false) solo se conoce el ancho de la
 * caja: distancia = ancho_real x focal / ancho_en_imagen, que es la profundidad
 * sobre el eje optico. Con la geometria del montaje medida (altura de la lente y
 * fila del horizonte, es decir su cabeceo) se usa el modelo completo:
 *   - fila del pie  ->  distancia al suelo, exacta con la camara inclinada;
 *   - el ancho se corrige por la inclinacion y por la altura del pilar, y solo
 *     sirve para detectar pilares pegados.
 * Un objeto cuyo pie cae por encima del horizonte no esta sobre el suelo del
 * campo (publico, un cartel al fondo): se descarta. */
static bool measure_distance(const uint8_t *data, int width, int height, int scale,
                             const vision_config_t *cfg, vision_box_t *b)
{
    const float span_q = (float)(b->max_x - b->min_x + 1) / (float)scale;
    const bool side_clipped = b->min_x <= 0 || b->max_x >= width - 1;
    const bool bottom_clipped = b->max_y >= cfg->roi_end * scale - 1 || b->max_y >= height - 1;
    if (side_clipped || bottom_clipped) b->flags |= VISION_FLAG_CLIPPED;
    const float depth_cm = cfg->pillar_width_cm * cfg->focal_px / span_q;
    if (!cfg->use_ground_distance) {
        b->distance_cm = depth_cm;
        return true;
    }
    const float v0 = (float)height / (float)scale * 0.5f;
    const float theta = atanf((v0 - cfg->horizon_row) / cfg->focal_px);   /* cabeceo hacia abajo */
    const float base_q = (float)(b->max_y + 1) / (float)scale;
    const float beta = atanf((base_q - v0) / cfg->focal_px);
    const float alpha = theta + beta;                                      /* angulo bajo el horizonte */
    if (alpha < 0.004f) return false;                                      /* no esta sobre el suelo */
    /* Ancho -> profundidad optica -> distancia horizontal (el centro del pilar
     * esta pillar_height/2 sobre el suelo y la lente a camera_height). */
    const float dh = cfg->pillar_height_cm * 0.5f - cfg->camera_height_cm;
    const float width_cm = (depth_cm + dh * sinf(theta)) / cosf(theta);
    if (bottom_clipped) {
        b->distance_cm = width_cm;
        return true;
    }
    if (base_occluded(data, width, height, cfg, b)) {
        b->flags |= VISION_FLAG_OCCLUDED;
        b->distance_cm = width_cm;
        return true;
    }
    const float ground_cm = cfg->camera_height_cm / tanf(alpha);
    b->flags |= VISION_FLAG_GROUND;
    b->distance_cm = ground_cm;
    if (!side_clipped && width_cm < cfg->merged_ratio * ground_cm) b->flags |= VISION_FLAG_MERGED;
    return true;
}

bool vision_process_cfg(const uint8_t *data, size_t length, int width, int height,
                        bool parking_mode, const vision_config_t *cfg_in,
                        vision_workspace_t *w, vision_result_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out)); out->action = V_STOP;
    reset_box(&out->red); reset_box(&out->green); reset_box(&out->parking);
    if (!data || !w || (width != 160 && width != 320) || height != width * 3 / 4 ||
        length < (size_t)width * height * 2) return false;
    vision_config_t local;
    if (!cfg_in) { vision_config_default(&local); cfg_in = &local; }
    const vision_config_t *cfg = cfg_in;
    const int scale = width / 160;
    const int grid_w = width / 2;
    const int samples = grid_w * (height / 2);
    const int roi_y0 = cfg->roi_start * scale, roi_y1 = cfg->roi_end * scale;
    const int line_y0 = cfg->line_row_start * scale, line_y1 = cfg->line_row_end * scale;
    int scan_y0 = roi_y0 < line_y0 ? roi_y0 : line_y0;
    int scan_y1 = roi_y1 > line_y1 ? roi_y1 : line_y1;
    if (scan_y1 > height) scan_y1 = height;
    uint16_t line_count[2][VISION_HEIGHT / 2];
    memset(line_count, 0, sizeof(line_count));
    memset(w->labels, 0, sizeof(w->labels));
    for (int y = scan_y0 & ~1; y < scan_y1; y += 2) {
        const bool in_roi = y >= roi_y0 && y < roi_y1;
        const bool in_line = y >= line_y0 && y < line_y1;
        for (int x = 0; x < width; x += 2) {
            const int cls = classify(cfg, pixel_at(data, width, x, y));
            if (cls >= VISION_CLASS_RED && cls <= VISION_CLASS_MAGENTA) {
                if (in_roi) w->labels[(y / 2) * grid_w + x / 2] = (uint8_t)cls;
            } else if (cls >= VISION_CLASS_ORANGE && in_line) {
                ++line_count[cls - VISION_CLASS_ORANGE][y / 2];
            }
        }
    }
    for (int k = 0; k < 2; ++k) {
        for (int y = scan_y1 - 1; y >= scan_y0; --y) {
            if (line_count[k][y / 2] >= (uint16_t)(cfg->line_min_samples * scale)) {
                const int row = y / scale;
                if (k == 0) out->line_orange_row = (uint8_t)(row > 0 ? row : 1);
                else out->line_blue_row = (uint8_t)(row > 0 ? row : 1);
                break;
            }
        }
    }
    for (int seed = 0; seed < samples; ++seed) {
        int color = w->labels[seed]; if (!color) continue;
        vision_box_t b; reset_box(&b);
        int head = 0, tail = 0;
        w->queue[tail++] = (uint16_t)seed; w->labels[seed] = 0;
        while (head < tail) {
            int i = w->queue[head++], gx = i % grid_w, gy = i / grid_w;
            int x = gx * 2, y = gy * 2;
            ++b.count;
            if (x < b.min_x) b.min_x = x;
            if (x > b.max_x) b.max_x = x;
            if (y < b.min_y) b.min_y = y;
            if (y > b.max_y) b.max_y = y;
            int neighbors[4] = { gx > 0 ? i - 1 : -1, gx + 1 < grid_w ? i + 1 : -1,
                                gy > 0 ? i - grid_w : -1,
                                gy + 1 < height / 2 ? i + grid_w : -1 };
            for (int n = 0; n < 4; ++n) {
                int j = neighbors[n];
                if (j >= 0 && w->labels[j] == color) {
                    w->labels[j] = 0; w->queue[tail++] = (uint16_t)j;
                }
            }
        }
        int span = b.max_x - b.min_x + 2;
        b.detected = b.count >= cfg->min_samples * scale * scale && span >= cfg->min_span_px * scale;
        if (!b.detected) continue;
        refine_box(data, width, height, cfg, color, &b);
        if (!measure_distance(data, width, height, scale, cfg, &b)) { b.detected = false; continue; }
        vision_box_t *best = color == VISION_CLASS_RED ? &out->red
                           : color == VISION_CLASS_GREEN ? &out->green : &out->parking;
        if (color == VISION_CLASS_RED || color == VISION_CLASS_GREEN) {
            const int limit = cfg->max_objects;
            if (out->object_count < limit) {
                out->objects[out->object_count++] = (vision_object_t){.box = b, .color = (uint8_t)color};
            } else {
                int farthest = 0;
                for (int k = 1; k < limit; ++k)
                    if (out->objects[k].box.distance_cm > out->objects[farthest].box.distance_cm) farthest = k;
                if (b.distance_cm < out->objects[farthest].box.distance_cm)
                    out->objects[farthest] = (vision_object_t){.box = b, .color = (uint8_t)color};
            }
        }
        /* Never merge separated objects into one artificially wide box. */
        bool replace = !best->detected;
        if (color == VISION_CLASS_MAGENTA) replace = replace || b.count > best->count;
        else if (blocks_path(cfg, &b, scale) != blocks_path(cfg, best, scale))
            replace = blocks_path(cfg, &b, scale);
        else replace = replace || b.distance_cm < best->distance_cm;
        if (replace) *best = b;
    }
    out->valid = true; out->action = decide(cfg, out, parking_mode, scale); return true;
}

bool vision_process(const uint8_t *data, size_t length, int width, int height,
                    bool parking_mode, vision_workspace_t *w, vision_result_t *out)
{
    return vision_process_cfg(data, length, width, height, parking_mode, NULL, w, out);
}

const char *vision_action_name(vision_action_t a)
{
    static const char *const names[] = {"STOP", "FORWARD", "RIGHT_SOFT", "RIGHT_HARD",
        "LEFT_SOFT", "LEFT_HARD", "PARK_LEFT", "PARK_RIGHT", "PARK_APPROACH", "PARK_READY",
        "RETURN_LEFT", "RETURN_RIGHT"};
    return (unsigned)a < sizeof(names) / sizeof(names[0]) ? names[a] : "STOP";
}

const char *vision_class_name(int cls)
{
    static const char *const names[] = {"none", "red", "green", "magenta", "orange", "blue"};
    return (unsigned)cls < VISION_CLASS_COUNT ? names[cls] : "none";
}

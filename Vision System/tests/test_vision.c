#include "vision.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static uint8_t frame[160 * 120 * 2];
static uint8_t qvga_frame[320 * 240 * 2];
static vision_workspace_t workspace;
static vision_result_t result;
static vision_config_t cfg;
static int checks;
#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

#define F 118.6f
static uint16_t rgb(int r, int g, int b)
{
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}
/* Official WRO colors. */
static uint16_t red565(void) { return rgb(238, 39, 55); }
static uint16_t green565(void) { return rgb(68, 214, 44); }
static uint16_t magenta565(void) { return rgb(255, 0, 255); }
static uint16_t orange565(void) { return rgb(255, 102, 0); }
static uint16_t blue565(void) { return rgb(0, 51, 255); }

static void rect(int x, int y, int width, int height, uint16_t color)
{
    for (int j = y; j < y + height; ++j) for (int i = x; i < x + width; ++i) {
        if (i < 0 || i >= 160 || j < 0 || j >= 120) continue;
        size_t p = ((size_t)j * 160 + i) * 2;
        frame[p] = (uint8_t)(color >> 8); frame[p + 1] = (uint8_t)color;
    }
}
static void clear(void)
{
    for (size_t i = 0; i < sizeof(frame); i += 2) { frame[i] = 0x52; frame[i + 1] = 0x8a; } /* gray floor */
}
/* The camera model the firmware assumes: pinhole, 5 cm high, horizon at row 30,
 * pillar 5 x 10 cm standing on the mat, optical depth `depth`. */
static void pillar(float lateral_cm, float depth, uint16_t color)
{
    const float cx = 80.0f + F * lateral_cm / depth, w = F * 5.0f / depth;
    const float bottom = 30.0f + F * 5.0f / depth, top = bottom - F * 10.0f / depth;
    const int x0 = (int)lroundf(cx - w / 2.0f), x1 = (int)lroundf(cx + w / 2.0f);
    rect(x0, (int)lroundf(top), x1 - x0, (int)lroundf(bottom) - (int)lroundf(top), color);
}
/* Camara inclinada hacia abajo (horizonte en la fila `horizon`), lente a `cam_h`
 * cm del suelo: la proyeccion exacta que usa el simulador del navegador. Un
 * punto a `fwd` cm delante y `up` cm sobre la lente cae en la fila devuelta. */
static float proj_row(float horizon, float fwd, float up)
{
    const float theta = atanf((60.0f - horizon) / F), yd = -up;
    const float zc = fwd * cosf(theta) + yd * sinf(theta), yc = -fwd * sinf(theta) + yd * cosf(theta);
    return 60.0f + F * yc / zc;
}
static void pillar_pitched(float lateral_cm, float distance_cm, float cam_h, float horizon, uint16_t color)
{
    const float theta = atanf((60.0f - horizon) / F);
    const float mid_up = 5.0f - cam_h;
    const float zc = distance_cm * cosf(theta) - mid_up * sinf(theta);
    const float w = F * 5.0f / zc, cx = 80.0f + F * lateral_cm / zc;
    const float bottom = proj_row(horizon, distance_cm, -cam_h), top = proj_row(horizon, distance_cm, 10.0f - cam_h);
    const int x0 = (int)lroundf(cx - w / 2.0f), x1 = (int)lroundf(cx + w / 2.0f);
    rect(x0, (int)lroundf(top), x1 - x0, (int)lroundf(bottom) - (int)lroundf(top), color);
}
static void upscale_to_qvga(void)
{
    for (int y = 0; y < 120; ++y) for (int x = 0; x < 160; ++x) {
        const size_t src = ((size_t)y * 160 + x) * 2;
        for (int dy = 0; dy < 2; ++dy) for (int dx = 0; dx < 2; ++dx) {
            const size_t dst = ((size_t)(y * 2 + dy) * 320 + x * 2 + dx) * 2;
            qvga_frame[dst] = frame[src]; qvga_frame[dst + 1] = frame[src + 1];
        }
    }
}
static void process(bool parking)
{
    CHECK(vision_process_cfg(frame, sizeof(frame), 160, 120, parking, &cfg, &workspace, &result));
    CHECK(result.valid);
}
static float rel(float a, float b) { return fabsf(a - b) / b; }

int main(void)
{
    vision_config_default(&cfg);

    /* --- defaults reproduce the calibrated behavior ------------------------ */
    clear(); process(false); CHECK(result.action == V_FORWARD);
    clear(); pillar(0, 30, red565()); process(false);
    CHECK(result.red.detected && !result.green.detected);
    CHECK(result.action == V_RIGHT_SOFT);
    CHECK(fabsf(result.red.distance_cm - 29.65f) < 0.01f);
    CHECK(result.red.min_x == 70 && result.red.max_x == 89);
    CHECK(!(result.red.flags & VISION_FLAG_CLIPPED));
    clear(); pillar(0, 30, green565()); process(false); CHECK(result.action == V_LEFT_SOFT);
    clear(); pillar(0, 40, red565()); process(false); CHECK(result.action == V_FORWARD);
    clear(); pillar(0, 12, red565()); process(false); CHECK(result.action == V_RIGHT_HARD);
    clear(); pillar(0, 12, green565()); process(false); CHECK(result.action == V_LEFT_HARD);
    /* The pure vision default does not trust the unmeasured camera mount. */
    CHECK(!cfg.use_ground_distance);

    /* --- color rule is data, not code -------------------------------------- */
    /* A pillar left of center still has to be passed on its color's side. */
    clear(); pillar(-4, 30, green565()); process(false); CHECK(result.action == V_LEFT_SOFT);
    clear(); pillar(4, 30, red565()); process(false); CHECK(result.action == V_RIGHT_SOFT);
    cfg.pass_side_red = -1; cfg.pass_side_green = 1; /* swapped venue */
    clear(); pillar(0, 30, red565()); process(false); CHECK(result.action == V_LEFT_SOFT);
    clear(); pillar(0, 30, green565()); process(false); CHECK(result.action == V_RIGHT_SOFT);
    cfg.enforce_color_side = false; /* color ignored: freer side, ties by color */
    clear(); pillar(-6, 24, green565()); process(false); CHECK(result.action == V_RIGHT_SOFT);
    clear(); pillar(6, 24, red565()); process(false); CHECK(result.action == V_LEFT_SOFT);
    vision_config_default(&cfg);

    /* --- offsets and the corridor ------------------------------------------ */
    clear(); pillar(-30, 30, red565()); process(false); CHECK(result.action == V_FORWARD);
    clear(); pillar(30, 30, green565()); process(false); CHECK(result.action == V_FORWARD);
    /* Closest blocking pillar wins; a passed one at the side does not hide it. */
    clear(); pillar(-25, 20, red565()); pillar(0, 30, green565());
    process(false); CHECK(result.action == V_LEFT_SOFT);
    /* Both sides taken: stop. */
    clear(); pillar(-5, 25, red565()); pillar(5, 25, green565());
    process(false); CHECK(result.action == V_STOP);

    /* --- exact width and distance ------------------------------------------ */
    for (float d = 25; d <= 70; d += 5) {
        clear(); pillar(0, d, red565()); process(false);
        CHECK(result.red.detected);
        const float expected = d;
        /* One pixel of quantization at most: width ~ F*5/d. */
        const float tolerance = 1.0f / (F * 5.0f / d) + 0.02f;
        CHECK(rel(result.red.distance_cm, expected) <= tolerance);
    }

    /* --- ground-contact distance, once the mount is configured ------------- */
    /* The camera looks 14 degrees down from 5 cm up, so the width of a pillar is
     * not the whole story: the base row gives the floor distance exactly. */
    cfg.use_ground_distance = true;
    for (float d = 25; d <= 70; d += 5) {
        clear(); pillar_pitched(0, d, 5.0f, 30.0f, green565()); process(false);
        CHECK(result.green.detected);
        CHECK(result.green.flags & VISION_FLAG_GROUND);
        /* One pixel of base row is d / (F * height) of the distance. */
        CHECK(rel(result.green.distance_cm, d) <= 0.65f * d / (F * 5.0f) + 0.01f);
    }
    /* Another mount: 9 cm high, horizon at row 22. */
    cfg.camera_height_cm = 9.0f; cfg.horizon_row = 22.0f;
    for (float d = 25; d <= 70; d += 15) {
        clear(); pillar_pitched(5, d, 9.0f, 22.0f, red565()); process(false);
        CHECK(result.red.detected && (result.red.flags & VISION_FLAG_GROUND));
        CHECK(rel(result.red.distance_cm, d) <= 0.65f * d / (F * 9.0f) + 0.02f);
    }
    cfg.camera_height_cm = 5.0f; cfg.horizon_row = 30.0f;
    /* A pillar cut at the bottom of the region of interest cannot be ranged. */
    clear(); rect(70, 60, 20, 50, red565()); process(false);
    CHECK(result.red.detected && (result.red.flags & VISION_FLAG_CLIPPED));
    CHECK(!(result.red.flags & VISION_FLAG_GROUND));
    /* Wide because two pillars touch: width says "closer" than the floor does. */
    clear(); pillar_pitched(-2.5f, 40, 5.0f, 30.0f, red565()); pillar_pitched(2.5f, 40, 5.0f, 30.0f, red565());
    process(false);
    CHECK(result.red.detected && (result.red.flags & VISION_FLAG_MERGED));
    CHECK(rel(result.red.distance_cm, 40.0f) < 0.1f);
    /* A nearer pillar covers the far base: do not trust the ground row. */
    clear(); pillar_pitched(0, 60, 5.0f, 30.0f, red565()); pillar_pitched(0, 30, 5.0f, 30.0f, green565());
    process(false);
    for (int i = 0; i < result.object_count; ++i)
        if (result.objects[i].color == VISION_CLASS_RED)
            CHECK(result.objects[i].box.flags & VISION_FLAG_OCCLUDED);
    /* Something red whose base is above the horizon (a person behind the wall,
     * a poster) is not on the floor of the field: ignored. */
    clear(); rect(60, 22, 30, 7, red565()); process(false);
    CHECK(!result.red.detected && result.object_count == 0);
    /* The same blob is accepted when the geometry is unknown: the width-only
     * default cannot tell, which is exactly why the geometry is configurable. */
    cfg.use_ground_distance = false; process(false); CHECK(result.red.detected);
    cfg.use_ground_distance = true;
    /* Calibration helpers recover the mount from one sample. */
    CHECK(fabsf(vision_calibrate_horizon(7.0f, F, proj_row(38.0f, 45.0f, -7.0f), 45.0f) - 38.0f) < 0.05f);
    CHECK(fabsf(vision_calibrate_horizon(5.0f, F, proj_row(30.0f, 30.0f, -5.0f), 30.0f) - 30.0f) < 0.05f);
    CHECK(fabsf(vision_calibrate_focal(5.0f, 13.17f, 45.0f) - 118.53f) < 0.1f);
    CHECK(vision_calibrate_focal(0, 1, 1) < 0 && vision_calibrate_horizon(5, F, 40, 0) < -999.0f);
    vision_config_default(&cfg);

    /* --- clipping at the image side ---------------------------------------- */
    clear(); rect(0, 40, 12, 30, red565()); process(false);
    CHECK(result.red.detected && (result.red.flags & VISION_FLAG_CLIPPED));

    /* --- QVGA keeps the same physical answers ------------------------------ */
    clear(); pillar(0, 30, red565()); upscale_to_qvga();
    CHECK(vision_process_cfg(qvga_frame, sizeof(qvga_frame), 320, 240, false, &cfg, &workspace, &result));
    CHECK(result.valid && result.action == V_RIGHT_SOFT);
    CHECK(fabsf(result.red.distance_cm - 29.65f) < 0.01f);
    CHECK(result.red.min_x == 140);

    /* --- official colors, floor lines and distractors ----------------------- */
    clear(); rect(40, 40, 50, 40, blue565()); process(false);
    CHECK(!result.red.detected && !result.green.detected);
    clear(); rect(40, 40, 50, 40, 0xffff); process(false);
    CHECK(!result.red.detected && !result.green.detected && !result.parking.detected);
    clear(); rect(0, 88, 160, 4, orange565()); process(false);
    CHECK(result.line_orange_row >= 90 && result.line_orange_row <= 92);
    CHECK(result.line_blue_row == 0 && !result.red.detected);
    clear(); rect(0, 70, 160, 4, blue565()); process(false);
    CHECK(result.line_blue_row >= 72 && result.line_blue_row <= 74 && result.line_orange_row == 0);
    /* An orange mat line must not be taken for a red pillar, even wide and close. */
    clear(); rect(30, 80, 100, 14, orange565()); process(false);
    CHECK(!result.red.detected && result.action == V_FORWARD);
    clear(); pillar(0, 30, red565()); rect(0, 100, 160, 3, orange565()); process(false);
    CHECK(result.red.detected && result.line_orange_row >= 100);
    /* Dark and bright lighting still classify the pillar. */
    clear(); rect(70, 40, 20, 40, rgb(140, 20, 30)); process(false); CHECK(result.red.detected);
    clear(); rect(70, 40, 20, 40, rgb(250, 130, 140)); process(false); CHECK(!result.red.detected);

    /* --- adaptability: thresholds change without rebuilding ----------------- */
    cfg.hsv[VISION_CLASS_RED].sat_min = 95; /* much stricter */
    clear(); pillar(0, 30, red565()); process(false); CHECK(!result.red.detected);
    vision_config_default(&cfg);
    cfg.hsv[VISION_CLASS_RED].enabled = false;
    clear(); pillar(0, 30, red565()); process(false); CHECK(!result.red.detected);
    vision_config_default(&cfg);
    cfg.min_samples = 400;
    clear(); pillar(0, 30, red565()); process(false); CHECK(!result.red.detected);
    vision_config_default(&cfg);
    cfg.max_objects = 1;
    clear(); pillar(-30, 40, red565()); pillar(0, 60, green565()); pillar(30, 50, red565());
    process(false); CHECK(result.object_count == 1);
    vision_config_default(&cfg);

    /* --- wire format -------------------------------------------------------- */
    uint8_t wire[VISION_CONFIG_WIRE_MAX];
    vision_config_t a, b;
    vision_config_default(&a);
    a.pass_side_red = -1; a.pass_side_green = 1; a.horizon_row = 33.5f;
    a.hsv[VISION_CLASS_GREEN].hue_lo = 70; a.camera.apply = true;
    a.camera.exposure = 640; a.camera.lock_after_frames = 90; a.camera.wb_mode = 3;
    size_t n = vision_config_encode(&a, wire, sizeof(wire));
    CHECK(n > 100 && n <= VISION_CONFIG_WIRE_MAX);
    CHECK(vision_config_decode(wire, n, &b));
    CHECK(b.pass_side_red == -1 && b.pass_side_green == 1 && b.horizon_row == 33.5f);
    CHECK(b.hsv[VISION_CLASS_GREEN].hue_lo == 70 && b.camera.apply);
    CHECK(b.camera.exposure == 640 && b.camera.lock_after_frames == 90 && b.camera.wb_mode == 3);
    CHECK(b.hsv[VISION_CLASS_RED].hue_lo == 335 && b.roi_end == 110);
    wire[10] ^= 0x40;
    CHECK(!vision_config_decode(wire, n, &b));
    wire[10] ^= 0x40;
    CHECK(!vision_config_decode(wire, n - 1, &b));
    CHECK(vision_config_encode(&a, wire, 20) == 0);
    /* Out-of-range remote values are clamped instead of crashing the detector. */
    a.roi_end = 5000; a.focal_px = NAN; a.camera.exposure = -4; a.pass_side_red = 9;
    CHECK(vision_config_sanitize(&a));
    CHECK(a.roi_end == 120 && a.focal_px == 40.0f && a.camera.exposure == 0 && a.pass_side_red == 0);
    CHECK(!vision_config_sanitize(&a));

    /* --- text interface: edit a file instead of recompiling ------------------ */
    {
        vision_config_t t, keep, round;
        char err[96], text[4096];
        uint8_t w1[VISION_CONFIG_WIRE_MAX], w2[VISION_CONFIG_WIRE_MAX];
        vision_config_default(&t);
        CHECK(vision_config_parse_text(&t, "# venue\npass_side_red = -1\nred.hue_lo 330\n"
            "camera.apply on\ncamera.exposure 500 # manual\nfocal_px 120.5\n", err, sizeof(err)));
        CHECK(t.pass_side_red == -1 && t.hsv[VISION_CLASS_RED].hue_lo == 330);
        CHECK(t.camera.apply && t.camera.exposure == 500 && fabsf(t.focal_px - 120.5f) < 1e-4f);
        keep = t;
        CHECK(!vision_config_parse_text(&t, "focal_px 130\nnope 3\n", err, sizeof(err)));
        CHECK(strstr(err, "line 2") != NULL && t.focal_px == keep.focal_px); /* all or nothing */
        const size_t len = vision_config_format(&keep, text, sizeof(text));
        CHECK(len > 600 && len < sizeof(text) - 1);
        vision_config_default(&round);
        CHECK(vision_config_parse_text(&round, text, err, sizeof(err)));
        const size_t n1 = vision_config_encode(&keep, w1, sizeof(w1));
        const size_t n2 = vision_config_encode(&round, w2, sizeof(w2));
        CHECK(n1 == n2 && memcmp(w1, w2, n1) == 0); /* every field survives text -> struct */
        CHECK(!vision_config_set(&t, "pass_side_red", "abc") && !vision_config_set(NULL, "a", "b"));
        CHECK(vision_config_set(&t, "pass_side_red", "100") && t.pass_side_red == 100);
        CHECK(vision_config_sanitize(&t) && t.pass_side_red == 0);
    }

    /* --- parking marker ----------------------------------------------------- */
    clear(); rect(70, 40, 20, 30, magenta565()); process(false); CHECK(result.action == V_FORWARD);
    process(true); CHECK(result.action == V_PARK_APPROACH);
    clear(); process(true); CHECK(result.action == V_STOP);
    rect(30, 40, 20, 30, magenta565()); process(true); CHECK(result.action == V_PARK_LEFT);
    clear(); rect(110, 40, 20, 30, magenta565()); process(true); CHECK(result.action == V_PARK_RIGHT);
    clear(); rect(70, 70, 20, 38, magenta565()); process(true); CHECK(result.action == V_PARK_READY);
    rect(100, 40, 50, 40, green565()); process(true);
    CHECK(result.action == V_LEFT_HARD);

    /* --- ROI and bad input -------------------------------------------------- */
    clear(); rect(40, 0, 50, 20, red565()); rect(40, 110, 50, 10, green565());
    process(false); CHECK(!result.red.detected && !result.green.detected);
    clear(); rect(40, 40, 8, 8, red565()); process(false); CHECK(!result.red.detected);
    CHECK(!vision_process(NULL, sizeof(frame), 160, 120, false, &workspace, &result));
    CHECK(result.action == V_STOP);
    CHECK(!vision_process(frame, 2, 160, 120, false, &workspace, &result)); CHECK(!result.valid);
    CHECK(!vision_process(frame, sizeof(frame), 320, 240, false, &workspace, &result));
    CHECK(!vision_process(frame, sizeof(frame), 160, 120, false, NULL, &result));
    CHECK(!vision_process(frame, sizeof(frame), 160, 120, false, &workspace, NULL));
    clear(); rect(0, 20, 160, 90, red565()); process(false);
    CHECK(result.red.count == 3600); CHECK(result.action == V_STOP);
    CHECK(strcmp(vision_action_name((vision_action_t)99), "STOP") == 0);
    CHECK(strcmp(vision_class_name(2), "green") == 0 && strcmp(vision_class_name(77), "none") == 0);

    /* Randomized buffers exercise every component boundary and workspace reuse. */
    unsigned rng = 42;
    for (int n2 = 0; n2 < 100; ++n2) {
        for (size_t i = 0; i < sizeof(frame); ++i) {
            rng = rng * 1664525u + 1013904223u; frame[i] = (uint8_t)(rng >> 24);
        }
        process(n2 % 2 != 0);
        CHECK(result.action >= V_STOP && result.action <= V_PARK_READY);
    }
    printf("PASS: %d checks (vision core, configuration, wire format, 100 random frames)\n", checks);
    return 0;
}

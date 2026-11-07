#ifndef WRO_VISION_H
#define WRO_VISION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_WIDTH 320
#define VISION_HEIGHT 240
#define VISION_SAMPLES ((VISION_WIDTH / 2) * (VISION_HEIGHT / 2))

typedef enum { V_STOP, V_FORWARD, V_RIGHT_SOFT, V_RIGHT_HARD,
    V_LEFT_SOFT, V_LEFT_HARD, V_PARK_LEFT, V_PARK_RIGHT, V_PARK_APPROACH,
    V_PARK_READY, V_RETURN_LEFT, V_RETURN_RIGHT } vision_action_t;

/* Color classes. Pillars are 1..3, floor lines are 4..5. */
typedef enum {
    VISION_CLASS_NONE = 0, VISION_CLASS_RED = 1, VISION_CLASS_GREEN = 2,
    VISION_CLASS_MAGENTA = 3, VISION_CLASS_ORANGE = 4, VISION_CLASS_BLUE = 5,
    VISION_CLASS_COUNT = 6
} vision_class_t;

/* ------------------------------------------------------------ configuration
 *
 * Everything the vision pipeline assumes about the track, the lighting and the
 * camera mount lives here, so it can change between rounds without a rebuild.
 * vision_config_default() reproduces the values calibrated for the official
 * WRO colors; a team only overrides what the venue changes. */

/* HSV window in percent / degrees. The hue interval is inclusive and wraps
 * through 0 when hue_lo > hue_hi (red is [335, 14]). */
typedef struct {
    int16_t hue_lo, hue_hi;
    uint8_t sat_min, val_min;
    bool enabled;
} vision_hsv_window_t;

/* Sensor registers. With apply == false the firmware never touches them and the
 * OV3660 keeps the driver defaults (what the camera did before this struct
 * existed). lock_after_frames lets the automatic controls converge and then
 * freezes exposure, gain and white balance, so lighting flicker or a bright
 * wall entering the frame cannot shift the color thresholds mid-run. */
typedef struct {
    bool apply;
    bool auto_exposure;
    int16_t exposure;          /* manual value, 0..1200 */
    int8_t ae_level;           /* -2..2 */
    bool auto_gain;
    uint8_t gain;              /* manual value, 0..30 */
    bool auto_wb;
    int8_t wb_mode;            /* 0 auto, 1 sunny, 2 cloudy, 3 office, 4 home */
    int8_t brightness, contrast, saturation; /* -2..2 */
    bool hmirror, vflip;
    uint16_t lock_after_frames; /* 0 = never freeze */
} vision_camera_settings_t;

typedef struct {
    vision_hsv_window_t hsv[VISION_CLASS_COUNT]; /* index = vision_class_t */

    /* Side on which the robot passes each pillar color: +1 the robot goes to the
     * RIGHT of the pillar (steers right), -1 to the LEFT, 0 no rule. WRO: red ->
     * right, green -> left, but a venue or a surprise rule may swap them. */
    int8_t pass_side_red, pass_side_green;
    bool enforce_color_side;   /* false: the camera ignores color and picks the freer side */

    /* Camera model, in QQVGA (160 x 120) pixels; scaled for 320 x 240. */
    float focal_px;
    float camera_height_cm;    /* optical axis height above the mat */
    float horizon_row;         /* image row of the horizon */
    float pillar_width_cm;     /* footprint width seen by the camera */
    float pillar_height_cm;
    bool use_ground_distance;  /* distance from the base row instead of the width */
    float merged_ratio;        /* width-distance / ground-distance below this: touching pillars */

    /* Detection. */
    int roi_start, roi_end;    /* pillar rows, QQVGA */
    int min_samples;           /* 2-pixel samples in a component */
    int min_span_px;
    int line_row_start, line_row_end; /* rows scanned for orange / blue floor lines */
    int line_min_samples;      /* samples in a row to call it a line */
    int max_objects;           /* 1..12 */

    /* Camera-only steering proposal. The P4 weighs it against the LiDAR. */
    float act_distance_cm;     /* obstacles farther than this are ignored */
    int corridor_left_px, corridor_right_px;
    int open_left_px, open_right_px;
    int park_left_px, park_right_px;
    int park_ready_row;
    float hard_distance_cm;    /* closer than this the proposal is a hard turn */
    float turn_radius_cm, turn_clearance_cm, turn_lead_cm;

    vision_camera_settings_t camera;
} vision_config_t;

void vision_config_default(vision_config_t *cfg);
/* Clamps every field to a range the pipeline can handle. Returns true if
 * something had to change. */
bool vision_config_sanitize(vision_config_t *cfg);
/* Wire format (A6 6A | version | length | payload | crc16) used to retune the
 * S3 from a PC or the P4 without reflashing. decode() sanitizes the result. */
#define VISION_CONFIG_WIRE_MAX 192u
size_t vision_config_encode(const vision_config_t *cfg, uint8_t *out, size_t capacity);
bool vision_config_decode(const uint8_t *bytes, size_t length, vision_config_t *cfg);

/* Text interface: "key value" lines ('#' starts a comment; the key names are
 * the field names, e.g. "red.hue_lo", "pass_side_red", "camera.exposure"). */
bool vision_config_set(vision_config_t *cfg, const char *key, const char *value);
size_t vision_config_format(const vision_config_t *cfg, char *out, size_t capacity);
bool vision_config_parse_text(vision_config_t *cfg, const char *text,
                              char *error, size_t error_size);

/* Calibration helpers: one measurement at a known distance fixes a parameter.
 * Each returns a negative value when the sample is unusable. */
float vision_calibrate_focal(float pillar_width_cm, float width_px, float distance_cm);
float vision_calibrate_horizon(float camera_height_cm, float focal_px,
                               float base_row, float distance_cm);

/* --------------------------------------------------------------- detection */

#define VISION_FLAG_CLIPPED  0x01u
#define VISION_FLAG_GROUND   0x02u
#define VISION_FLAG_OCCLUDED 0x04u
#define VISION_FLAG_MERGED   0x08u

typedef struct {
    int min_x, max_x, min_y, max_y, count;
    float distance_cm;     /* optical depth */
    bool detected;
    uint8_t flags;         /* VISION_FLAG_* */
    int area_px;           /* full-resolution pixels of the component */
} vision_box_t;

#define VISION_MAX_OBJECTS 12
typedef struct {
    vision_box_t box;
    uint8_t color; /* vision_class_t: 1 red, 2 green, 3 parking */
} vision_object_t;

typedef struct {
    bool valid;
    vision_box_t red, green, parking;
    vision_object_t objects[VISION_MAX_OBJECTS];
    uint8_t object_count;
    /* Lowest QQVGA row with an orange / blue floor line (0 = none). */
    uint8_t line_orange_row, line_blue_row;
    vision_action_t action;
} vision_result_t;

/* Workspace owned by caller; separate workspace per concurrent processor. */
typedef struct {
    uint8_t labels[VISION_SAMPLES];
    uint16_t queue[VISION_SAMPLES];
} vision_workspace_t;

/* cfg == NULL uses vision_config_default(). */
bool vision_process_cfg(const uint8_t *rgb565, size_t length, int width, int height,
                        bool parking_mode, const vision_config_t *cfg,
                        vision_workspace_t *work, vision_result_t *result);
bool vision_process(const uint8_t *rgb565, size_t length, int width, int height,
                    bool parking_mode, vision_workspace_t *work, vision_result_t *result);
const char *vision_action_name(vision_action_t action);
const char *vision_class_name(int cls);

#endif

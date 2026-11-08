/* Puerta de entrada del simulador al firmware REAL.
 *
 * Este archivo se compila a WebAssembly junto con el codigo del S3 (vision) y del
 * P4 (mapa, planificador, controlador de la mision): el navegador y las pruebas
 * de Node ejecutan exactamente las mismas funciones C que irian en las placas.
 * No hay una segunda implementacion en JavaScript que pueda divergir.
 *
 * La interfaz es plana: JS escribe en buffers estaticos (cuyos punteros pide a
 * core_*_ptr) y llama a las funciones. Todos los reales son float de 32 bits. */
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "camera_table_receiver.h"
#include "sensor_table_protocol.h"
#include "vision.h"
#include "vision_table.h"
#include "wro_controller.h"
#include "p4_runtime.h"

#define EXPORT(name) __attribute__((export_name(name)))

/* ------------------------------------------------------------------ vision */
static uint8_t g_frame[VISION_WIDTH * VISION_HEIGHT * 2];
static vision_workspace_t g_workspace;
static vision_config_t g_vision_cfg;
static vision_result_t g_vision;
/* cabecera: valid, action, count, orange_row, blue_row; luego 9 floats por objeto */
#define VIS_OUT_OBJ 9
static float g_vision_out[8 + (VISION_MAX_OBJECTS + 1) * VIS_OUT_OBJ];
static uint8_t g_wire[WRO_TABLE_MAX_SIZE];
static uint8_t g_wire_in[WRO_TABLE_MAX_SIZE + 8];
static char g_text[8192];
static char g_error[160];

EXPORT("core_frame_ptr") void *core_frame_ptr(void) { return g_frame; }
EXPORT("core_frame_size") int core_frame_size(void) { return (int)sizeof(g_frame); }
EXPORT("core_text_ptr") char *core_text_ptr(void) { return g_text; }
EXPORT("core_text_size") int core_text_size(void) { return (int)sizeof(g_text); }
EXPORT("core_error_ptr") char *core_error_ptr(void) { return g_error; }
EXPORT("core_vision_out_ptr") float *core_vision_out_ptr(void) { return g_vision_out; }
EXPORT("core_wire_ptr") uint8_t *core_wire_ptr(void) { return g_wire; }
EXPORT("core_wire_in_ptr") uint8_t *core_wire_in_ptr(void) { return g_wire_in; }

EXPORT("core_vision_defaults") void core_vision_defaults(void) { vision_config_default(&g_vision_cfg); }
/* Aplica el texto de core_text_ptr() a la configuracion de vision: 1 = aceptado. */
EXPORT("core_vision_config_apply") int core_vision_config_apply(void)
{
    g_error[0] = '\0';
    return vision_config_parse_text(&g_vision_cfg, g_text, g_error, sizeof(g_error)) ? 1 : 0;
}
EXPORT("core_vision_config_dump") int core_vision_config_dump(void)
{
    return (int)vision_config_format(&g_vision_cfg, g_text, sizeof(g_text));
}

static void export_vision(void)
{
    const vision_result_t *r = &g_vision;
    g_vision_out[0] = r->valid ? 1.0f : 0.0f;
    g_vision_out[1] = (float)r->action;
    g_vision_out[3] = (float)r->line_orange_row;
    g_vision_out[4] = (float)r->line_blue_row;
    int n = 0;
    for (int i = 0; i < r->object_count && n < VISION_MAX_OBJECTS; ++i, ++n) {
        const vision_object_t *o = &r->objects[i];
        float *p = &g_vision_out[8 + n * VIS_OUT_OBJ];
        p[0] = (float)o->color; p[1] = (float)o->box.min_x; p[2] = (float)o->box.max_x;
        p[3] = (float)o->box.min_y; p[4] = (float)o->box.max_y; p[5] = (float)o->box.count;
        p[6] = o->box.distance_cm; p[7] = (float)o->box.flags; p[8] = (float)o->box.area_px;
    }
    if (r->parking.detected) {
        float *p = &g_vision_out[8 + n * VIS_OUT_OBJ];
        p[0] = 3.0f; p[1] = (float)r->parking.min_x; p[2] = (float)r->parking.max_x;
        p[3] = (float)r->parking.min_y; p[4] = (float)r->parking.max_y; p[5] = (float)r->parking.count;
        p[6] = r->parking.distance_cm; p[7] = (float)r->parking.flags; p[8] = (float)r->parking.area_px;
        ++n;
    }
    g_vision_out[2] = (float)n;
}

/* Procesa el cuadro RGB565 160x120 de core_frame_ptr(). */
EXPORT("core_vision_process") int core_vision_process(int parking)
{
    const bool ok = vision_process_cfg(g_frame, 160u * 120u * 2u, 160, 120, parking != 0,
                                       &g_vision_cfg, &g_workspace, &g_vision);
    export_vision();
    return ok ? 1 : 0;
}

/* Como el S3: procesa el cuadro y lo empaqueta en la trama del enlace UART.
 * Devuelve la longitud de la trama (en core_wire_ptr()). */
EXPORT("core_camera_frame") int core_camera_frame(int sequence, unsigned timestamp_ms, int parking)
{
    core_vision_process(parking);
    wro_sensor_table_t table;
    vision_table_build(&g_vision, 160, (uint16_t)sequence, (uint32_t)timestamp_ms, &table);
    return (int)wro_sensor_table_encode(&table, g_wire, sizeof(g_wire));
}

/* -------------------------------------------------------------- controlador */
static wro_controller_t g_ctrl;
static wro_controller_config_t g_ctrl_cfg;
static camera_table_decoder_t g_decoder;
static local_lidar_point_t g_points[LOCAL_MAP_MAX_POINTS];
static float g_points_in[LOCAL_MAP_MAX_POINTS * 4];
static wro_command_t g_cmd;
#define CMD_FIELDS 86
static float g_cmd_out[CMD_FIELDS];
/* Linea de carrera: n, y despues x, z, v de cada estacion. */
static float g_race_out[2 + 3 * RACE_MAX_STATIONS + 3 * RACE_MAX_SIGNS];

static uint8_t g_map_cells[LOCAL_MAP_SIZE * LOCAL_MAP_SIZE];
static float g_debug_out[9 + 3 * LOCAL_DEBUG_ROUTE_POINTS * 2 + LOCAL_MAP_MAX_CAMERA_OBJECTS * 4];

/* Una version detecta cambios de significado; los tamaños detectan cambios de capacidad. */
EXPORT("core_abi_version") int core_abi_version(void) { return 2; }
EXPORT("core_cmd_fields") int core_cmd_fields(void) { return CMD_FIELDS; }
EXPORT("core_vision_fields") int core_vision_fields(void) { return (int)(sizeof(g_vision_out) / sizeof(g_vision_out[0])); }
EXPORT("core_debug_fields") int core_debug_fields(void) { return (int)(sizeof(g_debug_out) / sizeof(g_debug_out[0])); }
EXPORT("core_debug_route_points") int core_debug_route_points(void) { return LOCAL_DEBUG_ROUTE_POINTS; }
EXPORT("core_debug_camera_objects") int core_debug_camera_objects(void) { return LOCAL_MAP_MAX_CAMERA_OBJECTS; }
EXPORT("core_max_points") int core_max_points(void) { return LOCAL_MAP_MAX_POINTS; }

EXPORT("core_points_ptr") float *core_points_ptr(void) { return g_points_in; }
EXPORT("core_cmd_ptr") float *core_cmd_ptr(void) { return g_cmd_out; }
EXPORT("core_map_ptr") uint8_t *core_map_ptr(void) { return g_map_cells; }
EXPORT("core_map_size") int core_map_size(void) { return LOCAL_MAP_SIZE; }
EXPORT("core_map_cell_cm") float core_map_cell_cm(void) { return LOCAL_MAP_CELL_CM; }
EXPORT("core_debug_ptr") float *core_debug_ptr(void) { return g_debug_out; }
EXPORT("core_race_ptr") float *core_race_ptr(void) { return g_race_out; }

EXPORT("core_ctrl_defaults") void core_ctrl_defaults(void)
{
    wro_controller_config_default(&g_ctrl_cfg);
}
EXPORT("core_ctrl_config_apply") int core_ctrl_config_apply(void)
{
    g_error[0] = '\0';
    return wro_controller_config_parse_text(&g_ctrl_cfg, g_text, g_error, sizeof(g_error)) ? 1 : 0;
}
EXPORT("core_ctrl_config_dump") int core_ctrl_config_dump(void)
{
    return (int)wro_controller_config_format(&g_ctrl_cfg, g_text, sizeof(g_text));
}

/* Crea el controlador con la configuracion actual (el estado de mision se
 * reinicia; la configuracion no). */
EXPORT("core_ctrl_init") void core_ctrl_init(void)
{
    wro_controller_init(&g_ctrl, &g_ctrl_cfg);
    camera_table_decoder_reset(&g_decoder);
}
/* Cambia la configuracion con la mision en marcha. */
EXPORT("core_ctrl_reconfigure") void core_ctrl_reconfigure(void)
{
    wro_controller_set_config(&g_ctrl, &g_ctrl_cfg);
}
EXPORT("core_ctrl_encoders") void core_ctrl_encoders(double t_us, float left_deg, float right_deg)
{
    wro_controller_encoders(&g_ctrl, (uint64_t)t_us, left_deg, right_deg);
}
EXPORT("core_ctrl_axle_encoder") void core_ctrl_axle_encoder(double t_us, float axle_deg)
{
    wro_controller_axle_encoder(&g_ctrl, (uint64_t)t_us, axle_deg);
}

EXPORT("core_ctrl_imu") void core_ctrl_imu(double t_us, float gyro_dps, float mag_deg)
{
    if (!isfinite(t_us) || t_us <= 0 || !isfinite(gyro_dps) ||
        (g_ctrl.last_imu_us && t_us <= (double)g_ctrl.last_imu_us)) return;
    wro_controller_imu(&g_ctrl, (uint64_t)t_us, gyro_dps, mag_deg);
}
EXPORT("core_ctrl_start") int core_ctrl_start(double t_us)
{
    return wro_controller_start(&g_ctrl, (uint64_t)t_us) ? 1 : 0;
}

/* Una trama del S3 llega por el enlace (core_wire_in_ptr). Devuelve 1 si el CRC
 * y el formato eran validos y la tabla se entrego al controlador. */
EXPORT("core_camera_feed") int core_camera_feed(int length, double arrival_us)
{
    camera_table_t table;
    if (length <= 0 || length > (int)sizeof(g_wire_in)) return 0;
    if (!camera_table_decoder_feed(&g_decoder, g_wire_in, (size_t)length, (uint64_t)arrival_us, &table))
        return 0;
    wro_controller_camera(&g_ctrl, &table);
    return 1;
}
EXPORT("core_camera_focal") void core_camera_focal(float focal_px)
{
    camera_table_decoder_set_focal(&g_decoder, focal_px);
}
EXPORT("core_camera_pitch") void core_camera_pitch(float pitch_deg)
{
    camera_table_decoder_set_pitch_deg(&g_decoder, pitch_deg);
}

static void export_debug(void)
{
    const local_planner_debug_t *d = &g_ctrl.debug;
    float *o = g_debug_out;
    o[0] = d->lane_cm; o[1] = d->reference_deg; o[2] = d->pass_cm; o[3] = d->cost;
    o[4] = (float)d->route_count; o[5] = (float)d->probe_count; o[6] = (float)d->curtain_cells;
    o[7] = (float)g_ctrl.map.colored_count;
    o[8] = (float)d->plan_count;
    float *p = o + 9;
    for (size_t i = 0; i < LOCAL_DEBUG_ROUTE_POINTS; ++i) { *p++ = d->route[i][0]; *p++ = d->route[i][1]; }
    for (size_t i = 0; i < LOCAL_DEBUG_ROUTE_POINTS; ++i) { *p++ = d->probe[i][0]; *p++ = d->probe[i][1]; }
    for (size_t i = 0; i < LOCAL_DEBUG_ROUTE_POINTS; ++i) { *p++ = d->plan[i][0]; *p++ = d->plan[i][1]; }
    for (size_t i = 0; i < LOCAL_MAP_MAX_CAMERA_OBJECTS; ++i) {
        const local_colored_object_t *c = &g_ctrl.map.colored[i];
        *p++ = i < g_ctrl.map.colored_count ? c->x_cm : 0.0f;
        *p++ = i < g_ctrl.map.colored_count ? c->forward_cm : 0.0f;
        *p++ = i < g_ctrl.map.colored_count ? (float)c->color : 0.0f;
        *p++ = i < g_ctrl.map.colored_count && c->confirmed ? 1.0f : 0.0f;
    }
    memcpy(g_map_cells, g_ctrl.map.cells, sizeof(g_map_cells));
}

static void export_command(void)
{
    const wro_command_t *c = &g_cmd;
    float *o = g_cmd_out;
    o[0] = (float)c->action; o[1] = c->steer_deg; o[2] = c->speed_cm_s; o[3] = (float)c->phase;
    o[4] = c->done ? 1.0f : 0.0f; o[5] = (float)c->corners; o[6] = (float)c->laps;
    o[7] = (float)c->direction; o[8] = c->reversed ? 1.0f : 0.0f; o[9] = c->yaw_deg;
    o[10] = c->course_deg; o[11] = c->front_cm; o[12] = c->left_cm; o[13] = c->right_cm;
    o[14] = c->lane_cm; o[15] = c->pass_cm; o[16] = c->elapsed_s; o[17] = (float)c->last_sign_color;
    o[18] = (float)g_ctrl.planner_state.escape_phase; o[19] = (float)g_ctrl.planner_state.escapes;
    o[20] = g_ctrl.planner_state.escaping ? 1.0f : 0.0f;
    o[21] = (float)g_ctrl.map.camera_hits; o[22] = (float)g_ctrl.map.camera_unmatched;
    o[23] = g_ctrl.map.camera_fresh ? 1.0f : 0.0f;
    o[24] = c->yaw_rate_dps; o[25] = c->explicit_yaw_rate ? 1.0f : 0.0f;
    o[26] = g_ctrl.debug.free_cm;
    o[27] = g_ctrl.park_pose.u_cm; o[28] = g_ctrl.park_pose.w_cm; o[29] = g_ctrl.park_pose.theta_deg;
    o[30] = (float)g_ctrl.park_lost; o[31] = (float)g_ctrl.park_prev;
    o[32] = (float)g_ctrl.sidestep_phase; o[33] = (float)g_ctrl.sidestep_side; o[34] = g_ctrl.pivoting ? 1.0f : 0.0f;
    o[35] = g_ctrl.lap_dbg[0]; o[36] = g_ctrl.lap_dbg[1]; o[37] = g_ctrl.lap_dbg[2]; o[38] = g_ctrl.lap_dbg[3]; o[39] = g_ctrl.lap_off_cm;
    o[40] = g_ctrl.odom.x_cm; o[41] = g_ctrl.odom.y_cm; o[42] = g_ctrl.odom.path_cm; o[43] = g_ctrl.odom.slip_cm;
    o[44] = g_ctrl.odom_in_use ? 1.0f : 0.0f; o[45] = g_ctrl.odom.v_cm_s; o[46] = g_ctrl.odom.heading_deg; o[47] = (float)g_ctrl.odom.slip_count;
    o[48] = g_ctrl.arena.pose.x_cm; o[49] = g_ctrl.arena.pose.z_cm; o[50] = g_ctrl.arena.pose.theta_deg;
    o[51] = g_ctrl.arena.valid ? 1.0f : 0.0f; o[52] = (float)g_ctrl.arena.inliers; o[53] = (float)g_ctrl.arena.used;
    o[54] = g_ctrl.arena.rms_cm; o[55] = (float)g_ctrl.arena.updates;
    o[56] = g_ctrl.race.valid ? 1.0f : 0.0f; o[57] = (float)g_ctrl.race_idx; o[58] = g_ctrl.race_active ? 1.0f : 0.0f;
    o[59] = g_ctrl.race_v_line; o[60] = g_ctrl.race_v_cmd; o[61] = g_ctrl.race_lat_cm; o[62] = g_ctrl.race_dist_cm;
    o[63] = g_ctrl.race_hold_s; o[64] = g_ctrl.race.time_s; o[65] = g_ctrl.race.time_center_s; o[66] = g_ctrl.race.length_cm;
    o[67] = g_ctrl.race.mix; o[68] = (float)g_ctrl.race.n; o[69] = g_ctrl.race_tried ? 1.0f : 0.0f;
    o[70] = (float)g_ctrl.race.signs; o[71] = (float)g_ctrl.planner_state.ref_rows;
    for (int k = 0; k < 8; ++k) o[72 + k] = g_ctrl.planner_state.ref_v_cm[3 * k];
    o[80] = g_ctrl.race_grip; o[81] = (float)g_ctrl.odom.slip_count;
    o[82] = g_ctrl.hug_active ? 1.0f : 0.0f; o[83] = g_ctrl.hug_cmd_cm; o[84] = g_ctrl.hug_target_cm; o[85] = g_ctrl.hug_hold_s;
    if (g_ctrl.race.valid) {
        g_race_out[0] = (float)g_ctrl.race.n;
        for (int i = 0; i < g_ctrl.race.n; ++i) {
            g_race_out[1 + 3 * i] = g_ctrl.race.x[i]; g_race_out[2 + 3 * i] = g_ctrl.race.z[i]; g_race_out[3 + 3 * i] = g_ctrl.race.v[i];
        }
    } else if (!g_ctrl.race.valid) g_race_out[0] = 0.0f;
    {
        float *sg = &g_race_out[1 + 3 * RACE_MAX_STATIONS];
        sg[0] = (float)g_ctrl.race.signs;
        for (int i = 0; i < g_ctrl.race.signs; ++i) { sg[1 + 3 * i] = g_ctrl.race.sign[i].x_cm; sg[2 + 3 * i] = g_ctrl.race.sign[i].z_cm; sg[3 + 3 * i] = (float)g_ctrl.race.sign[i].color; }
    }
}

/* Un barrido: core_points_ptr() tiene `count` puntos de 4 floats
 * (angulo, distancia, valido, ciego). */
EXPORT("core_ctrl_scan") void core_ctrl_scan(int count, double t_us)
{
    if (count > LOCAL_MAP_MAX_POINTS) count = LOCAL_MAP_MAX_POINTS;
    if (count < 0) count = 0;
    for (int i = 0; i < count; ++i) {
        const float *s = &g_points_in[i * 4];
        g_points[i] = (local_lidar_point_t){.angle_deg = s[0], .distance_cm = s[1],
                                            .valid = s[2] > 0.5f, .blind = s[3] > 0.5f};
    }
    wro_controller_scan(&g_ctrl, g_points, (size_t)count, (uint64_t)t_us, &g_cmd);
    export_command();
    export_debug();
}
/* Depuracion: hay `distance` cm libres para la huella siguiendo un arco de volante `steer`? */
EXPORT("core_arc_clear") int core_arc_clear(float steer_deg, float distance_cm)
{
    return local_planner_arc_clear(&g_ctrl.map, &g_ctrl.cfg.planner, steer_deg, distance_cm) ? 1 : 0;
}
EXPORT("core_ctrl_watchdog") int core_ctrl_watchdog(double now_us)
{
    const bool stopped = p4_runtime_guard(&g_ctrl, (uint64_t)now_us, &g_cmd);
    if (stopped) export_command();
    return stopped ? 1 : 0;
}

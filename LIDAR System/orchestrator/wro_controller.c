#include "wro_controller.h"

#include <math.h>
#include <string.h>

#include "config_fields.h"
#include "park_ackermann.h"

#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.29577951308232f
#define FAR_CM 1.0e9f

void wro_controller_config_default(wro_controller_config_t *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    local_planner_config_default(&c->planner);
    odom_config_default(&c->odom);
    arena_config_default(&c->arena);
    c->arena_enabled = true;
    c->arena_signs_enabled = true;
    c->arena_sign_bias_cm = 2.5f;
    c->arena_map_lead = true;
    c->arena_gates_corners = true;
    c->arena_corner_margin_cm = 10.0f;
    c->peek_enabled = false;         /* ayuda a planos de S extremas pero baja de 95 a 89 perfectas en pistas aleatorias */
    c->peek_bearing_min_deg = 26.0f;
    c->peek_max_heading_deg = 25.0f;
    c->peek_rate_dps = 14.0f;
    c->odometry_enabled = true;
    c->lane_width_cm = 100.0f;
    c->laps = 3;
    c->time_limit_s = 180.0f;
    c->corner_trigger_extra_cm = 0.0f;
    c->corner_min_travel_cm = 50.0f;
    c->corner_done_deg = 12.0f;
    c->open_side_cm = 150.0f;
    c->wall_align_gain = 0.25f;
    c->wall_align_max_deg = 20.0f;
    c->wall_align_heading_deg = 15.0f;
    c->corner_gyro_deg = 60.0f;
    c->corner_gyro_min_travel_cm = 110.0f;
    c->corner_abort_cm = 160.0f;
    c->sidestep_enabled = false;     /* en 100 pistas aleatorias estorba mas que ayuda; sigue disponible (tu plano de S extremas) */
    c->hug_enabled = false;
    c->hug_offset_cm = 20.0f;
    c->hug_detach_cm = 90.0f;
    c->hug_after_sign_cm = 20.0f;
    c->hug_corner_clear_cm = 0.0f;     /* 0: llega pegado hasta el giro (ahi esta el atajo: el robot empieza la recta siguiente ya avanzado) */
    c->hug_after_corner_cm = 10.0f;
    c->hug_rate = 0.6f;
    c->hug_unknown_hold_s = 1.5f;
    c->hug_stop_before_end = 2;
    c->race_enabled = false;         /* opcional: en 100 pistas con 7 letreros no gana tiempo (141 s contra 142 s) y baja de 99 a 97 perfectas */
    c->race_speed_enabled = true;
    c->race_lateral_enabled = false;   /* sube la ganancia de tiempo pero baja la fiabilidad (90/100 contra 97/100 en el banco): opcion */
    race_config_default(&c->race);
    c->race_after_corners = 4;
    c->race_stop_before_end = 2;
    c->race_lookahead_cm = 12.0f;
    c->race_gain_lap2 = 0.6f;
    c->race_max_dist_cm = 40.0f;
    c->race_speed_dist_cm = 12.0f;
    c->race_unknown_hold_s = 1.5f;
    c->race_corner_extra_cm = 20.0f;
    c->race_slip_derate = 0.25f;
    c->race_blend_before_cm = 50.0f;
    c->race_blend_after_cm = 40.0f;
    c->race_corner_sign_cm = 90.0f;
    c->race_corner_sign_after_cm = 70.0f;
    c->lap_memory_enabled = false;   /* experimental: no gana de forma clara en las pruebas aleatorias */
    c->sidestep_after_s = 2.0f;
    c->sidestep_cooldown_s = 4.0f;
    c->sidestep_speed_cm_s = 10.0f;
    c->sidestep_margin_cm = 2.5f;
    c->pivot_enabled = true;
    c->pivot_after_s = 2.0f;
    c->pivot_rot_dps = 45.0f;
    c->pivot_margin_cm = 1.0f;
    c->corner_max_heading_deg = 30.0f;
    c->stuck_relax_s = 0.0f;   /* desactivado: el juez termina la ronda ante un lado equivocado */
    c->motion_comp = true;
    c->latency_s = 0.10f;
    c->stuck_relax_cm = 80.0f;
    c->reverse_on_color = 1;   /* rojo: dar media vuelta */
    c->continue_on_color = 2;  /* verde: seguir */
    c->turnaround_enabled = false;   /* reglamento 2026: las tres vueltas van en el mismo sentido */
    c->finish_extra_cm = 35.0f;
    c->uturn_extra_cm = 30.0f;
    c->parking_enabled = false;
    c->start_in_lot = false;
    c->park_speed_cm_s = 8.0f;
    c->park_rot_dps = 45.0f;
    c->park_margin_cm = 1.5f;
    c->park_lot_length_cm = 0.0f;
    c->park_lane_w_cm = 32.0f;
    c->park_search_speed_cm_s = 12.0f;
    c->park_search_max_cm = 330.0f;
    c->start_lot_w_cm = 12.5f;
    c->park_demo_dir = 0;
    c->sensor_timeout_us = 300000u;
    c->gyro_min_samples = 50u;
}

static void sanitize(wro_controller_config_t *c)
{
    local_planner_config_sanitize(&c->planner);
    if (!isfinite(c->lane_width_cm) || c->lane_width_cm < 40.0f) c->lane_width_cm = 40.0f;
    if (c->lane_width_cm > 200.0f) c->lane_width_cm = 200.0f;
    if (c->laps < 1u) c->laps = 1u;
    if (c->laps > 6u) c->laps = 6u;
    if (!isfinite(c->corner_trigger_extra_cm)) c->corner_trigger_extra_cm = 0.0f;
    if (!(c->corner_min_travel_cm >= 0.0f)) c->corner_min_travel_cm = 0.0f;
    if (!(c->corner_done_deg > 1.0f)) c->corner_done_deg = 1.0f;
    if (!(c->open_side_cm > c->lane_width_cm)) c->open_side_cm = c->lane_width_cm * 1.5f;
    if (!(c->wall_align_gain >= 0.0f)) c->wall_align_gain = 0.0f;
    if (c->wall_align_gain > 1.0f) c->wall_align_gain = 1.0f;
    if (!(c->wall_align_heading_deg >= 0.0f)) c->wall_align_heading_deg = 0.0f;
    if (!(c->corner_gyro_deg >= 0.0f)) c->corner_gyro_deg = 0.0f;
    if (!(c->corner_gyro_min_travel_cm >= 0.0f)) c->corner_gyro_min_travel_cm = 0.0f;
    if (!(c->corner_abort_cm >= 60.0f)) c->corner_abort_cm = 60.0f;
    if (!(c->sidestep_after_s >= 0.5f)) c->sidestep_after_s = 0.5f;
    if (!(c->sidestep_cooldown_s >= 0.0f)) c->sidestep_cooldown_s = 0.0f;
    if (!(c->sidestep_speed_cm_s >= 4.0f)) c->sidestep_speed_cm_s = 4.0f;
    if (c->sidestep_speed_cm_s > 20.0f) c->sidestep_speed_cm_s = 20.0f;
    if (!(c->sidestep_margin_cm >= 0.0f)) c->sidestep_margin_cm = 0.0f;
    if (!(c->pivot_after_s >= 0.5f)) c->pivot_after_s = 0.5f;
    if (!(c->pivot_rot_dps >= 30.0f)) c->pivot_rot_dps = 30.0f;
    if (c->pivot_rot_dps > 120.0f) c->pivot_rot_dps = 120.0f;
    if (!(c->pivot_margin_cm >= 0.0f)) c->pivot_margin_cm = 0.0f;
    if (!(c->corner_max_heading_deg >= 5.0f)) c->corner_max_heading_deg = 5.0f;
    if (!(c->stuck_relax_s >= 0.0f)) c->stuck_relax_s = 0.0f;
    if (!(c->latency_s >= 0.0f)) c->latency_s = 0.0f;
    if (c->latency_s > 0.5f) c->latency_s = 0.5f;
    if (!(c->stuck_relax_cm >= 20.0f)) c->stuck_relax_cm = 20.0f;
    if (!(c->finish_extra_cm >= 0.0f)) c->finish_extra_cm = 0.0f;
    if (!(c->park_speed_cm_s >= 3.0f)) c->park_speed_cm_s = 3.0f;
    if (c->park_speed_cm_s > 20.0f) c->park_speed_cm_s = 20.0f;
    if (!(c->park_rot_dps >= 30.0f)) c->park_rot_dps = 30.0f;
    if (c->park_rot_dps > 120.0f) c->park_rot_dps = 120.0f;
    if (!(c->park_margin_cm >= 0.0f)) c->park_margin_cm = 0.0f;
    if (c->park_margin_cm > 3.0f) c->park_margin_cm = 3.0f;
    if (!(c->park_lot_length_cm >= 0.0f)) c->park_lot_length_cm = 0.0f;
    if (!(c->park_lane_w_cm >= 22.0f)) c->park_lane_w_cm = 22.0f;
    if (c->park_lane_w_cm > 45.0f) c->park_lane_w_cm = 45.0f;
    if (!(c->park_search_speed_cm_s >= 4.0f)) c->park_search_speed_cm_s = 4.0f;
    if (!(c->park_search_max_cm >= 50.0f)) c->park_search_max_cm = 50.0f;
    if (c->park_demo_dir > 1) c->park_demo_dir = 1;
    if (c->park_demo_dir < -1) c->park_demo_dir = -1;
    if (!(c->start_lot_w_cm >= 8.0f)) c->start_lot_w_cm = 8.0f;
    if (c->start_lot_w_cm > 16.0f) c->start_lot_w_cm = 16.0f;
    if (c->sensor_timeout_us < 50000u) c->sensor_timeout_us = 50000u;
    odom_config_sanitize(&c->odom);
    arena_config_sanitize(&c->arena);
    if (!(c->arena_sign_bias_cm >= 0.0f)) c->arena_sign_bias_cm = 0.0f;
    if (c->arena_sign_bias_cm > 6.0f) c->arena_sign_bias_cm = 6.0f;
    if (!(c->arena_corner_margin_cm >= 0.0f)) c->arena_corner_margin_cm = 0.0f;
    if (c->arena_corner_margin_cm > 40.0f) c->arena_corner_margin_cm = 40.0f;
    if (!(c->peek_bearing_min_deg >= 10.0f)) c->peek_bearing_min_deg = 10.0f;
    if (c->peek_bearing_min_deg > 60.0f) c->peek_bearing_min_deg = 60.0f;
    if (!(c->peek_max_heading_deg >= 0.0f)) c->peek_max_heading_deg = 0.0f;
    if (c->peek_max_heading_deg > 45.0f) c->peek_max_heading_deg = 45.0f;
    race_config_sanitize(&c->race);
    if (!(c->hug_offset_cm >= 0.0f)) c->hug_offset_cm = 0.0f;
    if (c->hug_offset_cm > 35.0f) c->hug_offset_cm = 35.0f;
    if (!(c->hug_detach_cm >= 20.0f)) c->hug_detach_cm = 20.0f;
    if (c->hug_detach_cm > 200.0f) c->hug_detach_cm = 200.0f;
    if (!(c->hug_after_sign_cm >= 0.0f)) c->hug_after_sign_cm = 0.0f;
    if (c->hug_after_sign_cm > 60.0f) c->hug_after_sign_cm = 60.0f;
    if (!(c->hug_corner_clear_cm >= 0.0f)) c->hug_corner_clear_cm = 0.0f;
    if (!(c->hug_after_corner_cm >= 0.0f)) c->hug_after_corner_cm = 0.0f;
    if (!(c->hug_rate >= 0.1f)) c->hug_rate = 0.1f;
    if (c->hug_rate > 1.5f) c->hug_rate = 1.5f;
    if (!(c->hug_unknown_hold_s >= 0.0f)) c->hug_unknown_hold_s = 0.0f;
    if (c->hug_stop_before_end > 8u) c->hug_stop_before_end = 8u;
    if (c->race_after_corners < 1u) c->race_after_corners = 1u;
    if (c->race_stop_before_end > 8u) c->race_stop_before_end = 8u;
    if (!(c->race_lookahead_cm >= 0.0f)) c->race_lookahead_cm = 0.0f;
    if (c->race_lookahead_cm > 60.0f) c->race_lookahead_cm = 60.0f;
    if (!(c->race_gain_lap2 >= 0.0f)) c->race_gain_lap2 = 0.0f;
    if (c->race_gain_lap2 > 1.0f) c->race_gain_lap2 = 1.0f;
    if (!(c->race_max_dist_cm >= 5.0f)) c->race_max_dist_cm = 5.0f;
    if (c->race_max_dist_cm > 80.0f) c->race_max_dist_cm = 80.0f;
    if (!(c->race_speed_dist_cm >= 2.0f)) c->race_speed_dist_cm = 2.0f;
    if (c->race_speed_dist_cm > c->race_max_dist_cm) c->race_speed_dist_cm = c->race_max_dist_cm;
    if (!(c->race_unknown_hold_s >= 0.0f)) c->race_unknown_hold_s = 0.0f;
    if (!(c->race_corner_sign_cm >= 0.0f)) c->race_corner_sign_cm = 0.0f;
    if (!(c->race_corner_sign_after_cm >= 0.0f)) c->race_corner_sign_after_cm = 0.0f;
    if (!(c->race_blend_before_cm >= 0.0f)) c->race_blend_before_cm = 0.0f;
    if (!(c->race_blend_after_cm >= 0.0f)) c->race_blend_after_cm = 0.0f;
    if (!(c->race_slip_derate >= 0.0f)) c->race_slip_derate = 0.0f;
    if (c->race_slip_derate > 1.0f) c->race_slip_derate = 1.0f;
    if (!(c->race_corner_extra_cm >= -20.0f)) c->race_corner_extra_cm = -20.0f;
    if (c->race_corner_extra_cm > 60.0f) c->race_corner_extra_cm = 60.0f;
    if (!(c->peek_rate_dps >= 1.0f)) c->peek_rate_dps = 1.0f;
    if (c->peek_rate_dps > 40.0f) c->peek_rate_dps = 40.0f;
}

/* ------------------------------------------------------- configuracion en texto */

#define PF(n, t, member) CF_FIELD("planner." n, t, wro_controller_config_t, planner.member)
#define CF(n, t, member) CF_FIELD(n, t, wro_controller_config_t, member)
static const cf_field_t FIELDS[] = {
    PF("chassis_width_cm", CF_F32, chassis_width_cm), PF("chassis_length_cm", CF_F32, chassis_length_cm),
    PF("wheelbase_cm", CF_F32, wheelbase_cm), PF("chassis_rear_cm", CF_F32, chassis_rear_cm), PF("footprint_shrink_cm", CF_F32, footprint_shrink_cm), PF("squeeze_cm", CF_F32, squeeze_cm), PF("tight_clear_cm", CF_F32, tight_clear_cm), PF("steer_nudge_deg", CF_F32, steer_nudge_deg),
    PF("steer_soft_deg", CF_F32, steer_soft_deg),
    PF("steer_hard_deg", CF_F32, steer_hard_deg), PF("lidar_forward_cm", CF_F32, lidar_forward_cm),
    PF("camera_forward_cm", CF_F32, camera_forward_cm), PF("lidar_min_cm", CF_F32, lidar_min_cm),
    PF("speed_cm_s", CF_F32, speed_cm_s), PF("maneuver_speed_cm_s", CF_F32, maneuver_speed_cm_s),
    PF("cycle_s", CF_F32, cycle_s), PF("scan_period_s", CF_F32, scan_period_s),
    PF("sync_max_age_us", CF_U32, sync_max_age_us),
    PF("w_clearance", CF_F32, w_clearance), PF("w_lateral", CF_F32, w_lateral),
    PF("w_heading", CF_F32, w_heading), PF("w_turn", CF_F32, w_turn), PF("w_progress", CF_F32, w_progress),
    PF("w_dead_end", CF_F32, w_dead_end), PF("w_change", CF_F32, w_change), PF("w_camera", CF_F32, w_camera),
    PF("w_pass_shortfall", CF_F32, w_pass_shortfall), PF("switch_tie", CF_F32, switch_tie),
    PF("arc_cm", CF_F32, arc_cm), PF("discount", CF_F32, discount),
    PF("clearance_pref_cm", CF_F32, clearance_pref_cm), PF("lateral_scale_cm", CF_F32, lateral_scale_cm),
    PF("min_commit_cm", CF_F32, min_commit_cm), PF("step_guard_cm", CF_F32, step_guard_cm),
    PF("plan_enabled", CF_BOOL, plan_enabled), PF("plan_horizon_cm", CF_F32, plan_horizon_cm),
    PF("plan_margin_cm", CF_F32, plan_margin_cm), PF("plan_margin_extra_cm", CF_F32, plan_margin_extra_cm), PF("plan_lookahead_cm", CF_F32, plan_lookahead_cm),
    PF("plan_slack", CF_F32, plan_slack), PF("plan_w_lateral", CF_F32, plan_w_lateral),
    PF("plan_w_clearance", CF_F32, plan_w_clearance), PF("plan_w_slew", CF_F32, plan_w_slew), PF("plan_w_path", CF_F32, plan_w_path),
    PF("plan_path_scale_cm", CF_F32, plan_path_scale_cm), PF("plan_w_race", CF_F32, plan_w_race), PF("plan_race_clear_cm", CF_F32, plan_race_clear_cm), PF("plan_refine", CF_BOOL, plan_refine),
    PF("track_lookahead_cm", CF_F32, track_lookahead_cm), PF("track_deadband_deg", CF_F32, track_deadband_deg),
    PF("track_slack", CF_F32, track_slack), PF("pass_probe_cm", CF_F32, pass_probe_cm), PF("min_pass_cm", CF_F32, min_pass_cm),
    PF("lane_max_span_cm", CF_F32, lane_max_span_cm), PF("lane_spread_cm", CF_F32, lane_spread_cm),
    PF("recenter_lane_cm", CF_F32, recenter_lane_cm), PF("recenter_hard_deg", CF_F32, recenter_hard_deg),
    PF("recenter_hard_cm", CF_F32, recenter_hard_cm), PF("recenter_slack", CF_F32, recenter_slack),
    PF("recenter_slack_heading", CF_F32, recenter_slack_heading),
    PF("recenter_deg_to_cm", CF_F32, recenter_deg_to_cm), PF("recenter_clear_cm", CF_F32, recenter_clear_cm),
    PF("escape_room_cm", CF_F32, escape_room_cm), PF("escape_margin_cm", CF_F32, escape_margin_cm),
    PF("escape_turn_deg", CF_F32, escape_turn_deg), PF("escape_probe_pad_cm", CF_F32, escape_probe_pad_cm),
    PF("escape_back_max_cm", CF_F32, escape_back_max_cm), PF("escape_min_back_cm", CF_F32, escape_min_back_cm), PF("escape_retry_s", CF_F32, escape_retry_s), PF("escape_max_attempts", CF_U8, escape_max_attempts),
    PF("bridge_max_gap_cm", CF_F32, bridge_max_gap_cm), PF("bridge_max_jump_cm", CF_F32, bridge_max_jump_cm),
    PF("deskew", CF_BOOL, deskew),
    PF("pass_side_red", CF_I8, pass_side_red), PF("pass_side_green", CF_I8, pass_side_green),
    PF("enforce_color_side", CF_BOOL, enforce_color_side), PF("curtain_len_cm", CF_F32, curtain_len_cm),
    PF("curtain_min_forward_cm", CF_F32, curtain_min_forward_cm),
    PF("assoc_bearing_tol_deg", CF_F32, assoc_bearing_tol_deg), PF("assoc_range_rel", CF_F32, assoc_range_rel),
    PF("assoc_range_abs_cm", CF_F32, assoc_range_abs_cm), PF("cluster_radius_cm", CF_F32, cluster_radius_cm),
    PF("camera_ghost_fallback", CF_BOOL, camera_ghost_fallback),
    PF("curtain_unconfirmed", CF_BOOL, curtain_unconfirmed),
    CF("lane_width_cm", CF_F32, lane_width_cm), CF("laps", CF_U8, laps), CF("time_limit_s", CF_F32, time_limit_s),
    CF("corner_trigger_extra_cm", CF_F32, corner_trigger_extra_cm),
    CF("corner_min_travel_cm", CF_F32, corner_min_travel_cm), CF("corner_done_deg", CF_F32, corner_done_deg),
    CF("open_side_cm", CF_F32, open_side_cm), CF("wall_align_gain", CF_F32, wall_align_gain),
    CF("wall_align_max_deg", CF_F32, wall_align_max_deg),
    CF("wall_align_heading_deg", CF_F32, wall_align_heading_deg), CF("corner_gyro_deg", CF_F32, corner_gyro_deg),
    CF("corner_gyro_min_travel_cm", CF_F32, corner_gyro_min_travel_cm), CF("corner_abort_cm", CF_F32, corner_abort_cm),
    CF("arena_gates_corners", CF_BOOL, arena_gates_corners), CF("arena_corner_margin_cm", CF_F32, arena_corner_margin_cm),
    CF("peek_enabled", CF_BOOL, peek_enabled), CF("peek_bearing_min_deg", CF_F32, peek_bearing_min_deg),
    CF("peek_max_heading_deg", CF_F32, peek_max_heading_deg), CF("peek_rate_dps", CF_F32, peek_rate_dps),
    CF("arena_enabled", CF_BOOL, arena_enabled), CF("arena_sign_bias_cm", CF_F32, arena_sign_bias_cm), CF("arena_map_lead", CF_BOOL, arena_map_lead), CF("arena_signs_enabled", CF_BOOL, arena_signs_enabled),
    CF("arena.outer_half_cm", CF_F32, arena.outer_half_cm), CF("arena.inner_half_cm", CF_F32, arena.inner_half_cm),
    CF("arena.tol_cm", CF_F32, arena.tol_cm), CF("arena.gain", CF_F32, arena.gain),
    CF("arena.max_range_cm", CF_F32, arena.max_range_cm),
    CF("odometry_enabled", CF_BOOL, odometry_enabled),
    CF("odom.wheel_diameter_cm", CF_F32, odom.wheel_diameter_cm), CF("odom.track_width_cm", CF_F32, odom.track_width_cm),
    CF("odom.left_scale", CF_F32, odom.left_scale), CF("odom.right_scale", CF_F32, odom.right_scale),
    CF("odom.left_sign", CF_I8, odom.left_sign), CF("odom.right_sign", CF_I8, odom.right_sign),
    CF("odom.slip_yaw_dps", CF_F32, odom.slip_yaw_dps), CF("odom.max_speed_cm_s", CF_F32, odom.max_speed_cm_s),
    CF("hug_enabled", CF_BOOL, hug_enabled), CF("hug_offset_cm", CF_F32, hug_offset_cm), CF("hug_detach_cm", CF_F32, hug_detach_cm),
    CF("hug_after_sign_cm", CF_F32, hug_after_sign_cm), CF("hug_corner_clear_cm", CF_F32, hug_corner_clear_cm),
    CF("hug_after_corner_cm", CF_F32, hug_after_corner_cm), CF("hug_rate", CF_F32, hug_rate),
    CF("hug_unknown_hold_s", CF_F32, hug_unknown_hold_s), CF("hug_stop_before_end", CF_U8, hug_stop_before_end),
    CF("race_enabled", CF_BOOL, race_enabled), CF("race_speed_enabled", CF_BOOL, race_speed_enabled),
    CF("race_lateral_enabled", CF_BOOL, race_lateral_enabled), CF("race_after_corners", CF_U8, race_after_corners), CF("race_stop_before_end", CF_U8, race_stop_before_end),
    CF("race_lookahead_cm", CF_F32, race_lookahead_cm), CF("race_gain_lap2", CF_F32, race_gain_lap2),
    CF("race_max_dist_cm", CF_F32, race_max_dist_cm), CF("race_speed_dist_cm", CF_F32, race_speed_dist_cm), CF("race_unknown_hold_s", CF_F32, race_unknown_hold_s),
    CF("race_corner_extra_cm", CF_F32, race_corner_extra_cm), CF("race_slip_derate", CF_F32, race_slip_derate), CF("race_corner_sign_cm", CF_F32, race_corner_sign_cm), CF("race_corner_sign_after_cm", CF_F32, race_corner_sign_after_cm),
    CF("race.v_max_cm_s", CF_F32, race.v_max_cm_s), CF("race.v_sign_cm_s", CF_F32, race.v_sign_cm_s), CF("race.v_corner_cm_s", CF_F32, race.v_corner_cm_s), CF("race.a_lat_cm_s2", CF_F32, race.a_lat_cm_s2),
    CF("race.a_acc_cm_s2", CF_F32, race.a_acc_cm_s2), CF("race.a_dec_cm_s2", CF_F32, race.a_dec_cm_s2),
    CF("race.wall_margin_cm", CF_F32, race.wall_margin_cm), CF("race.sign_margin_cm", CF_F32, race.sign_margin_cm),
    CF("race.approach_cm", CF_F32, race.approach_cm), CF("race.sign_before_cm", CF_F32, race.sign_before_cm),
    CF("race.sign_after_cm", CF_F32, race.sign_after_cm), CF("race.corner_before_cm", CF_F32, race.corner_before_cm),
    CF("race.corner_after_cm", CF_F32, race.corner_after_cm), CF("race.corner_radius_cm", CF_F32, race.corner_radius_cm), CF("race.max_offset_cm", CF_F32, race.max_offset_cm),
    CF("race_blend_before_cm", CF_F32, race_blend_before_cm), CF("race_blend_after_cm", CF_F32, race_blend_after_cm),
    CF("race.station_cm", CF_F32, race.station_cm), CF("race.iterations", CF_INT, race.iterations),
    CF("lap_memory_enabled", CF_BOOL, lap_memory_enabled), CF("sidestep_enabled", CF_BOOL, sidestep_enabled), CF("sidestep_after_s", CF_F32, sidestep_after_s),
    CF("sidestep_cooldown_s", CF_F32, sidestep_cooldown_s), CF("sidestep_speed_cm_s", CF_F32, sidestep_speed_cm_s),
    CF("sidestep_margin_cm", CF_F32, sidestep_margin_cm),
    CF("pivot_enabled", CF_BOOL, pivot_enabled), CF("pivot_after_s", CF_F32, pivot_after_s),
    CF("pivot_rot_dps", CF_F32, pivot_rot_dps), CF("pivot_margin_cm", CF_F32, pivot_margin_cm), CF("corner_max_heading_deg", CF_F32, corner_max_heading_deg),
    CF("motion_comp", CF_BOOL, motion_comp), CF("latency_s", CF_F32, latency_s),
    CF("parking_enabled", CF_BOOL, parking_enabled), CF("park_ackermann", CF_BOOL, park_ackermann), CF("park_exact_moves", CF_BOOL, park_exact_moves), CF("start_in_lot", CF_BOOL, start_in_lot),
    CF("park_speed_cm_s", CF_F32, park_speed_cm_s), CF("park_rot_dps", CF_F32, park_rot_dps),
    CF("park_margin_cm", CF_F32, park_margin_cm), CF("park_lot_length_cm", CF_F32, park_lot_length_cm),
    CF("park_lane_w_cm", CF_F32, park_lane_w_cm), CF("park_search_speed_cm_s", CF_F32, park_search_speed_cm_s),
    CF("park_search_max_cm", CF_F32, park_search_max_cm), CF("start_lot_w_cm", CF_F32, start_lot_w_cm), CF("park_demo_dir", CF_I8, park_demo_dir),
    CF("stuck_relax_s", CF_F32, stuck_relax_s), CF("stuck_relax_cm", CF_F32, stuck_relax_cm),
    CF("reverse_on_color", CF_U8, reverse_on_color), CF("continue_on_color", CF_U8, continue_on_color),
    CF("turnaround_enabled", CF_BOOL, turnaround_enabled), CF("finish_extra_cm", CF_F32, finish_extra_cm),
    CF("uturn_extra_cm", CF_F32, uturn_extra_cm),
    CF("sensor_timeout_us", CF_U32, sensor_timeout_us), CF("gyro_min_samples", CF_U32, gyro_min_samples),
};
#define FIELD_COUNT (sizeof(FIELDS) / sizeof(FIELDS[0]))

bool wro_controller_config_set(wro_controller_config_t *cfg, const char *key, const char *value)
{
    return cf_set(cfg, FIELDS, FIELD_COUNT, key, value);
}

size_t wro_controller_config_format(const wro_controller_config_t *cfg, char *out, size_t capacity)
{
    return cf_format(cfg, FIELDS, FIELD_COUNT, out, capacity);
}

bool wro_controller_config_parse_text(wro_controller_config_t *cfg, const char *text,
                                      char *error, size_t error_size)
{
    static wro_controller_config_t work;
    if (!cfg || !cf_parse_text(cfg, &work, sizeof(work), FIELDS, FIELD_COUNT, text, error, error_size))
        return false;
    sanitize(cfg);
    return true;
}

void wro_controller_init(wro_controller_t *c, const wro_controller_config_t *cfg)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    if (cfg) c->cfg = *cfg; else wro_controller_config_default(&c->cfg);
    sanitize(&c->cfg);
    imu_yaw_init(&c->yaw);
    imu_yaw_begin_calibration(&c->yaw);
    odom_init(&c->odom, &c->cfg.odom);
    arena_init(&c->arena, &c->cfg.arena);
    local_planner_state_reset(&c->planner_state);
    c->planner_state.debug = &c->debug;
    c->phase = WRO_PHASE_IDLE;
    c->race_idx = -1;
}

void wro_controller_set_config(wro_controller_t *c, const wro_controller_config_t *cfg)
{
    if (!c || !cfg) return;
    c->cfg = *cfg;
    sanitize(&c->cfg);
    c->odom.cfg = c->cfg.odom;
    c->arena.cfg = c->cfg.arena;
}

/* Encoders de las ruedas (grados acumulados). Se integran al llegar; el escaneo toma lo acumulado. */
void wro_controller_encoders(wro_controller_t *c, uint64_t t_us, float left_deg, float right_deg)
{
    if (!c || !c->cfg.odometry_enabled) return;
    const float ds = odom_update(&c->odom, t_us, left_deg, right_deg, c->yaw.yaw_deg, c->yaw.has_last);
    c->odom_accum_cm += ds;
    c->odom_last_us = t_us;
    c->odom_have = true;
}

void wro_controller_axle_encoder(wro_controller_t *c, uint64_t t_us, float axle_deg)
{
    if (!c || !c->cfg.odometry_enabled) return;
    c->odom_accum_cm += odom_update_axle(&c->odom, t_us, axle_deg, c->yaw.yaw_deg, c->yaw.has_last);
    c->odom_last_us = t_us;
    c->odom_have = true;
}

void wro_controller_imu(wro_controller_t *c, uint64_t t_us, float gyro_dps, float mag_heading_deg)
{
    if (!c) return;
    imu_yaw_update(&c->yaw, t_us, gyro_dps, mag_heading_deg);
    c->last_imu_us = t_us;
}

void wro_controller_camera(wro_controller_t *c, const camera_table_t *table)
{
    if (!c || !table) return;
    c->camera = *table;
    c->camera_seen = true;
}

bool wro_controller_start(wro_controller_t *c, uint64_t t_us)
{
    if (!c || c->phase != WRO_PHASE_IDLE) return false;
    const bool calibrated = imu_yaw_end_calibration(&c->yaw, c->cfg.gyro_min_samples);
    c->phase = c->cfg.start_in_lot ? WRO_PHASE_PARK_EXIT : WRO_PHASE_RUN;
    if (!c->cfg.start_in_lot && c->cfg.parking_enabled && c->cfg.park_demo_dir != 0) {
        /* Demostracion del parqueo: ya en el carril, con las vueltas hechas. */
        c->phase = WRO_PHASE_PARK_SEARCH;
        c->direction = c->cfg.park_demo_dir;
        c->corners_done = (int)c->cfg.laps * 4;
        c->park_search_cm = 0.0f;
    }
    c->start_us = t_us;
    c->odom_accum_cm = 0.0f;
    c->aligned = false;
    c->travel_cm = FAR_CM;   /* la primera esquina puede estar ya cerca */
    return calibrated;
}

/* ------------------------------------------------------------- percepcion */

typedef struct {
    float front_cm;          /* muro frontal, desde el sensor; FAR_CM si no hay */
    bool front_wall;
    float left_cm, right_cm; /* distancia lateral mediana; FAR_CM si no hay retornos */
    int left_n, right_n;
    bool align_valid;
    float align_deg;         /* direccion de las paredes respecto al morro, derecha positiva */
} perception_t;

static float median_of(float *v, int n)
{
    for (int i = 1; i < n; ++i) {
        const float x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; --j; }
        v[j + 1] = x;
    }
    return v[n / 2];
}

/* Ajusta x = a + b f a las paredes de un lado y devuelve la pendiente b. Un
 * pilar entre el robot y la pared son unos pocos puntos fuera de linea: se
 * descartan en una segunda pasada. `delta_deg` es el rumbo del carril respecto
 * al morro: las ventanas angulares giran con el, para que un robot torcido siga
 * mirando a SU pared y no a la esquina. */
static bool fit_wall(const local_lidar_point_t *p, size_t n, int side, float lane_cm,
                     float delta_deg, float *slope)
{
    static float xs[LOCAL_MAP_MAX_POINTS], fs[LOCAL_MAP_MAX_POINTS];
    int count = 0;
    for (size_t i = 0; i < n && count < LOCAL_MAP_MAX_POINTS; ++i) {
        if (!p[i].valid || !(p[i].distance_cm > 0.0f)) continue;
        const float a = (p[i].angle_deg - delta_deg) * (float)side;
        if (a < 45.0f || a > 135.0f) continue;
        const float r = p[i].angle_deg * DEG_TO_RAD;
        const float x = p[i].distance_cm * sinf(r), f = p[i].distance_cm * cosf(r);
        if (fabsf(x) < 8.0f || fabsf(x) > lane_cm * 0.85f) continue;
        xs[count] = x; fs[count] = f; ++count;
    }
    if (count < 10) return false;
    for (int pass = 0; pass < 2; ++pass) {
        double sf = 0, sx = 0, sff = 0, sfx = 0;
        for (int i = 0; i < count; ++i) { sf += fs[i]; sx += xs[i]; sff += (double)fs[i] * fs[i]; sfx += (double)fs[i] * xs[i]; }
        const double det = count * sff - sf * sf;
        if (fabs(det) < 1.0e-6) return false;
        const double b = (count * sfx - sf * sx) / det, a0 = (sx - b * sf) / count;
        if (pass == 0) {
            int kept = 0;
            for (int i = 0; i < count; ++i) {
                if (fabs(xs[i] - (a0 + b * fs[i])) > 3.0) continue;
                xs[kept] = xs[i]; fs[kept] = fs[i]; ++kept;
            }
            count = kept;
            if (count < 8) return false;
        } else {
            float lo = fs[0], hi = fs[0];
            for (int i = 1; i < count; ++i) { if (fs[i] < lo) lo = fs[i]; if (fs[i] > hi) hi = fs[i]; }
            if (hi - lo < 40.0f) return false;
            *slope = (float)b;
            return true;
        }
    }
    return false;
}

/* `delta_deg`: direccion del carril respecto al morro (derecha positiva), la
 * que el controlador cree. Todo se mide en el sistema del CARRIL: el muro frontal
 * es un plano perpendicular a el, y los muros laterales quedan a +-90 grados. Con
 * el robot torcido 12 grados, medirlo en el sistema del robot hacia que el muro
 * frontal dejara de parecer un plano justo al acercarse a la esquina. */
static void analyze(const local_lidar_point_t *p, size_t n, float lane_cm, float delta_deg,
                    perception_t *out)
{
    static float front[LOCAL_MAP_MAX_POINTS], planar[LOCAL_MAP_MAX_POINTS];
    static float left[LOCAL_MAP_MAX_POINTS], right[LOCAL_MAP_MAX_POINTS];
    int nf = 0, np = 0, nl = 0, nr = 0;
    const float sd = sinf(delta_deg * DEG_TO_RAD), cd = cosf(delta_deg * DEG_TO_RAD);
    for (size_t i = 0; i < n; ++i) {
        if (!p[i].valid || !(p[i].distance_cm > 0.0f)) continue;
        const float d = p[i].distance_cm, r = p[i].angle_deg * DEG_TO_RAD;
        const float a = imu_wrap_deg(p[i].angle_deg - delta_deg);  /* respecto al carril */
        const float x = d * sinf(r), f = d * cosf(r);
        const float along = x * sd + f * cd, across = x * cd - f * sd;
        if (fabsf(a) <= 15.0f && nf < LOCAL_MAP_MAX_POINTS) front[nf++] = d;
        if (fabsf(a) <= 45.0f && fabsf(across) <= 40.0f && np < LOCAL_MAP_MAX_POINTS) planar[np++] = along;
        if (a >= -105.0f && a <= -75.0f && nl < LOCAL_MAP_MAX_POINTS) left[nl++] = fabsf(across);
        if (a >= 75.0f && a <= 105.0f && nr < LOCAL_MAP_MAX_POINTS) right[nr++] = fabsf(across);
    }
    memset(out, 0, sizeof(*out));
    out->front_cm = FAR_CM;
    out->left_cm = FAR_CM;
    out->right_cm = FAR_CM;
    out->left_n = nl;
    out->right_n = nr;
    if (nf >= 3) (void)median_of(front, nf);
    /* El muro frontal es un PLANO: la mayoria de los puntos en +-45 grados caen
     * a la misma distancia hacia delante. Un pilar delante (16 % de ese
     * abanico) no engana a la mediana, y un grupo de pilares no es un muro. */
    if (np >= 10) {
        const float f_med = median_of(planar, np);
        int close = 0;
        for (int i = 0; i < np; ++i) if (fabsf(planar[i] - f_med) <= 5.0f) ++close;
        if ((float)close >= 0.6f * (float)np) { out->front_wall = true; out->front_cm = f_med; }
    }
    if (nl >= 4) out->left_cm = median_of(left, nl);
    if (nr >= 4) out->right_cm = median_of(right, nr);
    float bl = 0.0f, br = 0.0f;
    const bool okl = fit_wall(p, n, -1, lane_cm, delta_deg, &bl);
    const bool okr = fit_wall(p, n, 1, lane_cm, delta_deg, &br);
    if (okl && okr) {
        if (fabsf(bl - br) < 0.12f) { out->align_valid = true; out->align_deg = atanf(0.5f * (bl + br)) * RAD_TO_DEG; }
    } else if (okl || okr) {
        out->align_valid = true;
        out->align_deg = atanf(okl ? bl : br) * RAD_TO_DEG;
    }
}

/* ---------------------------------------------------------------- ordenes */

static void command_from_action(const wro_controller_t *c, uint8_t action, wro_command_t *out)
{
    const local_planner_config_t *p = &c->cfg.planner;
    out->action = action;
    out->steer_deg = 0.0f;
    out->speed_cm_s = p->speed_cm_s;
    /* Los giros de la retícula llevan su volante continuo (el seguimiento fino pide
     * curvaturas entre las discretas). */
    if ((action >= LOCAL_ACTION_FORWARD && action <= LOCAL_ACTION_LEFT_HARD) ||
        action == LOCAL_ACTION_NUDGE_RIGHT || action == LOCAL_ACTION_NUDGE_LEFT) {
        out->steer_deg = c->planner_state.steer_deg;
    } else
    switch (action) {
    case LOCAL_ACTION_NUDGE_RIGHT: out->steer_deg = p->steer_nudge_deg; break;
    case LOCAL_ACTION_NUDGE_LEFT: out->steer_deg = -p->steer_nudge_deg; break;
    case LOCAL_ACTION_RIGHT_SOFT: out->steer_deg = p->steer_soft_deg; break;
    case LOCAL_ACTION_RIGHT_HARD: out->steer_deg = p->steer_hard_deg; break;
    case LOCAL_ACTION_LEFT_SOFT: out->steer_deg = -p->steer_soft_deg; break;
    case LOCAL_ACTION_LEFT_HARD: out->steer_deg = -p->steer_hard_deg; break;
    case LOCAL_ACTION_REVERSE: out->speed_cm_s = -p->maneuver_speed_cm_s; break;
    case LOCAL_ACTION_FORWARD: break;
    default: out->speed_cm_s = 0.0f; break;
    }
    /* Con la ruta libre cerca de su fin se baja la velocidad: la siguiente
     * lectura llega 100 ms despues y el chasis recorre 2 cm en cada una. */
    /* Pasando rozando un pilar la velocidad manda: a 20 cm/s el chasis sigue 1-2 cm despues de ordenar parar. */
    if (out->speed_cm_s > p->maneuver_speed_cm_s && p->tight_clear_cm > 0.0f && c->debug.pass_cm >= 0.0f &&
        c->debug.min_clear_cm < p->tight_clear_cm)
        out->speed_cm_s = p->maneuver_speed_cm_s;
    if (out->speed_cm_s > p->maneuver_speed_cm_s && c->debug.pass_cm >= 0.0f &&
        c->debug.pass_cm < 40.0f)
        out->speed_cm_s = p->maneuver_speed_cm_s + 0.5f * (p->speed_cm_s - p->maneuver_speed_cm_s);
}

static void fill_status(const wro_controller_t *c, const perception_t *pc, wro_command_t *out)
{
    out->phase = c->phase;
    out->done = c->phase == WRO_PHASE_DONE;
    out->corners = c->corners_done;
    out->laps = c->corners_done / 4;
    out->direction = c->direction;
    out->reversed = c->reversed;
    out->yaw_deg = c->yaw.yaw_deg;
    out->course_deg = c->course_deg;
    out->front_cm = pc ? pc->front_cm : FAR_CM;
    out->left_cm = pc ? pc->left_cm : FAR_CM;
    out->right_cm = pc ? pc->right_cm : FAR_CM;
    out->lane_cm = c->debug.lane_cm;
    out->pass_cm = c->debug.pass_cm;
    out->last_sign_color = c->last_sign_color;
    out->relaxed = c->relaxed;
    /* Velocidad angular equivalente del (volante, velocidad) del planificador: es lo que
     * necesita un robot diferencial (rueda izquierda/derecha); un robot con servo usa el volante. */
    if (!out->explicit_yaw_rate)
        out->yaw_rate_dps = out->speed_cm_s * tanf(out->steer_deg * DEG_TO_RAD) /
                            c->cfg.planner.wheelbase_cm * RAD_TO_DEG;
    out->elapsed_s = c->start_us && c->last_scan_us > c->start_us
        ? (float)(c->last_scan_us - c->start_us) * 1.0e-6f : 0.0f;
}

static void stop_command(const wro_controller_t *c, wro_command_t *out)
{
    out->action = LOCAL_ACTION_STOP;
    out->steer_deg = 0.0f;
    out->speed_cm_s = 0.0f;
    (void)c;
}

/* Media vuelta con marcha atras si hace falta (K-turn): el carril de 100 cm deja
 * sitio al semicirculo de 2 x 21 cm, pero un pilar o una pared pueden cortarlo.
 * Gira hacia delante mientras el arco restante este libre; si no, retrocede con
 * el volante cruzado (marcha atras + volante contrario = mismo sentido de giro)
 * y vuelve a intentarlo. */
static bool uturn_step(wro_controller_t *c, float dt, wro_command_t *out)
{
    const local_planner_config_t *p = &c->cfg.planner;
    float err = imu_wrap_deg(c->uturn_target_deg - c->yaw.yaw_deg);
    if (fabsf(err) <= 10.0f) return true;
    const int rot = fabsf(err) > 100.0f ? c->uturn_dir : (err > 0.0f ? 1 : -1);
    const float hard = p->steer_hard_deg;
    out->action = LOCAL_ACTION_STOP;
    out->steer_deg = 0.0f;
    out->speed_cm_s = 0.0f;
    if (!c->uturn_reversing) {
        const float k = tanf(hard * DEG_TO_RAD) / p->wheelbase_cm;
        const float distance = fabsf(err) * DEG_TO_RAD / k + p->escape_probe_pad_cm;
        if (local_planner_arc_clear(&c->map, p, (float)rot * hard, distance)) {
            out->action = rot > 0 ? LOCAL_ACTION_RIGHT_HARD : LOCAL_ACTION_LEFT_HARD;
            out->steer_deg = (float)rot * hard;
            out->speed_cm_s = p->maneuver_speed_cm_s;
            return false;
        }
        if (c->uturn_legs < 8 && local_planner_rear_clear(&c->map, p)) {
            c->uturn_reversing = true;
            c->uturn_leg_cm = 0.0f;
        } else {
            return false; /* sin sitio: se queda parado y el vigilante lo anota */
        }
    }
    if (c->uturn_reversing) {
        c->uturn_leg_cm += p->maneuver_speed_cm_s * dt;
        if (c->uturn_leg_cm >= 25.0f || !local_planner_rear_clear(&c->map, p)) {
            c->uturn_reversing = false;
            ++c->uturn_legs;
            return false;
        }
        out->action = LOCAL_ACTION_REVERSE;
        out->steer_deg = -(float)rot * hard;
        out->speed_cm_s = -p->maneuver_speed_cm_s;
    }
    return false;
}

/* Letreros seguidos entre escaneos. Cada vuelta del LiDAR se transforma lo que se
 * recuerda con el giro del giroscopio y el avance (velocidad ordenada x tiempo), se
 * funde con los letreros confirmados de este escaneo y se anota el ultimo que el
 * robot deja atras: su eje trasero pasa de delante a detras del letrero. */
static void track_signs(wro_controller_t *c, float dt)
{
    (void)dt;
    const float yaw = c->yaw.yaw_deg;
    float dpsi = c->prev_yaw_valid ? imu_wrap_deg(yaw - c->prev_yaw_deg) * DEG_TO_RAD : 0.0f;
    c->prev_yaw_deg = yaw;
    c->prev_yaw_valid = true;
    const float ds = c->scan_ds_cm;
    const float cs = cosf(dpsi), sn = sinf(dpsi);
    const float mx = ds * sinf(0.5f * dpsi), mf = ds * cosf(0.5f * dpsi);
    const int n = (int)(sizeof(c->signs) / sizeof(c->signs[0]));
    for (int i = 0; i < n; ++i) {
        if (!c->signs[i].valid) continue;
        const float x = c->signs[i].x_cm, f = c->signs[i].forward_cm;
        c->signs[i].stale_cm += fabsf(ds);
        c->signs[i].x_cm = x * cs - f * sn - mx;
        c->signs[i].forward_cm = x * sn + f * cs - mf;
    }
    for (size_t k = 0; k < c->map.colored_count; ++k) {
        const local_colored_object_t *o = &c->map.colored[k];
        if (!o->confirmed || (o->color != 1 && o->color != 2)) continue;
        int match = -1, free_slot = -1;
        float best = 10.0f;
        for (int i = 0; i < n; ++i) {
            if (!c->signs[i].valid) { if (free_slot < 0) free_slot = i; continue; }
            const float d = hypotf(c->signs[i].x_cm - o->x_cm, c->signs[i].forward_cm - o->forward_cm);
            if (d < best) { best = d; match = i; }
        }
        if (match >= 0) {
            c->signs[match].x_cm = 0.5f * (c->signs[match].x_cm + o->x_cm);
            c->signs[match].forward_cm = 0.5f * (c->signs[match].forward_cm + o->forward_cm);
            c->signs[match].color = o->color;
            c->signs[match].stale_cm = 0.0f;
        } else if (free_slot >= 0) {
            c->signs[free_slot].x_cm = o->x_cm;
            c->signs[free_slot].forward_cm = o->forward_cm;
            c->signs[free_slot].color = o->color;
            c->signs[free_slot].stale_cm = 0.0f;
            c->signs[free_slot].valid = true;
        }
    }
    for (int i = 0; i < n; ++i) {
        if (!c->signs[i].valid) continue;
        /* Su forward del escaneo anterior = el de ahora mas lo que avanzo: si estaba
         * por delante del eje y ahora esta detras, acaba de ser rebasado. */
        const float before = c->signs[i].forward_cm + mf;
        if (before >= 0.0f && c->signs[i].forward_cm < 0.0f && fabsf(c->signs[i].x_cm) < c->cfg.lane_width_cm * 0.8f)
            c->last_sign_color = c->signs[i].color;
        /* La estima (giroscopio y velocidad ordenada) se desvia con la distancia: un
         * recuerdo que lleva 1,5 m sin verse ya no apunta al letrero. */
        if (c->signs[i].forward_cm < -60.0f || fabsf(c->signs[i].x_cm) > 250.0f ||
            c->signs[i].stale_cm > 150.0f) c->signs[i].valid = false;
    }
}

/* Memoria por seccion de los letreros con color. Registra (con la estima de la recta: velocidad ordenada x tiempo
 * desde la ultima esquina) donde estan los que acaba de ver la camara, y reinyecta los de esta seccion que ya se
 * vieron en una vuelta anterior; el mapa solo los acepta si hay un pilar compacto del LiDAR cerca de ahi. */
static void lap_memory_step(wro_controller_t *c, const local_planner_config_t *pc)
{
    /* El avance de la seccion se mide desde la ultima esquina: al reiniciarse, la correccion tambien. */
    if (c->travel_cm + 1.0f < c->lap_prev_travel_cm) c->lap_off_cm = 0.0f;
    c->lap_prev_travel_cm = c->travel_cm;
    if (!c->cfg.lap_memory_enabled || c->phase != WRO_PHASE_RUN || c->direction == 0 || c->corner_active || c->corners_done < 1 ||
        c->sidestep_phase != 0 || c->travel_cm > 2000.0f) return;
    const int sec = ((c->corners_done % 4) + 4) % 4;
    const float d = imu_wrap_deg(c->yaw.yaw_deg - c->course_deg) * DEG_TO_RAD;
    if (fabsf(d) > 0.6f) return;
    const float cd = cosf(d), sd = sinf(d);
    const float mt = c->travel_cm + c->lap_off_cm;
    const int n = (int)(sizeof(c->signs) / sizeof(c->signs[0]));
    const int slots = (int)(sizeof(c->lap_mem[0]) / sizeof(c->lap_mem[0][0]));
    const float lane = isfinite(c->debug.lane_cm) ? c->debug.lane_cm : 0.0f;
    for (int i = 0; i < n; ++i) {
        if (!c->signs[i].valid || c->signs[i].stale_cm > 0.5f || (c->signs[i].color != 1 && c->signs[i].color != 2)) continue;
        const float x = c->signs[i].x_cm, f = c->signs[i].forward_cm;
        const float lat = x * cd + f * sd, fw = -x * sd + f * cd;
        if (fw < 15.0f || fw > 90.0f || !isfinite(c->debug.lane_cm)) continue;
        const float along = mt + fw, v = lat - lane;
        if (fabsf(v) > 70.0f) continue;
        int match = -1, free_slot = -1;
        for (int k = 0; k < slots; ++k) {
            if (!c->lap_mem[sec][k].used) { if (free_slot < 0) free_slot = k; continue; }
            if (c->lap_mem[sec][k].color == c->signs[i].color && fabsf(c->lap_mem[sec][k].along_cm - along) < 30.0f &&
                fabsf(c->lap_mem[sec][k].lat_cm - v) < 30.0f) match = k;
        }
        if (match >= 0) {
            /* Reencuentro con uno conocido: afina el recuerdo, no lo descarta. */
            c->lap_mem[sec][match].along_cm += 0.3f * (along - c->lap_mem[sec][match].along_cm);
            c->lap_mem[sec][match].lat_cm += 0.3f * (v - c->lap_mem[sec][match].lat_cm);
            continue;
        }
        if (free_slot >= 0) {
            c->lap_mem[sec][free_slot].along_cm = along;
            c->lap_mem[sec][free_slot].lat_cm = v;
            c->lap_mem[sec][free_slot].color = c->signs[i].color;
            c->lap_mem[sec][free_slot].used = true;
        }
    }
    float xf[12], found_xf[12];
    uint8_t colors[6], found[6];
    int slot_of[6];
    size_t m = 0;
    for (int k = 0; k < slots; ++k) {
        if (!c->lap_mem[sec][k].used) continue;
        const float fw = c->lap_mem[sec][k].along_cm - mt;
        if (fw < -5.0f || fw > 90.0f) continue;
        const float lat = c->lap_mem[sec][k].lat_cm + lane;
        xf[2 * m] = lat * cd - fw * sd;
        xf[2 * m + 1] = lat * sd + fw * cd;
        slot_of[m] = k;
        colors[m++] = c->lap_mem[sec][k].color;
    }
    if (!m) return;
    memset(found_xf, 0, sizeof(found_xf));
    local_obstacle_map_add_memory_within(&c->map, pc, xf, colors, m, 20.0f, found_xf, found);
    for (size_t k = 0; k < m; ++k) {
        c->lap_dbg[0] += 1.0f;
        if (!found[k]) continue;
        c->lap_dbg[found[k] == 2 ? 2 : 1] += 1.0f;
        /* Donde se ha encontrado de verdad el letrero frente a donde se esperaba: la diferencia es el error de la
         * estima de avance. Se corrige poco a poco (no se fia de una sola medida). */
        const float fw_exp = c->lap_mem[sec][slot_of[k]].along_cm - mt;
        const float fx = found_xf[2 * k], ff = found_xf[2 * k + 1];
        const float fw_obs = -fx * sd + ff * cd;
        const float delta = fw_exp - fw_obs;
        c->lap_dbg[3] = delta;
        if (fabsf(delta) < 25.0f) {
            c->lap_off_cm += 0.4f * delta;
            if (c->lap_off_cm > 40.0f) c->lap_off_cm = 40.0f;
            if (c->lap_off_cm < -40.0f) c->lap_off_cm = -40.0f;
        }
    }
}

/* Letreros en coordenadas del campo. Con la pose del localizador (2-3 cm) cada letrero con color confirmado por la
 * camara y el LiDAR se guarda donde esta, y en cuanto esta en la zona que ve el robot se reinyecta anclandolo a un
 * pilar compacto del LiDAR a poca distancia: el color se recuerda aunque la camara no lo vea (fuera de su campo
 * de vision, de lado). */
/* La pose del campo a la hora del mapa del planificador (adelantada lo que avanza y gira el robot hasta que la orden hace efecto). */
static arena_pose_t map_pose(const wro_controller_t *c)
{
    arena_pose_t p = c->arena.pose;
    if (!c->cfg.arena_map_lead) return p;
    const float th = p.theta_deg * DEG_TO_RAD;
    p.x_cm += c->map_lead_cm * sinf(th);
    p.z_cm += c->map_lead_cm * cosf(th);
    p.theta_deg = imu_wrap_deg(p.theta_deg + c->map_lead_deg);
    return p;
}

/* Un punto del robot (centroide de la cara visible) al centro del letrero, o al revés (sign = +1 / -1). */
static void sign_bias(const wro_controller_t *c, float *qx, float *qf, float sign)
{
    const float r = hypotf(*qx, *qf), b = sign * c->cfg.arena_sign_bias_cm;
    if (r < 8.0f || r + b < 1.0f) return;
    const float s = (r + b) / r;
    *qx *= s; *qf *= s;
}

static void world_signs_step(wro_controller_t *c, const local_planner_config_t *pc)
{
    const arena_pose_t mp = map_pose(c);
    if (!c->arena.valid || !isfinite(c->arena.pose.x_cm) || c->arena.rms_cm > 3.5f || c->phase != WRO_PHASE_RUN) return;
    const int slots = (int)(sizeof(c->world_signs) / sizeof(c->world_signs[0]));
    const bool turning = fabsf(c->yaw.rate_dps) > 25.0f;
    /* 1) Anotar los que la camara ve ahora, ya confirmados por el LiDAR y a una distancia fiable. */
    for (size_t k = 0; k < c->live_colored && !turning; ++k) {
        const local_colored_object_t *o = &c->map.colored[k];
        if (!o->confirmed || (o->color != 1 && o->color != 2) || o->forward_cm < 12.0f || o->forward_cm > 90.0f) continue;
        float wx, wz, fx, fz;
        arena_to_world(&c->arena.pose, o->x_cm, o->forward_cm, &wx, &wz);
        float bx = o->x_cm, bf = o->forward_cm;
        sign_bias(c, &bx, &bf, 1.0f);
        arena_to_world(&mp, bx, bf, &fx, &fz);
        int match = -1, free_slot = -1;
        for (int i = 0; i < slots; ++i) {
            if (!c->world_signs[i].used) { if (free_slot < 0) free_slot = i; continue; }
            if (hypotf(c->world_signs[i].x_cm - wx, c->world_signs[i].z_cm - wz) < 12.0f) match = i;
        }
        if (match >= 0) {
            if (c->world_signs[match].color == o->color) {
                c->world_signs[match].x_cm += 0.25f * (wx - c->world_signs[match].x_cm);
                c->world_signs[match].z_cm += 0.25f * (wz - c->world_signs[match].z_cm);
                c->world_signs[match].fx_cm += 0.25f * (fx - c->world_signs[match].fx_cm);
                c->world_signs[match].fz_cm += 0.25f * (fz - c->world_signs[match].fz_cm);
                if (c->world_signs[match].hits < 250) ++c->world_signs[match].hits;
            }
        } else if (free_slot >= 0) {
            c->world_signs[free_slot].x_cm = wx;
            c->world_signs[free_slot].z_cm = wz;
            c->world_signs[free_slot].fx_cm = fx;
            c->world_signs[free_slot].fz_cm = fz;
            c->world_signs[free_slot].color = o->color;
            c->world_signs[free_slot].hits = 1;
            c->world_signs[free_slot].used = true;
        }
    }
    /* 2) Reinyectar los de la zona que ve el robot. */
    float xf[32], found_xf[32];
    uint8_t colors[16], found[16];
    size_t m = 0;
    int n_signs = 0;
    for (int i = 0; i < slots; ++i) {
        if (!c->world_signs[i].used) continue;
        ++n_signs;
        if (c->world_signs[i].hits < 3) continue;           /* hace falta haberlo visto varias veces */
        float qx, qf;
        arena_to_robot(&c->arena.pose, c->world_signs[i].x_cm, c->world_signs[i].z_cm, &qx, &qf);
        if (qf < -5.0f || qf > 110.0f || fabsf(qx) > 100.0f) continue;
        xf[2 * m] = qx; xf[2 * m + 1] = qf; colors[m++] = c->world_signs[i].color;
    }
    c->lap_dbg[3] = (float)n_signs;
    if (!m) return;
    memset(found_xf, 0, sizeof(found_xf));
    local_obstacle_map_add_memory_within(&c->map, pc, xf, colors, m, 8.0f, found_xf, found);
    for (size_t k = 0; k < m; ++k) {
        c->lap_dbg[0] += 1.0f;
        if (found[k] == 1) c->lap_dbg[1] += 1.0f;
        else if (found[k] == 2) c->lap_dbg[2] += 1.0f;
    }
}

/* Pose en el campo: la odometria da el avance y el rumbo (giroscopio); el barrido se casa con las paredes
 * conocidas y corrige lo que se haya desviado. La primera pose sale de buscar el barrido inicial en la seccion de
 * salida (el robot arranca ahi, mirando a un lado u otro). */
static void arena_step(wro_controller_t *c, const local_lidar_point_t *points, size_t count)
{
    if (!c->cfg.arena_enabled || count == 0) return;
    if (c->phase == WRO_PHASE_IDLE || c->phase == WRO_PHASE_DONE || c->phase == WRO_PHASE_FAULT) return;
    static arena_point_t ap[LOCAL_MAP_MAX_POINTS];
    const size_t n = count < LOCAL_MAP_MAX_POINTS ? count : LOCAL_MAP_MAX_POINTS;
    for (size_t i = 0; i < n; ++i) {
        ap[i].angle_deg = points[i].angle_deg;
        ap[i].distance_cm = points[i].distance_cm;
        ap[i].valid = points[i].valid;
    }
    const float lf = c->cfg.planner.lidar_forward_cm;
    if (!c->arena.valid) {
        /* Demostracion del parqueo: ya en el carril, no en la seccion de salida; no hay busqueda posible. */
        if (c->cfg.park_demo_dir != 0 && !c->cfg.start_in_lot) return;
        if (c->arena_tries >= 40 || c->corners_done > 0) return;
        ++c->arena_tries;
        if (arena_acquire(&c->arena, ap, n, lf))
            c->arena_yaw_off = imu_wrap_deg(c->arena.pose.theta_deg - c->yaw.yaw_deg);
        return;
    }
    arena_predict(&c->arena, c->scan_ds_cm, imu_wrap_deg(c->yaw.yaw_deg + c->arena_yaw_off));
    if (arena_update(&c->arena, ap, n, lf)) c->arena_yaw_off = imu_wrap_deg(c->arena_yaw_off + c->arena.last_dtheta_deg);
}

/* Con la pose del campo, una esquina solo puede empezar donde hay esquina: |x| y |z| a partir de la mitad del bloque
 * interior (menos un margen). En mitad de una recta (aunque el robot vaya girado) no hay esquina que contar: asi un
 * giro de 60 grados para esquivar un letrero, o una pared interior vista de frente, ya no se toman por una. */
static bool arena_corner_ok(const wro_controller_t *c)
{
    if (!c->cfg.arena_gates_corners || !c->arena.valid) return true;
    const float lim = c->cfg.arena.inner_half_cm - c->cfg.arena_corner_margin_cm;
    return fabsf(c->arena.pose.x_cm) >= lim && fabsf(c->arena.pose.z_cm) >= lim;
}

/* ------------------------------------------------------------ linea de carrera */

/* Terminada la primera vuelta, con la pose del campo y los letreros ya conocidos, se calcula la linea una sola vez. */
static void race_build_if_ready(wro_controller_t *c)
{
    if (c->race.valid || c->race_tried) return;
    if (c->corner_active || c->corners_done < (int)c->cfg.race_after_corners) return;
    if (!c->arena.valid || c->direction == 0) return;
    race_sign_t s[RACE_MAX_SIGNS];
    int n = 0;
    const int slots = (int)(sizeof(c->world_signs) / sizeof(c->world_signs[0]));
    for (int i = 0; i < slots && n < RACE_MAX_SIGNS; ++i) {
        if (!c->world_signs[i].used || c->world_signs[i].hits < 3) continue;
        s[n].x_cm = c->world_signs[i].fx_cm; s[n].z_cm = c->world_signs[i].fz_cm; s[n].color = c->world_signs[i].color;
        ++n;
    }
    c->race_tried = true;
    race_config_t rc = c->cfg.race;
    rc.outer_half_cm = c->cfg.arena.outer_half_cm;
    rc.inner_half_cm = c->cfg.arena.inner_half_cm;
    rc.half_width_cm = 0.5f * c->cfg.planner.chassis_width_cm;
    rc.pass_side_red = c->cfg.planner.enforce_color_side ? c->cfg.planner.pass_side_red : 0;
    rc.pass_side_green = c->cfg.planner.enforce_color_side ? c->cfg.planner.pass_side_green : 0;
    race_line_build(&c->race, &rc, c->direction, s, n);
    c->race_idx = -1;
    c->race_slips0 = c->odom.slip_count;
    c->race_grip = 1.0f;
}

/* Cada ciclo: donde esta el robot en la linea, a que velocidad ir y que posicion lateral prefiere el corredor. */
static void race_step(wro_controller_t *c, const local_planner_config_t *pc, float dt)
{
    c->race_active = false;
    c->planner_state.ref_rows = 0;
    if (!c->cfg.race_enabled || c->phase != WRO_PHASE_RUN) return;
    /* Las ultimas rectas son del parqueo: el robot llega a la seccion de salida como lo haria sin linea. */
    if (c->corners_done >= (int)c->cfg.laps * 4 - (int)c->cfg.race_stop_before_end) return;
    race_build_if_ready(c);
    if (!c->race.valid || !c->arena.valid || !(c->arena.rms_cm <= 3.5f) || !isfinite(c->arena.pose.x_cm)) return;
    float lat = 0.0f, dist = 0.0f;
    const int idx = race_line_locate(&c->race, c->arena.pose.x_cm, c->arena.pose.z_cm, c->race_idx, 80.0f, &lat, &dist);
    if (idx < 0) return;
    c->race_idx = idx; c->race_lat_cm = lat; c->race_dist_cm = dist;
    const float robot_field = imu_wrap_deg(c->yaw.yaw_deg + c->arena_yaw_off);
    if (dist > c->cfg.race_max_dist_cm || fabsf(imu_wrap_deg(c->race.heading_deg[idx] - robot_field)) > 70.0f) return;
    c->race_active = true;
    /* Empezar la esquina antes solo si no hay un letrero conocido cerca de ella (antes o despues): cortar la esquina
     * con un letrero ahi lo dejaria en el camino. La linea ya lo rodea, pero el giro de la esquina es del controlador. */
    c->race_corner_clear = false;
    {
        int k = idx;
        float to_corner = 0.0f;
        bool found = false;
        for (int step = 0; step < c->race.n && to_corner < 260.0f; ++step) {
            if (c->race.corner[k]) { found = true; break; }
            to_corner += c->race.h[k];
            k = (k + 1) % c->race.n;
        }
        if (found) {
            const float sc = c->race.s[k];
            bool clear = true;
            for (int j = 0; j < c->race.signs && clear; ++j) {
                float d = c->race.sign_s[j] - sc;                       /* + : el letrero esta despues de la esquina */
                while (d > 0.5f * c->race.length_cm) d -= c->race.length_cm;
                while (d <= -0.5f * c->race.length_cm) d += c->race.length_cm;
                if (d > -c->cfg.race_corner_sign_cm && d < c->cfg.race_corner_sign_after_cm) clear = false;
            }
            c->race_corner_clear = clear;
        }
    }
    /* Un pilar que la linea no esperaba (no lo vi en la primera vuelta): velocidad base un rato. */
    if (c->race_hold_s > 0.0f) c->race_hold_s -= dt;
    /* Un pilar compacto delante que no esta entre los letreros conocidos del campo: la linea no lo esperaba. */
    {
        float pf[16];
        const size_t np = local_obstacle_map_compact_pillars(&c->map, pc, 10.0f, 105.0f, pf, 8);
        const int slots = (int)(sizeof(c->world_signs) / sizeof(c->world_signs[0]));
        for (size_t k = 0; k < np; ++k) {
            float wx, wz;
            float bx = pf[2 * k], bf = pf[2 * k + 1];
            sign_bias(c, &bx, &bf, 1.0f);
            const arena_pose_t mpp = map_pose(c);
            arena_to_world(&mpp, bx, bf, &wx, &wz);
            bool known = false;
            for (int i = 0; i < slots && !known; ++i)
                if (c->world_signs[i].used && hypotf(c->world_signs[i].fx_cm - wx, c->world_signs[i].fz_cm - wz) < 14.0f) known = true;
            if (!known) { c->race_hold_s = c->cfg.race_unknown_hold_s; break; }
        }
    }
    const float base = pc->speed_cm_s;
    float v = race_line_speed(&c->race, idx, c->cfg.race_lookahead_cm);
    if (v > base) {
        /* El agarre se mide con los encoders: un patinaje nuevo desde que se calculo la linea quita parte del exceso. */
        c->race_grip = 1.0f - c->cfg.race_slip_derate * (float)(c->odom.slip_count - c->race_slips0);
        if (c->race_grip < 0.0f) c->race_grip = 0.0f;
        const float gain = (c->corners_done / 4 >= (int)c->cfg.laps - 1 ? 1.0f : c->cfg.race_gain_lap2) * c->race_grip;
        v = base + gain * (v - base);
    }
    if ((c->race_hold_s > 0.0f || dist > c->cfg.race_speed_dist_cm) && v > base) v = base;
    c->race_v_line = race_line_speed(&c->race, idx, c->cfg.race_lookahead_cm);
    c->race_v_cmd = v;
    /* La ultima recta antes de la ultima esquina entrega el robot al parqueo: ahi no se pega a la pared interior. */
    if (!c->cfg.race_lateral_enabled || (c->cfg.parking_enabled && c->corners_done >= (int)c->cfg.laps * 4 - 1)) return;
    /* Posicion lateral preferida a 5, 10, 15... cm por delante, en los ejes del carril del planificador. */
    const float sd = sinf(imu_wrap_deg(c->course_deg - c->map.yaw_deg) * DEG_TO_RAD), cd = cosf(imu_wrap_deg(c->course_deg - c->map.yaw_deg) * DEG_TO_RAD);
    const float course_field = imu_wrap_deg(c->course_deg + c->arena_yaw_off);
    int rows = (int)(pc->plan_horizon_cm / 5.0f);
    if (rows > LOCAL_REF_ROWS) rows = LOCAL_REF_ROWS;
    for (int k = 0; k < LOCAL_REF_ROWS; ++k) c->planner_state.ref_v_cm[k] = NAN;
    float prev_u = -1.0e9f, prev_v = 0.0f;
    int next_row = 0;
    for (int m = 0; m <= 48 && next_row < rows; ++m) {
        float px, pz, hd;
        race_line_point(&c->race, idx, 4.0f * (float)m, &px, &pz, &hd);
        if (fabsf(imu_wrap_deg(hd - course_field)) > 55.0f) break;
        float qx, qf;
        const arena_pose_t mp = map_pose(c);
        arena_to_robot(&mp, px, pz, &qx, &qf);
        const float u = qx * sd + qf * cd, v2 = qx * cd - qf * sd;
        if (m > 0 && u <= prev_u) break;
        while (next_row < rows) {
            const float ur = 5.0f * (float)(next_row + 1);
            if (ur > u) break;
            if (m > 0 && ur > prev_u) {
                const float t = (ur - prev_u) / (u - prev_u);
                const float vl = prev_v + t * (v2 - prev_v);
                /* Cerca de una esquina la referencia es el centro del carril (como en las vueltas sin linea). */
                const float f = race_line_corner_factor(&c->race, c->race.s[idx] + ur, c->cfg.race_blend_before_cm, c->cfg.race_blend_after_cm);
                c->planner_state.ref_v_cm[next_row] = c->debug.lane_cm + f * (vl - c->debug.lane_cm);
            }
            ++next_row;
        }
        prev_u = u; prev_v = v2;
    }
    c->planner_state.ref_rows = rows;
}

/* ------------------------------------------------------------ pegado a la pared interior */

/* Vueltas 2 y 3: el robot ya conoce los letreros. En las rectas se pega a la pared interior (atajo) y solo se despega
 * cuando un letrero conocido, por delante o aun alrededor, le exige estar en otro sitio. El resto (esquivar, holgura,
 * cortinas) lo sigue haciendo el planificador reactivo: esto solo le pide un desplazamiento del centro del carril. */
static void hug_step(wro_controller_t *c, const local_planner_config_t *pc, float front_cm, float dt)
{
    float target = 0.0f;
    bool active = false;
    const wro_controller_config_t *g = &c->cfg;
    if (c->hug_hold_s > 0.0f) c->hug_hold_s -= dt;
    for (int once = 0; once < 1; ++once) {
        if (!g->hug_enabled || c->phase != WRO_PHASE_RUN || c->direction == 0) break;
        if (c->corners_done < 4 || c->corners_done >= (int)g->laps * 4 - (int)g->hug_stop_before_end) break;
        if (!c->arena.valid || !(c->arena.rms_cm <= 3.5f) || !isfinite(c->arena.pose.x_cm)) break;
        if (c->corner_active || c->pivoting || c->sidestep_phase != 0 || c->planner_state.escaping || c->uturn_pending) break;
        if (fabsf(imu_wrap_deg(c->yaw.yaw_deg - c->course_deg)) > 25.0f) break;
        const float radius = pc->wheelbase_cm / tanf(pc->steer_hard_deg * DEG_TO_RAD);
        const float trigger = 0.5f * g->lane_width_cm + radius - pc->lidar_forward_cm + g->corner_trigger_extra_cm;
        if ((g->hug_corner_clear_cm > 0.0f && front_cm < trigger + g->hug_corner_clear_cm) || c->travel_cm < g->hug_after_corner_cm) break;
        /* Un pilar compacto que no esta entre los letreros conocidos: centrado un rato (no se sabe de que lado hay que pasar). */
        const arena_pose_t mp = map_pose(c);
        const int slots = (int)(sizeof(c->world_signs) / sizeof(c->world_signs[0]));
        {
            float pf[16];
            const size_t np = local_obstacle_map_compact_pillars(&c->map, pc, 10.0f, 105.0f, pf, 8);
            for (size_t k = 0; k < np; ++k) {
                float bx = pf[2 * k], bf = pf[2 * k + 1], wx, wz;
                sign_bias(c, &bx, &bf, 1.0f);
                arena_to_world(&mp, bx, bf, &wx, &wz);
                bool known = false;
                for (int i = 0; i < slots && !known; ++i)
                    if (c->world_signs[i].used && hypotf(c->world_signs[i].fx_cm - wx, c->world_signs[i].fz_cm - wz) < 14.0f) known = true;
                if (!known) { c->hug_hold_s = g->hug_unknown_hold_s; break; }
            }
        }
        if (c->hug_hold_s > 0.0f) break;
        active = true;
        const float inner = (float)c->direction;          /* +1: la pared interior queda a la derecha (sentido horario) */
        const float sd = sinf(imu_wrap_deg(c->course_deg - c->map.yaw_deg) * DEG_TO_RAD);
        const float cd = cosf(imu_wrap_deg(c->course_deg - c->map.yaw_deg) * DEG_TO_RAD);
        const float limit = 0.5f * g->lane_width_cm - 0.5f * pc->chassis_width_cm - 6.0f;   /* el robot no se acerca mas a una pared */
        float lo = -limit, hi = limit;
        const float need = 2.5f * 1.4142f + 0.5f * pc->chassis_width_cm + 3.0f;             /* centro del robot a centro del letrero */
        for (int i = 0; i < slots; ++i) {
            if (!c->world_signs[i].used || c->world_signs[i].hits < 3 || !pc->enforce_color_side) continue;
            const int side = c->world_signs[i].color == 1 ? pc->pass_side_red : c->world_signs[i].color == 2 ? pc->pass_side_green : 0;
            if (side == 0) continue;
            float qx, qf;
            arena_to_robot(&mp, c->world_signs[i].fx_cm, c->world_signs[i].fz_cm, &qx, &qf);
            const float u = qx * sd + qf * cd, v = qx * cd - qf * sd;
            if (u > g->hug_detach_cm || u < -g->hug_after_sign_cm) continue;
            const float lam = v - c->debug.lane_cm;          /* donde esta el letrero respecto al centro del carril (+ a la derecha) */
            if (side > 0) { if (lam + need > lo) lo = lam + need; }
            else { if (lam - need < hi) hi = lam - need; }
        }
        if (lo > hi) break;                                   /* sin hueco coherente: centrado y que decida el reactivo */
        target = inner * g->hug_offset_cm;
        if (target < lo) target = lo;
        if (target > hi) target = hi;
    }
    c->hug_active = active;
    c->hug_target_cm = target;
    /* El desplazamiento cambia con una pendiente maxima respecto al avance (y vuelve a 0 al desactivarse). */
    const float max_step = g->hug_rate * fmaxf(fabsf(c->scan_ds_cm), 0.5f);
    float d = target - c->hug_cmd_cm;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    c->hug_cmd_cm += d;
    c->planner_state.lane_offset_cm = c->hug_cmd_cm;
}

/* Mueve los puntos del barrido al instante en que la orden de este ciclo hara
 * efecto: cada retorno se midio `edad` antes (la vuelta del LiDAR dura 100 ms y el
 * ultimo punto es el mas reciente) mas la latencia de calculo y actuacion, y en ese
 * tiempo el carrito avanzo (velocidad ordenada) y giro (giroscopio). Sin esto el
 * planificador decide sobre una escena de hace 100-200 ms, que a 20 cm/s son 2-4 cm:
 * justo el margen de un paso estrecho. Devuelve el giro predicho (grados) para el
 * rumbo; sin datos de giroscopio solo corrige el avance. */
static float compensate_motion(const wro_controller_t *c, const local_planner_config_t *pl,
                               const local_lidar_point_t *in, size_t n, local_lidar_point_t *out)
{
    const float rate = c->yaw.has_last && isfinite(c->yaw.rate_dps) ? c->yaw.rate_dps : 0.0f;
    const float v = c->odom_in_use ? c->odom.v_cm_s : c->last_speed_cm_s;
    for (size_t i = 0; i < n; ++i) {
        out[i] = in[i];
        if (!(in[i].valid || in[i].blind) || !(in[i].distance_cm > 0.0f)) continue;
        const float age = (n > 1 ? (1.0f - (float)i / (float)(n - 1)) * pl->scan_period_s : 0.0f) + c->cfg.latency_s;
        float psi = rate * age * DEG_TO_RAD;
        if (psi > 1.0f) psi = 1.0f; else if (psi < -1.0f) psi = -1.0f;
        const float ds = v * age;
        const float mx = ds * sinf(0.5f * psi), mf = ds * cosf(0.5f * psi);
        const float a = in[i].angle_deg * DEG_TO_RAD;
        const float x = in[i].distance_cm * sinf(a) - mx, f = in[i].distance_cm * cosf(a) - mf;
        const float cs = cosf(psi), sn = sinf(psi);
        const float nx = x * cs - f * sn, nf = x * sn + f * cs;
        out[i].distance_cm = hypotf(nx, nf);
        out[i].angle_deg = atan2f(nx, nf) * RAD_TO_DEG;
    }
    return rate * c->cfg.latency_s;
}


/* -------------------------------------------------------------------- parqueo */
static park_params_t g_ack_park_params;
/* Entrada por el camino real de la salida, recorrido al reves (experimental, apagado: en 8 rondas
 * llega a 1-2 repasadas en 4 de ellas, pero el tramo final no es robusto y otras suben a 7). */
#ifndef PARK_LOST_MAX_SCANS
#define PARK_LOST_MAX_SCANS 30
#endif
#ifndef PARK_RETRACE_ENABLED
#define PARK_RETRACE_ENABLED 0
#endif
static int park_mirror(const wro_controller_t *c);

static void park_params_from_cfg(const wro_controller_t *c, park_params_t *p)
{
    const local_planner_config_t *pl = &c->cfg.planner;
    park_params_default(p, pl->chassis_width_cm, pl->chassis_length_cm, pl->chassis_rear_cm);
    p->margin_cm = c->cfg.park_margin_cm;
    p->lot_length_cm = c->cfg.park_lot_length_cm;
}

/* Rumbo del chasis en el marco del cajon medido con el giroscopio. La pose localizada se reajusta
 * con el LiDAR en cada barrido y su rumbo salta varios grados dentro del cajon; el giroscopio no.
 * Se usa para medir el giro de cada arco (park_arc_dtheta_deg). */
static float park_heading(const wro_controller_t *c)
{
    return -(float)park_mirror(c) * imu_wrap_deg(c->yaw.yaw_deg - c->park_course_deg);
}

/* Avance a ciegas de un arco del plan: la pose que deberia tener el chasis al terminarlo. Sirve
 * para distinguir un desvio de verdad (choque, patinazo) del ruido de la pose medida. */
static void park_plan_advance(wro_controller_t *c, int dir, int steer, float step_cm)
{
    const local_planner_config_t *pl = &c->cfg.planner;
    const float ds = (float)dir * step_cm;
    const float k = tanf((float)steer * pl->steer_hard_deg * DEG_TO_RAD) / pl->wheelbase_cm;
    const float dth = ds * k * RAD_TO_DEG;
    const float mid = (c->park_plan_pose.theta_deg + 0.5f * dth) * DEG_TO_RAD;
    c->park_plan_pose.u_cm += ds * cosf(mid);
    c->park_plan_pose.w_cm += ds * sinf(mid);
    c->park_plan_pose.theta_deg = imu_wrap_deg(c->park_plan_pose.theta_deg + dth);
}

/* Aproximacion por el carril hasta la pose donde acaba la salida: seguidor de rumbo suave a la
 * velocidad de busqueda (el carril esta libre, no hace falta ir a 8 cm/s). Devuelve true al
 * llegar, o al pasarse (entonces se entra desde donde este). */
static bool park_cruise_step(wro_controller_t *c, float dt, wro_command_t *out)
{
    (void)dt;
    const local_planner_config_t *pl = &c->cfg.planner;
    const park_pose_t *t = &c->park_exit_pose;
    /* Donde acabo la salida es el minimo de su meta (28,5 cm de la pared): desde ahi el A* tiene
     * que entrar con dos repasadas. Con 2 cm mas por delante y 1,5 mas lejos de la pared la
     * misma maniobra cabe con una, que es lo que se busca (medido en el planificador: 1 repasada
     * desde (25,30,5) y 2 desde (25,28,5)). */
    const bool retrace = PARK_RETRACE_ENABLED && c->park_trace_n > 8;
    const float du = t->u_cm + (retrace ? 0.0f : 2.0f) - c->park_pose.u_cm;
    const float dw = t->w_cm + (retrace ? 0.0f : 1.5f) - c->park_pose.w_cm;
    const float speed = fmaxf(c->cfg.park_speed_cm_s, c->cfg.park_search_speed_cm_s);
    const float th_goal = retrace ? t->theta_deg : 0.0f;
    if ((du <= 1.0f && fabsf(dw) <= 0.8f && fabsf(imu_wrap_deg(c->park_pose.theta_deg - th_goal)) <= (retrace ? 1.2f : 3.0f)) || du < -1.5f) return true;
    const float look_cm = 18.0f;
    float th_ref = atan2f(dw, look_cm) * RAD_TO_DEG;
    if (retrace) {
        /* El camino de vuelta empieza con el rumbo con que acabo la salida: un error de unos grados
         * ahi se arrastra hasta el cajon (con el volante al tope no se puede corregir despues). */
        const float wgt = fminf(fmaxf(1.0f - du / 15.0f, 0.0f), 1.0f);
        th_ref = (1.0f - wgt) * th_ref + wgt * (th_goal + 0.5f * th_ref);
    }
    if (th_ref > 22.0f) th_ref = 22.0f;
    if (th_ref < -22.0f) th_ref = -22.0f;
    float steer = 2.5f * (th_ref - c->park_pose.theta_deg);
    if (steer > pl->steer_hard_deg) steer = pl->steer_hard_deg;
    if (steer < -pl->steer_hard_deg) steer = -pl->steer_hard_deg;
    out->speed_cm_s = speed;
    out->steer_deg = -(float)park_mirror(c) * steer;
    out->yaw_rate_dps = speed * tanf(out->steer_deg * DEG_TO_RAD) / pl->wheelbase_cm * RAD_TO_DEG;
    out->explicit_yaw_rate = false;
    out->action = LOCAL_ACTION_FORWARD;
    return false;
}

/* El campo de costes es unico (ocupa casi 1 MB): se reconstruye al cambiar de meta. */
static bool park_prepare(wro_controller_t *c, park_goal_t goal)
{
    if (c->cfg.park_ackermann) {
        park_params_from_cfg(c, &g_ack_park_params);
        c->park_field_goal = (int)goal + 1;
        return true;
    }
    if (c->park_field_goal == (int)goal + 1 && park_field_ready()) return true;
    park_params_t p;
    park_params_from_cfg(c, &p);
    if (!park_field_build(&p, goal)) return false;
    c->park_field_goal = (int)goal + 1;
    return true;
}

static int park_mirror(const wro_controller_t *c) { return c->direction < 0 ? 1 : -1; }

/* Orden para un movimiento del campo: girar en el sitio hacia el rumbo del movimiento y,
 * una vez alineado (3 grados), avanzar o retroceder sujetando ese rumbo con el giroscopio. */
static void park_command(wro_controller_t *c, const park_decision_t *d, float theta_deg,
                         wro_command_t *out)
{
    const int mirror = park_mirror(c);
    out->speed_cm_s = 0.0f;
    out->steer_deg = 0.0f;
    out->yaw_rate_dps = 0.0f;
    out->explicit_yaw_rate = true;
    out->action = LOCAL_ACTION_STOP;
    if (d->move == PARK_MOVE_NONE) return;
    const float err = imu_wrap_deg(d->target_theta_deg - theta_deg);   /* > 0: hay que girar contra reloj */
    const bool translate = d->move == PARK_MOVE_FORWARD || d->move == PARK_MOVE_BACK;
    float theta_rate = 0.0f;
    if (!translate || fabsf(err) > 2.0f) {
        /* El giro en el sitio (sin avance) se hace en pulsos: el motor sigue girando despues
         * de la orden (retardo de la placa + constante del motor) y repetir la consigna cada
         * barrido lo pasa de largo y lo devuelve. Se gira un barrido y se deja asentar el
         * siguiente, que es lo que hace el controlador al aparcar. */
        if (!translate) {
            if (c->park_rot_hold > 0) { --c->park_rot_hold; return; }
            c->park_rot_hold = 1;
        }
        float mag = fabsf(err) * 2.5f;
        if (mag < 30.0f) mag = 30.0f;                  /* por debajo, las ruedas no arrancan */
        if (mag > c->cfg.park_rot_dps) mag = c->cfg.park_rot_dps;
        theta_rate = err >= 0.0f ? mag : -mag;
        out->action = theta_rate > 0.0f ? (mirror > 0 ? LOCAL_ACTION_LEFT_HARD : LOCAL_ACTION_RIGHT_HARD)
                                        : (mirror > 0 ? LOCAL_ACTION_RIGHT_HARD : LOCAL_ACTION_LEFT_HARD);
    } else {
        const float v = c->cfg.park_speed_cm_s;
        out->speed_cm_s = d->move == PARK_MOVE_FORWARD ? v : -v;
        theta_rate = 2.0f * err;
        if (theta_rate > 20.0f) theta_rate = 20.0f;
        if (theta_rate < -20.0f) theta_rate = -20.0f;
        out->action = d->move == PARK_MOVE_FORWARD ? LOCAL_ACTION_FORWARD : LOCAL_ACTION_REVERSE;
    }
    /* theta crece contra reloj; el rumbo del giroscopio, a favor. Con la pared a la
     * izquierda el marco canonico esta reflejado y los signos se invierten. */
    out->yaw_rate_dps = -(float)mirror * theta_rate;
}

/* Un barrido en el cajon: localiza, decide y ordena. Entrar (PARKING) o salir (PARK_EXIT). */
/* ---- Traza de la salida y recorrido al reves (la entrada sigue el camino real de la salida). ---- */
#ifndef PARK_PROJECT_ENABLED
#define PARK_PROJECT_ENABLED 0
#endif
#define PARK_LATENCY_S 0.12f
#define TRACE_GAIN_TH 0.20f   /* 1/cm por rad */
#ifndef PARK_END_BIAS_W_CM
#define PARK_END_BIAS_W_CM 1.6f
#define PARK_END_BIAS_TH_DEG 1.0f
#endif
#define PARK_END_TH_AIM_DEG 1.5f   /* rumbo final apuntado: la meta pide casi 0 o negativo */
#define PARK_END_TH_TOL_DEG 0.5f
#define TRACE_GAIN_Y 0.010f   /* 1/cm^2 */
#define TRACE_TURN_STOP_CM 0.6f  /* se para antes del punto de giro: el motor sigue ~0,15 s */
static void park_trace_record(wro_controller_t *c, float wheelbase_cm)
{
    if (!PARK_RETRACE_ENABLED || c->park_trace_n >= PARK_TRACE_MAX || c->park_arc_direction == 0) return;
    const unsigned n = c->park_trace_n;
    if (n > 0 && hypotf(c->park_pose.u_cm - c->park_trace_u[n-1], c->park_pose.w_cm - c->park_trace_w[n-1]) < 0.8f) return;
    c->park_trace_u[n] = c->park_pose.u_cm; c->park_trace_w[n] = c->park_pose.w_cm;
    c->park_trace_th[n] = c->park_pose.theta_deg;
    const float s_park = -(float)park_mirror(c) * c->park_arc_steer_deg;   /* volante en el marco del cajon */
    c->park_trace_kappa[n] = tanf(s_park * DEG_TO_RAD) / wheelbase_cm;
    c->park_trace_dir[n] = (int8_t)c->park_arc_direction;
    ++c->park_trace_n;
}

/* Sigue la traza al reves: devuelve false si debe cederse al A* (desvio grande o fin del camino). */
static bool park_retrace_step(wro_controller_t *c, const park_params_t *params, wro_command_t *out)
{
    const local_planner_config_t *pl = &c->cfg.planner;
    int k = c->park_trace_k;
    park_pose_t p = c->park_pose;
    /* Gemelo: la orden de este barrido actua ~PARK_LATENCY_S despues (barrido, servo y motor); el
     * seguimiento se calcula sobre la pose proyectada con la velocidad medida y el volante puesto. */
    if (PARK_PROJECT_ENABLED && c->park_trace_dt > 0.0f) {
        const float v = c->scan_ds_cm / c->park_trace_dt;   /* con signo, cm/s */
        const float ds = v * PARK_LATENCY_S, tt = p.theta_deg * DEG_TO_RAD;
        p.u_cm += ds * cosf(tt); p.w_cm += ds * sinf(tt);
        p.theta_deg = imu_wrap_deg(p.theta_deg + ds * tanf(c->park_trace_steer_deg * DEG_TO_RAD) / pl->wheelbase_cm * RAD_TO_DEG);
    }
    float ex = 0.0f, ey = 0.0f, eth = 0.0f, ref_th = 0.0f;
    /* El punto mas cercano de la traza, siempre hacia atras (la traza vuelve sobre si misma en el
     * cambio de sentido, asi que no se busca fuera de la ventana). Las poses estimadas tienen ~0,5 cm
     * de ruido: la direccion entre dos vecinos no sirve, el rumbo guardado si. */
    /* Punto de giro: el primer punto por delante donde el sentido cambia. No se cruza de largo (el
     * coche se pasaria al tope): se llega a el por proyeccion sobre su rumbo y se para. */
    const int dir_k = -c->park_trace_dir[k > 0 ? k : 0];
    int turn = 0;
    for (int j = k - 1; j > 0; --j) if (-c->park_trace_dir[j] != dir_k) { turn = j; break; }
    while (k > turn && k > 0 && hypotf(p.u_cm - c->park_trace_u[k-1], p.w_cm - c->park_trace_w[k-1]) <=
                    hypotf(p.u_cm - c->park_trace_u[k], p.w_cm - c->park_trace_w[k])) --k;
    if (turn > 0 && k > turn) {
        const float th = c->park_trace_th[turn] * DEG_TO_RAD;
        const float proj = ((p.u_cm - c->park_trace_u[turn]) * cosf(th) + (p.w_cm - c->park_trace_w[turn]) * sinf(th)) * (float)dir_k;
        if (proj >= -TRACE_TURN_STOP_CM) k = turn;
    }
    c->park_trace_k = k;
    /* Tras el ultimo cambio de sentido: en cuanto el chasis esta dentro del cajon y paralelo (la meta
     * de siempre), se cede al A* para que lo confirme; seguir hasta el ultimo punto lo sacaba de ella. */
    if (turn == 0 && c->park_trace_cur_dir != 0 && k < (int)c->park_trace_n - 2) {
        const park_ackermann_result_t g = park_ackermann_plan(params, &(park_ackermann_config_t)
            {pl->wheelbase_cm, pl->steer_hard_deg, 3.5f, c->park_trace_cur_dir}, &p, PARK_GOAL_IN);
        if (g.at_goal) return false;
    }
    if (k <= 0) {
        /* Llego a la pose de partida de la salida pero con el rumbo aun unos grados corrido (el
         * volante ya iba al tope): unos centimetros mas, con el mismo sentido, lo cierran. */
        const float e0 = imu_wrap_deg(p.theta_deg - (c->park_trace_th[0] - PARK_END_TH_AIM_DEG));
        if (e0 > PARK_END_TH_TOL_DEG && c->park_trace_extra < 8 && c->park_trace_cur_dir != 0) {
            ++c->park_trace_extra;
            const float sg = (float)c->park_trace_cur_dir;
            const float sd = -sg * pl->steer_hard_deg;   /* hacia rumbo menor */
            out->speed_cm_s = sg * fminf(c->cfg.park_speed_cm_s, 3.0f);
            out->steer_deg = -(float)park_mirror(c) * sd;
            out->yaw_rate_dps = out->speed_cm_s * tanf(out->steer_deg * DEG_TO_RAD) / pl->wheelbase_cm * RAD_TO_DEG;
            out->explicit_yaw_rate = false;
            out->action = sg > 0 ? LOCAL_ACTION_FORWARD : LOCAL_ACTION_REVERSE;
            return true;
        }
        c->park_trace_done = true; return false;
    }
    ref_th = c->park_trace_th[k];
    float ru = c->park_trace_u[k], rw = c->park_trace_w[k];
    /* La meta del cajon deja ~1 cm y ~1 grado (la nariz roza el borde del lado del carril): el tramo
     * final se apunta un poco hacia la pared y con el rumbo algo cerrado. */
    if (turn == 0 && c->park_trace_cur_dir != 0) { rw -= PARK_END_BIAS_W_CM; ref_th -= PARK_END_BIAS_TH_DEG; }
    const float rt = ref_th * DEG_TO_RAD;
    ex = p.u_cm - ru; ey = -sinf(rt) * ex + cosf(rt) * (p.w_cm - rw);
    eth = imu_wrap_deg(p.theta_deg - ref_th) * DEG_TO_RAD;
    if (fabsf(ey) > 3.0f || fabsf(eth) > 20.0f * DEG_TO_RAD || !park_pose_free(params, &p)) return false;
    const int dir = -c->park_trace_dir[k];
    const float kappa_rec = c->park_trace_kappa[k];
    float kappa = kappa_rec - (float)dir * TRACE_GAIN_TH * eth - TRACE_GAIN_Y * ey;
    c->park_plan_expanded = k; c->park_plan_count = (int)(c->park_trace_kappa[k] * 1000.0f); c->park_plan_step = (int)(ey * 10.0f); c->park_plan_found = (int)(eth * 57.3f * 10.0f);
    float s_deg = atanf(kappa * pl->wheelbase_cm) * RAD_TO_DEG;
    if (s_deg > pl->steer_hard_deg) s_deg = pl->steer_hard_deg;
    if (s_deg < -pl->steer_hard_deg) s_deg = -pl->steer_hard_deg;
    const float steer_out = -(float)park_mirror(c) * s_deg;
    /* Cambio de sentido o primer tramo: parado, con el volante ya puesto (el servo tarda ~0,2 s). */
    if (dir != c->park_trace_cur_dir) {
        if (c->park_trace_hold < 2) {
            ++c->park_trace_hold;
            out->speed_cm_s = 0.0f; out->steer_deg = steer_out; out->yaw_rate_dps = 0.0f;
            out->explicit_yaw_rate = true; out->action = LOCAL_ACTION_STOP;
            return true;
        }
        c->park_trace_cur_dir = dir; c->park_trace_hold = 0;
    }
    float speed = c->cfg.park_speed_cm_s;
    /* Frenar antes de un cambio de sentido, para no pasarse. */
    if (turn > 0) {
        const float remain = hypotf(p.u_cm - c->park_trace_u[turn], p.w_cm - c->park_trace_w[turn]);
        const float tail = fmaxf(2.0f, 1.0f * remain + 1.0f);
        if (speed > tail) speed = tail;
    }
    c->park_trace_steer_deg = s_deg;
    out->speed_cm_s = (float)dir * speed;
    out->steer_deg = steer_out;
    out->yaw_rate_dps = out->speed_cm_s * tanf(out->steer_deg * DEG_TO_RAD) / pl->wheelbase_cm * RAD_TO_DEG;
    out->explicit_yaw_rate = false;
    out->action = dir > 0 ? LOCAL_ACTION_FORWARD : LOCAL_ACTION_REVERSE;
    return true;
}

#ifndef PARK_TWIN_ENABLED
#define PARK_TWIN_ENABLED 0
#endif
#ifndef PARK_TWIN_K_POS
#define PARK_TWIN_K_POS 0.35f
#endif
#ifndef PARK_TWIN_K_TH
#define PARK_TWIN_K_TH 0.35f
#endif
#define PARK_TWIN_JUMP_CM 3.0f
static void park_step(wro_controller_t *c, const local_lidar_point_t *points, size_t count, float dt,
                      const perception_t *per, wro_command_t *out)
{
    c->park_trace_dt = dt;
    const bool exiting = c->phase == WRO_PHASE_PARK_EXIT;
    const local_planner_config_t *pl = &c->cfg.planner;
    if (exiting && !c->park_init) {
        /* Arranca paralelo a la pared, que esta a un lado: ese lado dice el sentido de la ronda. */
        const int mirror = per->right_cm <= per->left_cm ? 1 : -1;
        c->direction = -mirror;
        c->park_course_deg = c->yaw.yaw_deg;
        c->park_pose.u_cm = -(0.5f * pl->chassis_length_cm - pl->chassis_rear_cm);
        c->park_pose.w_cm = c->cfg.start_lot_w_cm;
        c->park_pose.theta_deg = 0.0f;
        c->park_prev = PARK_MOVE_NONE;
        c->park_lost = 0; c->park_goal_hits = 0;
        c->park_rot_hold = 0;
        c->park_init = true;
    }
    stop_command(c, out);
    out->explicit_yaw_rate = true;
    if (!park_prepare(c, exiting ? PARK_GOAL_OUT : PARK_GOAL_IN)) {
        /* No hay maniobra posible con estas medidas: no se inventa una. */
        if (exiting) { c->phase = WRO_PHASE_RUN; c->aligned = false; c->travel_cm = 0.0f; }
        else c->phase = WRO_PHASE_DONE;
        return;
    }
    const int mirror = park_mirror(c);
    const park_loc_t loc = {.params = c->cfg.park_ackermann ? &g_ack_park_params : park_field_params(), .lidar_forward_cm = pl->lidar_forward_cm, .mirror = mirror};
    park_pose_t prior = c->park_pose;
    prior.theta_deg = -(float)mirror * imu_wrap_deg(c->yaw.yaw_deg - c->park_course_deg);
    const float th = prior.theta_deg * DEG_TO_RAD;
    prior.u_cm += c->scan_ds_cm * cosf(th);
    prior.w_cm += c->scan_ds_cm * sinf(th);
    park_fix_t fix;
    if (park_localize_track(&loc, points, count, &prior, &fix)) {
        /* Gemelo del coche: la odometria y el giroscopio predicen la pose (prior), el LiDAR la
         * corrige. Antes la pose era el ajuste del LiDAR a secas, con ~0,5-0,9 cm de ruido por
         * barrido, tanto como la holgura de la meta. Se mezcla con una ganancia fija: el ruido
         * de cada barrido se promedia y la dinamica entre barridos aporta lo que el LiDAR no. */
        if (PARK_TWIN_ENABLED && c->park_twin_ok && hypotf(fix.pose.u_cm - prior.u_cm, fix.pose.w_cm - prior.w_cm) < PARK_TWIN_JUMP_CM) {
            c->park_pose.u_cm = prior.u_cm + PARK_TWIN_K_POS * (fix.pose.u_cm - prior.u_cm);
            c->park_pose.w_cm = prior.w_cm + PARK_TWIN_K_POS * (fix.pose.w_cm - prior.w_cm);
            c->park_pose.theta_deg = imu_wrap_deg(prior.theta_deg + PARK_TWIN_K_TH * imu_wrap_deg(fix.pose.theta_deg - prior.theta_deg));
        } else {
            c->park_pose = fix.pose;
        }
        c->park_twin_ok = true;
        c->park_lost = 0;
        const float measured = imu_wrap_deg(c->yaw.yaw_deg + (float)mirror * fix.pose.theta_deg);
        c->park_course_deg = imu_wrap_deg(c->park_course_deg + 0.5f * imu_wrap_deg(measured - c->park_course_deg));
    } else {
        c->park_pose = prior;
        ++c->park_lost;
    }
    /* Sin pared ni topes a la vista: se sigue unos barridos con la pose de odometria y giroscopio
     * (precisa a corto plazo) antes de parar. Parar en cuanto se pierde (8 barridos) dejaba al coche
     * quieto con 30 grados en la boca del cajon, sin ver nada, para siempre: ccw 12 y 17 acababan
     * 'stalled' en parcial (medido). Quieto no vuelve a ver; moviendose si. */
    if (c->park_lost > PARK_LOST_MAX_SCANS) {   /* parado, no a ciegas */
        c->park_prev = PARK_MOVE_NONE;
        c->park_arc_remaining_cm = 0.0f;
        c->park_arc_count = c->park_arc_index = 0;
        if (c->park_lost > 120 && !exiting) c->phase = WRO_PHASE_DONE;
        return;
    }
    if (c->cfg.park_ackermann) {
        park_params_t relaxed;
        const park_params_t *params = &g_ack_park_params;
        if (!park_pose_free(params, &c->park_pose)) {
            /* La holgura es una precaucion al planear, no un muro: si la maniobra deja el
             * chasis dentro de ella (error de la pose medida, o un arco que la roza), negarse a
             * planificar deja al robot parado el resto de la ronda. Se reintenta sin holgura
             * solo para este barrido, para que pueda retroceder y salir. */
            relaxed = *params;
            relaxed.margin_cm = 0.0f;
            if (!park_pose_free(&relaxed, &c->park_pose)) {
                c->park_arc_remaining_cm = 0.0f; c->park_arc_count = 0; return;
            }
            params = &relaxed;
        }
        /* Entrada: el coche llega de la ronda por el carril. Primero se pone donde acabo la salida
         * (a la velocidad de busqueda: el carril esta libre) y desde ahi hace un solo plan, que es
         * la maniobra de la salida al reves. Entrar desde donde el LiDAR ve el cajon por primera
         * vez cuesta 4-8 repasadas y 15 s (medido); desde la pose de la salida, 3 y ~12 s. */
        if (!exiting && c->park_stage == 0) {
            c->park_stage = c->park_exit_valid ? 1 : 3;
            /* Sólo la entrada simétrica (la que arranca donde acabó la salida) usa el asentamiento
             * del servo y el metrado por rumbo: el tanteo de siempre y la demostración desde la
             * pista se quedan tal cual estaban (medido: cambiarlos deja el parqueo en parcial). */
            c->park_symmetric = c->park_exit_valid;
        }
        if (!exiting && c->park_stage == 1) {
            if (park_cruise_step(c, dt, out)) {
                c->park_stage = (PARK_RETRACE_ENABLED && c->park_trace_n > 8) ? 4 : 2;
                c->park_trace_k = (int)c->park_trace_n - 1; c->park_trace_cur_dir = 0; c->park_trace_hold = 0; c->park_trace_extra = 0;
                c->park_arc_count = 0;
                c->park_arc_index = 0;
                c->park_goal_hits = 0;
            }
            return;
        }
        /* El servo tarda ~0,2 s en pasar de un volante al otro (medido en el chasis: 80 grados a
         * 420 grados/s). Si se arranca a mover en el mismo barrido, el primer arco de cada cambio
         * de volante gira MENOS de lo que dice el plan y el desvio se acumula: en la salida al
         * reves, 12-15 arcos acababan 3 cm y 10 grados cruzados, el controlador abortaba el plan y
         * la correccion costaba una repasada mas. Se le da el volante con el coche quieto y asi el
         * arco que empieza es el del plan. Solo al entrar: la salida va sobrada (su meta es una
         * region) y no merece pagar 1,5 s de asentamientos; la entrada si. */
        if (!exiting && c->park_stage == 4) {
            if (park_retrace_step(c, params, out)) return;
            /* Fin del camino o desvio: el A* de siempre termina (y corrige) desde donde este. */
            c->park_stage = 3; c->park_arc_count = 0; c->park_arc_index = 0; c->park_goal_hits = 0;
            c->park_symmetric = false;
            out->speed_cm_s = 0.0f; out->steer_deg = 0.0f; out->yaw_rate_dps = 0.0f;
            out->explicit_yaw_rate = true; out->action = LOCAL_ACTION_STOP;
            return;
        }
        const bool entry_settle = !exiting && c->park_symmetric;
        if (entry_settle && c->park_settle > 0) {
            --c->park_settle;
            out->speed_cm_s = 0.0f;
            out->steer_deg = c->park_arc_steer_deg;
            out->yaw_rate_dps = 0.0f;
            out->explicit_yaw_rate = true;
            out->action = LOCAL_ACTION_STOP;
            return;
        }
        c->park_arc_remaining_cm -= fabsf(c->scan_ds_cm);
        /* La ejecucion "simetrica" (arcos metrados por el giroscopio, avance a ciegas y media
         * holgura) es solo para la entrada que arranca en la pose de la salida. El tanteo de
         * siempre (y la demostracion desde la pista, que no tiene salida) siguen igual: cambiarlo
         * ahi dejaba el parqueo en parcial (medido). */
        if (!exiting && c->park_stage == 2 && c->park_arc_count > 0 &&
            c->park_arc_index >= c->park_arc_count && c->park_arc_remaining_cm <= 0.3f)
            c->park_stage = 3; /* El plan inicial termino; las correcciones usan el tanteo. */
        const bool symmetric = !exiting && c->park_stage == 2;
        /* Tolerancia provisional para el cuantizado de la odometria. */
        const bool arc_turned = symmetric && fabsf(c->park_arc_dtheta_deg) > 1.0f &&
            fabsf(park_heading(c) - c->park_arc_theta0_deg) >= fabsf(c->park_arc_dtheta_deg) - 0.6f;
        if (c->park_arc_remaining_cm <= 0.3f || arc_turned) {
            /* La busqueda es discreta y sensible al paso del arco: con el volante a 40 grados,
             * un paso de 3,5 cm gira 12 grados y hay poses desde las que el A* no alcanza la
             * meta aunque la maniobra exista (el robot se quedaba parado a mitad de la salida).
             * Si el primer paso no encuentra plan se reintenta con uno mas fino; solo cuesta
             * cuando la busqueda ya habia fallado. */
            float step_cm = 3.5f;
            /* (Un paso de 5 cm probado con el volante a 40 se probo y sale peor: el chasis de
             * 22 cm ni siquiera sale del cajon. El arco corto es lo que deja afinar.) */
            if (c->park_arc_index >= c->park_arc_count) {
                /* Al entrar, primero la maniobra de tres tramos (colocarse, entrar girando,
                 * enderezar): dos cambios de sentido, como aparca una persona. Se intenta en CADA
                 * repaso mientras el coche siga en el carril: si solo se intentara la primera vez,
                 * una correccion a mitad de maniobra cae en el A* de arcos cortos y vuelve el
                 * tanteo (medido: 8 pasadas). Ya dentro del cajon manda el A*. */
                park_ackermann_result_t a = {0};
                bool s_plan = false;
                bool planned = false;
                /* Entrada desde la pose de la salida: un solo plan, sin la maniobra de tres tramos
                 * (desde ahi gastaria una repasada mas). Las correcciones, si hacen falta, vuelven
                 * al tanteo de siempre. */
                if (!exiting && c->park_stage == 2) {
                    a = park_ackermann_plan(params, &(park_ackermann_config_t)
                        {pl->wheelbase_cm, pl->steer_hard_deg, step_cm, c->park_arc_direction},
                        &c->park_pose, PARK_GOAL_IN);
                    if (!a.found) {
                        step_cm = 2.5f;
                        a = park_ackermann_plan(params, &(park_ackermann_config_t)
                            {pl->wheelbase_cm, pl->steer_hard_deg, step_cm, c->park_arc_direction},
                            &c->park_pose, PARK_GOAL_IN);
                    }
                    planned = a.found;
                }
                /* Solo desde el carril (aun fuera del cajon): la maniobra supone que el coche llega
                 * casi paralelo. Intentarla desde dentro, a 30-40 grados, produce maniobras que no
                 * convergen (medido: la ronda acababa a los 180 s con parqueo parcial). */
                if (!planned && !exiting && c->park_pose.w_cm > params->lot_width_cm - 1.0f)
                    s_plan = park_ackermann_entry_maneuver(params, &(park_ackermann_config_t)
                        {pl->wheelbase_cm, pl->steer_hard_deg, step_cm, c->park_arc_direction},
                        &c->park_pose, &a);
                if (!planned && !s_plan) {
                    a = park_ackermann_plan(params, &(park_ackermann_config_t)
                        {pl->wheelbase_cm, pl->steer_hard_deg, step_cm, c->park_arc_direction},
                        &c->park_pose, exiting ? PARK_GOAL_OUT : PARK_GOAL_IN);
                    if (!a.found) {
                        step_cm = 2.5f;
                        a = park_ackermann_plan(params, &(park_ackermann_config_t)
                            {pl->wheelbase_cm, pl->steer_hard_deg, step_cm, c->park_arc_direction},
                            &c->park_pose, exiting ? PARK_GOAL_OUT : PARK_GOAL_IN);
                    }
                }
                c->park_plan_found = a.found ? 1 : 0; c->park_plan_expanded = (int)a.expanded;
                c->park_plan_at_goal = a.at_goal ? 1 : 0; c->park_plan_count = (int)a.count;
                c->park_plan_step = (int)(step_cm * 10.0f);
                if (!a.found) { c->park_goal_hits = 0; return; }
                if (a.at_goal) {
                if (++c->park_goal_hits >= 3) {
                    c->phase = exiting ? WRO_PHASE_RUN : WRO_PHASE_DONE;
                    c->parked = !exiting;
                    if (exiting) {
                        c->course_deg = c->park_course_deg; c->aligned = true; c->travel_cm = 0.0f;
                        /* Donde acabo la salida: desde ahi se entra al final de la ronda. */
                        c->park_exit_pose = c->park_pose; c->park_exit_valid = true;
                    }
                    c->park_field_goal = 0;
                }
                    return;
                }
                c->park_goal_hits = 0;
                c->park_arc_count = a.count; c->park_arc_index = 0;
                c->park_plan_pose = c->park_pose;
                memcpy(c->park_arc_directions, a.directions, a.count * sizeof(a.directions[0]));
                memcpy(c->park_arc_steering, a.steering, a.count * sizeof(a.steering[0]));
            }
            const park_ackermann_config_t acfg = {pl->wheelbase_cm, pl->steer_hard_deg, step_cm, c->park_arc_direction};
            /* El plan se ejecuta a ciegas: lo que se vigila es que el chasis siga donde el plan
             * cree. El servo tarda ~0,2 s en invertir el volante, asi que los primeros arcos de
             * cada cambio de sentido giran menos que el modelo y el avance a ciegas se separa sin
             * que nada vaya mal: el margen es holgado y solo corta un desvio grande (choque). */
            if (symmetric && c->park_arc_index > 0)
                park_plan_advance(c, c->park_arc_directions[c->park_arc_index - 1],
                                  c->park_arc_steering[c->park_arc_index - 1], acfg.step_cm);
            const float dev = hypotf(c->park_pose.u_cm - c->park_plan_pose.u_cm,
                                     c->park_pose.w_cm - c->park_plan_pose.w_cm);
            if (symmetric && (dev > 6.0f ||
                fabsf(imu_wrap_deg(c->park_pose.theta_deg - c->park_plan_pose.theta_deg)) > 25.0f)) {
                c->park_arc_count = c->park_arc_index = 0;
                return;
            }
            c->park_arc_direction = c->park_arc_directions[c->park_arc_index];
            c->park_arc_steer_deg = -(float)mirror * c->park_arc_steering[c->park_arc_index] * acfg.steer_max_deg;
            /* Al entrar, la comprobacion del arco se hace con media holgura: el camino ya se
             * planeo con 1 cm y la pose del LiDAR dentro del cajon se mueve medio centimetro, asi
             * que la holgura de planeo aqui solo servia para abortar planes buenos (cada abort,
             * una repasada mas). Sin ninguna holgura el chasis se queda tan justo que el juez lo
             * da por parqueo parcial: media es el punto que midio 3 repasadas sin contactos. */
            park_params_t entry_check;
            const park_params_t *check = params;
            if (symmetric) {
                /* Nunca mas estricto que el plan: si el coche esta a menos de la holgura de planeo, el plan
                 * se hizo con margen 0 (arriba) y comprobar el arco con 0,5 lo rechazaba cada barrido para
                 * siempre (ccw 12: parado 20 s con un plan valido, 'stalled' y parqueo parcial). */
                entry_check = *params; entry_check.margin_cm = fminf(params->margin_cm, 0.5f); check = &entry_check;
            }
            if (!park_ackermann_arc_free(check, &acfg, &c->park_pose, c->park_arc_direction,
                                         c->park_arc_steering[c->park_arc_index])) {
                c->park_arc_count = c->park_arc_index = 0;
                return;
            }
            ++c->park_arc_index;
            c->park_arc_remaining_cm = acfg.step_cm;
            /* Avance exacto: MIC recorre acfg.step_cm con el encoder y queda parado en el punto (sin la cola de
             * 0,6-1 cm que dejaba el rodar tras la orden). El arco sigue acabando por odometria/giroscopio. */
            if (c->cfg.park_exact_moves) {
                /* Solo tramos rectos (el volante a 0): los arcos con volante se cortan por rumbo del giroscopio. Los arcos
                 * rectos seguidos del mismo sentido forman un solo avance de n x paso, sin parar entre ellos. */
                const int idx = (int)c->park_arc_index;   /* ya incrementado: el arco actual es idx-1 */
                const int st = c->park_arc_steering[idx - 1];
                if (st != 0) {
                    c->park_move_cm = 0.0f;
                } else if (!(idx >= 2 && c->park_arc_steering[idx - 2] == 0 && c->park_arc_directions[idx - 2] == c->park_arc_direction && c->park_move_cm != 0.0f)) {
                    int n = 0;
                    for (int j = idx - 1; j < (int)c->park_arc_count && c->park_arc_steering[j] == 0 && c->park_arc_directions[j] == c->park_arc_direction; ++j) ++n;
                    c->park_move_cm = (float)c->park_arc_direction * (float)n * acfg.step_cm;
                    ++c->move_id;
                }
            }
            /* El arco se corta tambien por rumbo, al entrar: el giroscopio da el giro exacto del
             * plan (un arco de 3,5 cm a volante lleno son 9 grados) mientras que la distancia la
             * mide el LiDAR con 1 cm de ruido y lo deja 0,2-0,3 cm pronto; en 15 arcos eso era
             * 3 cm y 10 grados de morro, y el plan se abortaba antes de enderezar. La salida va
             * sobrada (su meta es una region) y se queda con la distancia sola: asi tarda 8-9 s. */
            c->park_arc_theta0_deg = park_heading(c);
            const float arc_steer = (float)c->park_arc_steering[c->park_arc_index - 1] * acfg.steer_max_deg;
            c->park_arc_dtheta_deg = (float)c->park_arc_direction * acfg.step_cm *
                tanf(arc_steer * DEG_TO_RAD) / pl->wheelbase_cm * RAD_TO_DEG;
            /* Volante distinto del arco anterior: se pone quieto antes de mover (ver arriba). El
             * servo tarda ~0,1 s por cada 40 grados, asi que un cambio pequeno (0 <-> lleno) se
             * asienta en un barrido y la inversion completa (lleno a lleno, 80 grados) en dos. */
            const int prev_steer = c->park_arc_index >= 2 ? c->park_arc_steering[c->park_arc_index - 2] : 0;
            if (entry_settle && c->park_arc_steering[c->park_arc_index - 1] != prev_steer) {
                c->park_settle = fabsf((float)(c->park_arc_steering[c->park_arc_index - 1] - prev_steer)) > 1.0f ? 2 : 1;
                out->speed_cm_s = 0.0f;
                out->steer_deg = c->park_arc_steer_deg;
                out->yaw_rate_dps = 0.0f;
                out->explicit_yaw_rate = true;
                out->action = LOCAL_ACTION_STOP;
                return;
            }
        }
        /* Velocidad del arco: la de la maniobra, sin el tope de 4 cm/s que habia. El final de un
         * arco NO es una parada: es donde se vuelve a planificar y se encadena el siguiente, asi
         * que frenar progresivamente en cada uno (el codigo anterior bajaba a 1 cm/s el ultimo
         * tramo) solo gastaba tiempo: la salida tardaba 21-24 s con 21,9 s en movimiento y 0,4 s
         * planeando. Solo se limita en el ultimo centimetro, por si hay que cortar en seco. */
        float arc_speed = c->cfg.park_speed_cm_s;
        if (c->park_arc_remaining_cm < 1.0f) {
            const float tail = fmaxf(3.0f, 8.0f * c->park_arc_remaining_cm);
            if (arc_speed > tail) arc_speed = tail;
        }
        if (dt > 0.0f && c->park_arc_remaining_cm / dt < arc_speed)
            arc_speed = c->park_arc_remaining_cm / dt;
        out->speed_cm_s = c->park_arc_direction * arc_speed;
        out->steer_deg = c->park_arc_steer_deg;
        out->yaw_rate_dps = out->speed_cm_s * tanf(out->steer_deg * DEG_TO_RAD) / pl->wheelbase_cm * RAD_TO_DEG;
        out->explicit_yaw_rate = false;
        out->action = c->park_arc_direction > 0 ? LOCAL_ACTION_FORWARD : LOCAL_ACTION_REVERSE;
        if (c->cfg.park_exact_moves && c->park_move_cm != 0.0f) { out->move_cm = c->park_move_cm; out->move_id = c->move_id; }
        if (exiting) park_trace_record(c, pl->wheelbase_cm);
        return;
    }
    const park_decision_t d = park_field_decide(&c->park_pose, c->park_prev);
    c->park_prev = d.move;
    if (d.at_goal) {
        if (++c->park_goal_hits >= 3) {
            if (exiting) {
                /* Fuera del cajon y casi paralelo: sigue la ronda con el rumbo de la pared. */
                c->phase = WRO_PHASE_RUN;
                c->course_deg = c->park_course_deg;
                c->aligned = true;
                c->travel_cm = 0.0f;
                c->net_cm = 0.0f; c->progress_best_cm = 0.0f; c->stuck_s = 0.0f;
                c->park_field_goal = 0;
            } else {
                c->phase = WRO_PHASE_DONE;
                c->parked = true;
            }
            return;
        }
        return;                          /* en la meta: quieto hasta confirmarlo */
    }
    c->park_goal_hits = 0;
    if (d.move == PARK_MOVE_NONE) {
        /* Fuera del campo: atras si se paso del cajon, adelante si aun no llega. */
        park_decision_t d2 = d;
        if (c->park_pose.u_cm > 30.0f) d2.move = PARK_MOVE_BACK;
        else if (c->park_pose.u_cm < -34.0f) d2.move = PARK_MOVE_FORWARD;
        d2.target_theta_deg = 0.0f;
        park_command(c, &d2, c->park_pose.theta_deg, out);
        return;
    }
    park_command(c, &d, c->park_pose.theta_deg, out);
}

/* Buscando el cajon por la recta de salida, pegado a la pared exterior. */
static bool park_search_step(wro_controller_t *c, const local_lidar_point_t *points, size_t count, float dt)
{
    (void)dt;
    const local_planner_config_t *pl = &c->cfg.planner;
    if (c->scan_ds_cm > 0.0f) c->park_search_cm += c->scan_ds_cm;
    if (!park_prepare(c, PARK_GOAL_IN)) { c->phase = WRO_PHASE_FINISH; return false; }
    const int mirror = park_mirror(c);
    const park_loc_t loc = {.params = c->cfg.park_ackermann ? &g_ack_park_params : park_field_params(), .lidar_forward_cm = pl->lidar_forward_cm, .mirror = mirror};
    /* Solo con el morro casi paralelo a la pared: de lado, un tope lejano se confunde con otro. */
    const float yaw_to_wall = imu_wrap_deg(c->yaw.yaw_deg - c->course_deg);
    if (fabsf(yaw_to_wall) > 30.0f) return false;
    park_fix_t fix;
    if (park_localize_acquire(&loc, points, count, 20.0f, &fix) && fix.pose.u_cm >= -36.0f &&
        fix.pose.u_cm <= 34.0f && fabsf(fix.pose.theta_deg) < 35.0f) {
        c->park_pose = fix.pose;
        c->park_course_deg = imu_wrap_deg(c->yaw.yaw_deg + (float)mirror * fix.pose.theta_deg);
        c->park_prev = PARK_MOVE_NONE;
        c->park_lost = 0; c->park_goal_hits = 0;
        c->park_rot_hold = 0;
        c->phase = WRO_PHASE_PARKING;
        return true;
    }
    if (c->park_search_cm > c->cfg.park_search_max_cm) {
        c->phase = WRO_PHASE_FINISH;     /* sin cajon a la vista: parar como sin parqueo */
        c->finish_travel_cm = 0.0f;
        c->finish_seen_wall = false;
    }
    return false;
}

/* Cuanto hay que desplazarse de lado para dejar el letrero del lado que toca: >0 hacia la derecha del recorrido,
 * <0 hacia la izquierda, 0 si ya esta bien colocado. Rojo se pasa por la derecha (el letrero a la izquierda). */
static float sidestep_shift_needed(const wro_controller_t *c, int sign, float lat)
{
    const local_planner_config_t *pl = &c->cfg.planner;
    const float need = 2.5f + 0.5f * pl->chassis_width_cm + c->cfg.sidestep_margin_cm;
    const int side = c->signs[sign].color == 1 ? pl->pass_side_red : pl->pass_side_green;   /* +1: robot a la derecha */
    if (side > 0) return lat > -need ? lat + need : 0.0f;
    if (side < 0) return lat < need ? -(need - lat) : 0.0f;
    return 0.0f;
}

/* Primer letrero (de los que se siguen con el giroscopio) por delante que esta del lado que no toca, en el marco del
 * carril: `lat` = cuanto queda a la derecha del robot y `fwd` = cuanto por delante, medidos sobre el rumbo del
 * recorrido. Los ya bien colocados se saltan. Si otro letrero esta pegado al robot o entre el y este (el desplazamiento
 * de lado lo chocaria) no se elige ninguno: se espera a pasarlo. */
static int sidestep_pick_sign(const wro_controller_t *c, float *lat, float *fwd)
{
    const float d = imu_wrap_deg(c->yaw.yaw_deg - c->course_deg) * DEG_TO_RAD;
    const float cd = cosf(d), sd = sinf(d);
    const int n = (int)(sizeof(c->signs) / sizeof(c->signs[0]));
    int best = -1;
    float best_f = 1.0e9f;
    for (int i = 0; i < n; ++i) {
        if (!c->signs[i].valid || (c->signs[i].color != 1 && c->signs[i].color != 2)) continue;
        const float x = c->signs[i].x_cm, f = c->signs[i].forward_cm;
        const float l = x * cd + f * sd, fw = -x * sd + f * cd;
        if (fw < -8.0f || fw > 50.0f || fabsf(l) > 90.0f) continue;
        if (fabsf(sidestep_shift_needed(c, i, l)) < 3.0f) continue;
        if (fw < best_f) { best_f = fw; best = i; *lat = l; *fwd = fw; }
    }
    if (best < 0) return -1;
    for (int i = 0; i < n; ++i) {
        if (i == best || !c->signs[i].valid) continue;
        const float x = c->signs[i].x_cm, f = c->signs[i].forward_cm;
        const float l = x * cd + f * sd, fw = -x * sd + f * cd;
        if (fw > -14.0f && fw < best_f && fabsf(l) < 100.0f) return -1;
    }
    return best;
}

/* Cambio de lado con giros en el sitio. Se dispara cuando el planificador lleva `sidestep_after_s` sin ruta, el
 * morro esta casi paralelo al carril y un letrero por delante esta mal colocado: gira 90 grados hacia el lado que
 * falta, avanza de lado hasta dejarlo bien y vuelve a girar al rumbo del recorrido. */
static bool sidestep_update(wro_controller_t *c, uint8_t action, float dt, wro_command_t *out)
{
    const local_planner_config_t *pl = &c->cfg.planner;
    if (c->sidestep_cooldown_s > 0.0f) c->sidestep_cooldown_s -= dt;
    const float front = pl->chassis_length_cm - pl->chassis_rear_cm;
    const float far = front > pl->chassis_rear_cm ? front : pl->chassis_rear_cm;
    const float radius = hypotf(far, 0.5f * pl->chassis_width_cm) + c->cfg.pivot_margin_cm;
    const bool fits = local_planner_axle_clearance_cm() >= radius;
    const float to_course = imu_wrap_deg(c->course_deg - c->yaw.yaw_deg);

    if (c->sidestep_phase == 0) {
        const bool no_route = c->debug.pass_cm < 0.0f && (action == LOCAL_ACTION_STOP || c->planner_state.escaping);
        if (!no_route) c->sidestep_wait_s = 0.0f;
        if (c->sidestep_cooldown_s > 0.0f || c->corner_active || fabsf(to_course) > 25.0f || !fits) return false;
        float lat = 0.0f, fwd = 0.0f;
        const int sign = sidestep_pick_sign(c, &lat, &fwd);
        if (sign < 0) return false;
        const float shift = sidestep_shift_needed(c, sign, lat);
        if (fabsf(shift) < 3.0f || fabsf(shift) > 110.0f) return false;
        /* Dos disparadores: atascado (sin ruta durante sidestep_after_s), o PREVISTO: el cambio de lado que pide el
         * letrero no cabe en la recta que queda hasta el (mas de 0,8 cm de lado por cada cm de avance libre). */
        const float room = fwd - front - 4.0f;
        const bool predicted = c->travel_cm >= 30.0f && fabsf(shift) >= 12.0f && fwd >= 16.0f && fwd <= 48.0f && fabsf(shift) > 1.5f * (room > 5.0f ? room : 5.0f);
        if (no_route) c->sidestep_wait_s += dt;
        if (!predicted && !(no_route && c->sidestep_wait_s >= c->cfg.sidestep_after_s)) return false;
        c->sidestep_sign = sign;
        c->sidestep_side = shift > 0.0f ? 1 : -1;
        c->sidestep_need_cm = fabsf(shift) + 2.0f;
        c->sidestep_slid_cm = 0.0f;
        c->sidestep_phase = 1;
        c->pivoting = false;
        c->sidestep_wait_s = 0.0f;
        c->sidestep_phase_s = 0.0f;
    }
    c->sidestep_phase_s += dt;

    /* Rumbo objetivo de la fase: de lado (curso +-90) al salir y al avanzar, el del curso al volver. */
    const float side_yaw = imu_wrap_deg(c->course_deg + (float)c->sidestep_side * 90.0f);
    float target = c->sidestep_phase == 3 ? c->course_deg : side_yaw;
    float err = imu_wrap_deg(target - c->yaw.yaw_deg);
    out->explicit_yaw_rate = true;
    out->steer_deg = 0.0f;
    /* Seguro: ninguna fase dura para siempre (un giro que no cabe se abandona y sigue el planificador normal). */
    if ((c->sidestep_phase != 2 && c->sidestep_phase_s > 8.0f) || c->sidestep_phase_s > 16.0f) {
        c->sidestep_phase = 0;
        c->sidestep_wait_s = 0.0f;
        c->sidestep_cooldown_s = 3.0f * c->cfg.sidestep_cooldown_s;
        c->planner_state.escaping = false;
        return false;
    }
    if (c->sidestep_phase == 1) {
        if (fabsf(err) < 6.0f) { c->sidestep_phase = 2; c->sidestep_phase_s = 0.0f; }
    }
    if (c->sidestep_phase == 2) {
        /* Lo que falta: el letrero que lo motiva visto desde el rumbo del recorrido; si se perdio, la distancia avanzada. */
        bool done = c->sidestep_slid_cm >= c->sidestep_need_cm + 6.0f;
        if (c->signs[c->sidestep_sign].valid) {
            const float d = imu_wrap_deg(c->yaw.yaw_deg - c->course_deg) * DEG_TO_RAD;
            const float lat = c->signs[c->sidestep_sign].x_cm * cosf(d) + c->signs[c->sidestep_sign].forward_cm * sinf(d);
            if (fabsf(sidestep_shift_needed(c, c->sidestep_sign, lat)) < 1.0f) done = true;
        }
        /* Hacia donde avanza (de lado) debe haber sitio: la huella recta. */
        if (!local_planner_arc_clear(&c->map, pl, 0.0f, 10.0f)) done = true;
        /* Y que quepa girar de vuelta: se para antes de que el chasis ya no pueda girar en el sitio. */
        if (local_planner_axle_clearance_cm() < radius + 3.0f) done = true;
        if (done) { c->sidestep_phase_s = 0.0f; c->sidestep_phase = 3; target = c->course_deg; err = imu_wrap_deg(target - c->yaw.yaw_deg); }
    }
    if (c->sidestep_phase == 3 && fabsf(err) < 6.0f) {
        c->sidestep_phase = 0;
        c->sidestep_wait_s = 0.0f;
        c->sidestep_cooldown_s = c->cfg.sidestep_cooldown_s;
        c->planner_state.escaping = false;
        return false;
    }
    if (c->sidestep_phase == 2) {
        const float v = c->cfg.sidestep_speed_cm_s;
        out->speed_cm_s = v;
        float rate = 2.0f * err;
        if (rate > 20.0f) rate = 20.0f;
        if (rate < -20.0f) rate = -20.0f;
        out->yaw_rate_dps = rate;
        out->action = LOCAL_ACTION_FORWARD;
        c->sidestep_slid_cm += v * dt;
    } else {
        float rate = fabsf(err) * 2.5f;
        if (rate < 30.0f) rate = 30.0f;
        if (rate > c->cfg.pivot_rot_dps) rate = c->cfg.pivot_rot_dps;
        out->speed_cm_s = 0.0f;
        out->yaw_rate_dps = err > 0.0f ? rate : -rate;
        out->action = err > 0.0f ? LOCAL_ACTION_RIGHT_HARD : LOCAL_ACTION_LEFT_HARD;
    }
    return true;
}

/* Atasco con el morro girado de la pista (por ejemplo de cara a una pared tras una esquiva): sin ruta
 * hacia delante, se gira en el sitio hacia el rumbo del recorrido. Solo si el chasis cabe girando (la
 * esquina mas lejana del eje, mas un margen, a la distancia libre del obstaculo mas cercano). */
static void pivot_recovery(wro_controller_t *c, uint8_t action, float dt, wro_command_t *out)
{
    const local_planner_config_t *pl = &c->cfg.planner;
    const bool no_route = c->debug.pass_cm < 0.0f && (action == LOCAL_ACTION_STOP || c->planner_state.escaping);
    if (!no_route && !c->pivoting) { c->pivot_wait_s = 0.0f; return; }
    const float err = imu_wrap_deg(c->course_deg - c->yaw.yaw_deg);      /* > 0: girar a la derecha (yaw crece) */
    const float front = pl->chassis_length_cm - pl->chassis_rear_cm;
    const float far = front > pl->chassis_rear_cm ? front : pl->chassis_rear_cm;
    const float radius = hypotf(far, 0.5f * pl->chassis_width_cm) + c->cfg.pivot_margin_cm;
    const bool fits = local_planner_axle_clearance_cm() >= radius;
    if (!c->pivoting) {
        c->pivot_wait_s += dt;
        if (c->pivot_wait_s < c->cfg.pivot_after_s || fabsf(err) < 15.0f || !fits) return;
        c->pivoting = true;
    }
    if (fabsf(err) < 8.0f || !fits || (!no_route && c->debug.pass_cm >= 0.0f)) {
        c->pivoting = false;
        c->pivot_wait_s = 0.0f;
        return;
    }
    float rate = fabsf(err) * 2.0f;
    if (rate < 30.0f) rate = 30.0f;
    if (rate > c->cfg.pivot_rot_dps) rate = c->cfg.pivot_rot_dps;
    out->speed_cm_s = 0.0f;
    out->steer_deg = 0.0f;
    out->explicit_yaw_rate = true;
    out->yaw_rate_dps = err > 0.0f ? rate : -rate;
    out->action = err > 0.0f ? LOCAL_ACTION_RIGHT_HARD : LOCAL_ACTION_LEFT_HARD;
}

void wro_controller_scan(wro_controller_t *c, const local_lidar_point_t *points, size_t count,
                         uint64_t t_us, wro_command_t *out)
{
    if (!c || !out) return;
    memset(out, 0, sizeof(*out));
    /* Ultimo recurso: si lleva mucho rato sin avanzar (un paso que el chasis no puede
     * trazar en el sentido de la ultima vuelta), se suspende un trecho la regla del
     * lado de paso. Puede costar una infraccion, pero no el tiempo de la ronda. */
    local_planner_config_t effective = c->cfg.planner;
    if (c->relaxed) effective.enforce_color_side = false;
    effective.lane_half_cm = 0.5f * c->cfg.lane_width_cm;
    const local_planner_config_t *pc = &effective;
    const float dt = c->last_scan_us && t_us > c->last_scan_us
        ? (float)(t_us - c->last_scan_us) * 1.0e-6f : pc->cycle_s;
    c->last_scan_us = t_us;
    /* Avance de este ciclo: lo que midieron los encoders desde el escaneo anterior si siguen llegando muestras; si no
     * (sin encoders, o se cayo su lectura), velocidad ordenada x tiempo como antes. */
    c->odom_in_use = c->cfg.odometry_enabled && c->odom_have && t_us >= c->odom_last_us && t_us - c->odom_last_us < 300000u;
    c->scan_ds_cm = c->odom_in_use ? c->odom_accum_cm : c->last_speed_cm_s * dt;
    c->odom_accum_cm = 0.0f;

    arena_step(c, points, count);

    local_camera_object_t objects[LOCAL_MAP_MAX_CAMERA_OBJECTS];
    size_t object_count = 0;
    uint8_t camera_action = LOCAL_ACTION_FORWARD;
    if (c->camera_seen) {
        object_count = c->camera.object_count < LOCAL_MAP_MAX_CAMERA_OBJECTS
            ? c->camera.object_count : LOCAL_MAP_MAX_CAMERA_OBJECTS;
        for (size_t i = 0; i < object_count; ++i) objects[i] = c->camera.objects[i];
        if (c->camera.valid) camera_action = c->camera.action;
    }
    static local_lidar_point_t moved[LOCAL_MAP_MAX_POINTS];
    float lead_deg = 0.0f;
    c->map_lead_cm = 0.0f; c->map_lead_deg = 0.0f;
    if (c->cfg.motion_comp && count <= LOCAL_MAP_MAX_POINTS && c->phase != WRO_PHASE_IDLE) {
        lead_deg = compensate_motion(c, pc, points, count, moved);
        c->map_lead_deg = lead_deg;
        c->map_lead_cm = (c->odom_in_use ? c->odom.v_cm_s : c->last_speed_cm_s) * c->cfg.latency_s;
        points = moved;
        effective.deskew = false;     /* ya incluido arriba */
    }
    const local_imu_sample_t imu = {.valid = c->yaw.has_last, .yaw_deg = imu_wrap_deg(c->yaw.yaw_deg + lead_deg),
        .timestamp_us = c->last_imu_us, .has_rate = true, .yaw_rate_dps = c->yaw.rate_dps};
    local_obstacle_map_update_cfg(&c->map, pc, points, count, t_us,
        c->camera_seen ? objects : NULL, object_count,
        c->camera.received_timestamp_us, &imu);
    c->live_colored = c->map.colored_count;
    /* Solo se proyecta en campo cuando existe pose; el mapa no interviene en
     * las decisiones del planificador y se reinicia al inicializar la mision. */
    if (c->arena.valid && c->arena.rms_cm <= 3.5f && c->phase != WRO_PHASE_IDLE) {
        /* Los puntos ya estan expresados en la pose adelantada por compensate_motion,
         * independientemente de la opcion de adelanto del mapa del planificador. */
        arena_pose_t tp = c->arena.pose;
        const float th = tp.theta_deg * DEG_TO_RAD;
        tp.x_cm += c->map_lead_cm * sinf(th);
        tp.z_cm += c->map_lead_cm * cosf(th);
        tp.theta_deg = imu_wrap_deg(tp.theta_deg + c->map_lead_deg);
        trace_map_update(&c->trace_map, points, count, &c->map, &tp, pc->lidar_forward_cm);
        /* Objetos y plaza: cada 10 barridos (0,05 ms en PC). El espaciado entre topes es el largo del cajon mas el grosor de un tope. */
        if (c->trace_map.scans % 10u == 0u) {
            c->trace_map.spot_spacing_cm = (c->cfg.park_lot_length_cm > 0.0f ? c->cfg.park_lot_length_cm : 1.5f * pc->chassis_length_cm) + 2.0f;
            trace_map_refresh(&c->trace_map);
        }
        /* Diagnóstico experimental: una sola detección magenta inicia la búsqueda.
         * Se usa la pose disponible al recibirla (no la pose exacta de captura).
         * El resultado NO se utiliza para dirigir o aparcar. */
        const bool retry_trace_camera = !c->trace_park_seen ||
            (c->trace_park_result.status != TRACE_PARK_CANDIDATE &&
             t_us >= c->trace_park_observation_us && t_us-c->trace_park_observation_us >= 5000000u);
        if (retry_trace_camera && c->phase == WRO_PHASE_RUN && c->camera.valid &&
            c->map.camera_fresh && c->camera.received_timestamp_us <= t_us &&
            (!c->trace_park_seen || c->camera.received_timestamp_us > c->trace_park_observation_us)) {
            for (size_t i=0; i<c->camera.object_count && i<LOCAL_MAP_MAX_CAMERA_OBJECTS; ++i) {
                const local_camera_object_t *o=&c->camera.objects[i];
                if (!o->valid || o->color!=3 || !isfinite(o->distance_cm) || o->distance_cm<=0 || !isfinite(o->bearing_deg)) continue;
                trace_parking_observation_t *v=&c->trace_park_observation;
                v->camera_pose=c->arena.pose;
                v->camera_forward_cm=pc->camera_forward_cm; v->camera_lateral_cm=0.0f;
                v->min_bearing_deg=o->bearing_deg-10.0f; v->max_bearing_deg=o->bearing_deg+10.0f;
                v->distance_cm=o->distance_cm;
                v->spacing_cm=(c->cfg.park_lot_length_cm>0.0f ? c->cfg.park_lot_length_cm : 1.5f*pc->chassis_length_cm)+2.0f;
                c->trace_park_seen=true;
                c->trace_park_observation_us=c->camera.received_timestamp_us;
                memset(&c->trace_park_result,0,sizeof(c->trace_park_result));
                break;
            }
        }
        if (c->trace_park_seen && c->trace_map.scans%10u==0u) {
            c->trace_park_result=trace_parking_estimate(&c->trace_map,&c->trace_park_observation);
            if(c->trace_park_result.status==TRACE_PARK_CANDIDATE){
                if(c->trace_park_has_memory && !trace_parking_consistent(&c->trace_park_result,&c->trace_park_first_candidate,8.0f)) {
                    /* No sustituir un recuerdo con una pareja incompatible. Ninguna
                     * de las dos queda confirmada por esta comparación. */
                    c->trace_park_result.status=TRACE_PARK_AMBIGUOUS;
                } else {
                    if(!c->trace_park_has_memory)c->trace_park_first_candidate=c->trace_park_result;
                    c->trace_park_last_candidate=c->trace_park_result;
                    c->trace_park_has_memory=true;
                }
            }
        }
    }

    perception_t per;
    /* Rumbo del carril respecto al morro, segun el controlador. Durante una esquina
     * o la media vuelta el carril todavia no es el que se mide. */
    const float lane_delta = (c->aligned && !c->corner_active && c->phase != WRO_PHASE_UTURN)
        ? imu_wrap_deg(c->course_deg - (c->yaw.yaw_deg + lead_deg)) : 0.0f;
    analyze(points, count, c->cfg.lane_width_cm, fabsf(lane_delta) < 45.0f ? lane_delta : 0.0f, &per);

    if (c->phase == WRO_PHASE_IDLE || c->phase == WRO_PHASE_DONE || c->phase == WRO_PHASE_FAULT) {
        stop_command(c, out);
        fill_status(c, &per, out);
        return;
    }

    if (c->phase == WRO_PHASE_PARK_EXIT || c->phase == WRO_PHASE_PARKING) {
        park_step(c, points, count, dt, &per, out);
        c->last_speed_cm_s = out->speed_cm_s;
        fill_status(c, &per, out);
        return;
    }

    c->travel_cm += c->scan_ds_cm;
    c->net_cm += c->scan_ds_cm;
    if (c->phase == WRO_PHASE_RUN && c->cfg.stuck_relax_s > 0.0f) {
        if (c->net_cm > c->progress_best_cm + 20.0f) { c->progress_best_cm = c->net_cm; c->stuck_s = 0.0f; }
        else c->stuck_s += dt;
        if (!c->relaxed && c->stuck_s >= c->cfg.stuck_relax_s) {
            c->relaxed = true;
            c->relax_from_cm = c->net_cm;
            c->stuck_s = 0.0f;
        } else if (c->relaxed && c->net_cm - c->relax_from_cm >= c->cfg.stuck_relax_cm) {
            c->relaxed = false;
            c->progress_best_cm = c->net_cm;
            c->stuck_s = 0.0f;
        }
    }
    if (!c->aligned) {
        /* Orientacion inicial: las paredes del carril dan el rumbo del recorrido. */
        c->course_deg = imu_wrap_deg(c->yaw.yaw_deg + (per.align_valid ? per.align_deg : 0.0f));
        c->aligned = true;
    }
    if (!c->corner_active && c->phase != WRO_PHASE_UTURN && per.align_valid &&
        per.front_cm > 70.0f && c->cfg.wall_align_gain > 0.0f) {
        const float err = imu_wrap_deg(c->yaw.yaw_deg + per.align_deg - c->course_deg);
        /* Solo con el morro casi paralelo al carril: torcido por una esquiva, las
         * paredes se ven en diagonal y arrastrarian el rumbo del recorrido. */
        if (fabsf(err) < c->cfg.wall_align_max_deg &&
            fabsf(imu_wrap_deg(c->yaw.yaw_deg - c->course_deg)) < c->cfg.wall_align_heading_deg)
            c->course_deg = imu_wrap_deg(c->course_deg + c->cfg.wall_align_gain * err);
    }
    track_signs(c, dt);
    {
        float xf[24];
        uint8_t colors[12];
        size_t n = 0;
        for (size_t i = 0; i < sizeof(c->signs) / sizeof(c->signs[0]); ++i) {
            if (!c->signs[i].valid || c->signs[i].forward_cm < -10.0f || c->signs[i].stale_cm > 150.0f) continue;
            xf[2 * n] = c->signs[i].x_cm; xf[2 * n + 1] = c->signs[i].forward_cm;
            colors[n++] = c->signs[i].color;
        }
        local_obstacle_map_add_memory(&c->map, pc, xf, colors, n);
    }
    if (c->cfg.arena_signs_enabled) world_signs_step(c, pc); else lap_memory_step(c, pc);

    /* ------------------------------------------------------------ esquinas */
    if (c->phase == WRO_PHASE_RUN) {
        if (c->corner_active && c->travel_cm > c->cfg.corner_abort_cm &&
            fabsf(imu_wrap_deg(c->course_deg - c->yaw.yaw_deg)) > c->cfg.corner_done_deg) {
            /* Una esquina de verdad se gira en menos de un metro. Esta no termino: era una esquiva muy
             * abierta tomada por esquina. Se deshace el giro del rumbo del recorrido. */
            c->corner_active = false;
            c->course_deg = imu_wrap_deg(c->course_deg - (float)c->corner_side * 90.0f);
            c->travel_cm = 0.0f;
        }
        if (c->corner_active) {
            if (fabsf(imu_wrap_deg(c->course_deg - c->yaw.yaw_deg)) <= c->cfg.corner_done_deg) {
                c->corner_active = false;
                ++c->corners_done;
                c->travel_cm = 0.0f;
                if (c->corners_done == 4) c->last_sign_color = 0; /* la regla mira la 2.a vuelta */
                if (c->corners_done == ((int)c->cfg.laps - 1) * 4 && !c->turnaround_decided) {
                    c->turnaround_decided = true;
                    /* Se decide aqui, con el ultimo letrero de la 2.a vuelta, pero la media
                     * vuelta espera a estar dentro de la recta de salida: la vuelta 2 no
                     * termina hasta salir de la esquina, y dar la vuelta antes es conducir
                     * al reves una vuelta que aun cuenta. */
                    if (c->cfg.turnaround_enabled && c->last_sign_color == c->cfg.reverse_on_color &&
                        c->cfg.reverse_on_color != 0) {
                        c->uturn_pending = true;
                        c->uturn_travel_cm = 0.0f;
                        c->uturn_wall_seen = false;
                    }
                }
                if (c->corners_done >= (int)c->cfg.laps * 4) {
                    c->phase = c->cfg.parking_enabled ? WRO_PHASE_PARK_SEARCH : WRO_PHASE_FINISH;
                    c->park_search_cm = 0.0f;
                    c->finish_travel_cm = 0.0f;
                    c->finish_seen_wall = false;
                }
            }
        } else if (per.front_wall && c->sidestep_phase == 0 &&
                   fabsf(imu_wrap_deg(c->yaw.yaw_deg - c->course_deg)) <= c->cfg.corner_max_heading_deg) {
            /* Con el morro girado mas de 30 grados del carril, la pared interior de enfrente es la del
             * propio carril, no una esquina: sin esta condicion una esquiva hacia ella cambia el rumbo. */
            const local_planner_config_t *pl = &c->cfg.planner;
            const float radius = pl->wheelbase_cm / tanf(pl->steer_hard_deg * DEG_TO_RAD);
            const float trigger = 0.5f * c->cfg.lane_width_cm + radius - pl->lidar_forward_cm +
                                  c->cfg.corner_trigger_extra_cm + (c->race_active && c->race_corner_clear && c->corners_done < (int)c->cfg.laps * 4 - 1 ? c->cfg.race_corner_extra_cm : 0.0f);
            const bool left_open = per.left_cm > c->cfg.open_side_cm;
            const bool right_open = per.right_cm > c->cfg.open_side_cm;
            int side = 0;
            if (right_open && !left_open) side = 1;
            else if (left_open && !right_open) side = -1;
            if (per.front_cm <= trigger && side != 0 && c->travel_cm >= c->cfg.corner_min_travel_cm && arena_corner_ok(c) &&
                (c->direction == 0 || c->direction == side)) {
                c->direction = side;
                c->corner_active = true;
                c->corner_side = side;
                c->course_deg = imu_wrap_deg(c->course_deg + (float)side * 90.0f);
                c->travel_cm = 0.0f;
            }
        }
        /* Respaldo: si el muro frontal no se llego a ver como plano (el robot venia
         * torcido de esquivar), el giroscopio delata la esquina: no se gira 60 grados
         * hacia el lado de la ronda en una recta. */
        /* Durante un desplazamiento de lado el morro esta a 90 grados del carril a proposito: no es una esquina. */
        if (!c->corner_active && c->sidestep_phase == 0 && c->direction != 0 && c->cfg.corner_gyro_deg > 0.0f && arena_corner_ok(c) &&
            c->travel_cm >= c->cfg.corner_gyro_min_travel_cm &&
            imu_wrap_deg(c->yaw.yaw_deg - c->course_deg) * (float)c->direction >= c->cfg.corner_gyro_deg) {
            c->corner_active = true;
            c->corner_side = c->direction;
            c->course_deg = imu_wrap_deg(c->course_deg + (float)c->direction * 90.0f);
            c->travel_cm = 0.0f;
        }
    }

    if (c->uturn_pending && c->phase == WRO_PHASE_RUN) {
        const float inner = c->direction >= 0 ? per.right_cm : per.left_cm;
        const int inner_n = c->direction >= 0 ? per.right_n : per.left_n;
        if (!c->uturn_wall_seen && inner < 0.9f * c->cfg.lane_width_cm && inner_n >= 6) c->uturn_wall_seen = true;
        if (c->uturn_wall_seen) c->uturn_travel_cm += c->scan_ds_cm;
        if (c->uturn_wall_seen && c->uturn_travel_cm >= c->cfg.uturn_extra_cm) {
            c->uturn_pending = false;
            c->phase = WRO_PHASE_UTURN;
            c->uturn_target_deg = imu_wrap_deg(c->course_deg + 180.0f);
            c->uturn_dir = per.right_cm >= per.left_cm ? 1 : -1;
            c->uturn_reversing = false;
            c->uturn_legs = 0;
        }
    }
    if (c->phase == WRO_PHASE_UTURN) {
        stop_command(c, out);
        if (uturn_step(c, dt, out)) {
            c->phase = WRO_PHASE_RUN;
            c->direction = -c->direction;
            c->reversed = true;
            c->course_deg = c->uturn_target_deg;
            c->travel_cm = 0.0f;
        }
        c->last_speed_cm_s = out->speed_cm_s;
        fill_status(c, &per, out);
        return;
    }

    /* ------------------------------------------------------------- final */
    if (c->phase == WRO_PHASE_FINISH) {
        const float inner = c->direction >= 0 ? per.right_cm : per.left_cm;
        const int inner_n = c->direction >= 0 ? per.right_n : per.left_n;
        if (!c->finish_seen_wall && inner < 0.9f * c->cfg.lane_width_cm && inner_n >= 6)
            c->finish_seen_wall = true;
        if (c->finish_seen_wall) c->finish_travel_cm += c->scan_ds_cm;
        if (c->finish_seen_wall && c->finish_travel_cm >= c->cfg.finish_extra_cm) {
            c->phase = WRO_PHASE_DONE;
            stop_command(c, out);
            c->last_speed_cm_s = 0.0f;
            fill_status(c, &per, out);
            return;
        }
    }

    c->planner_state.lane_offset_cm = 0.0f;
    race_step(c, pc, dt);
    hug_step(c, pc, per.front_cm, dt);
    if (c->phase == WRO_PHASE_PARK_SEARCH) {
        if (park_search_step(c, points, count, dt)) {
            park_step(c, points, count, dt, &per, out);
            c->last_speed_cm_s = out->speed_cm_s;
            fill_status(c, &per, out);
            return;
        }
        /* A la pared exterior (a la derecha en sentido antihorario) para ver el cajon. */
        c->planner_state.lane_offset_cm = (0.5f * c->cfg.lane_width_cm - c->cfg.park_lane_w_cm) * (float)park_mirror(c);
    }
    const uint8_t action = local_obstacle_map_recommend_cfg(&c->map, pc, &c->planner_state,
        camera_action, c->course_deg);
    command_from_action(c, action, out);
    if (c->phase == WRO_PHASE_RUN && c->cfg.pivot_enabled) {
        if (!(c->cfg.sidestep_enabled && sidestep_update(c, action, dt, out))) pivot_recovery(c, action, dt, out);
    }
    if (c->phase == WRO_PHASE_PARK_SEARCH && out->speed_cm_s > c->cfg.park_search_speed_cm_s)
        out->speed_cm_s = c->cfg.park_search_speed_cm_s;
    /* Mirar antes de pasar (ver peek_enabled). */
    if (c->cfg.peek_enabled && c->phase == WRO_PHASE_RUN && !c->corner_active && c->sidestep_phase == 0 && !c->pivoting &&
        !out->explicit_yaw_rate && out->speed_cm_s > 5.0f && c->debug.pass_cm >= 50.0f && !c->planner_state.escaping) {
        const float off = imu_wrap_deg(c->yaw.yaw_deg - c->course_deg);
        float tx = 0.0f, tf = 0.0f;
        if (fabsf(off) < c->cfg.peek_max_heading_deg &&
            local_obstacle_map_unlabeled_pillar(&c->map, pc, c->cfg.peek_bearing_min_deg, 70.0f, 25.0f, 130.0f, &tx, &tf)) {
            const float bearing = atan2f(tx, tf) * RAD_TO_DEG;       /* + si esta a la derecha */
            const float room = c->cfg.peek_max_heading_deg - 2.0f;
            if ((bearing > 0.0f && off < room) || (bearing < 0.0f && off > -room)) {
                const float base = out->speed_cm_s * tanf(out->steer_deg * DEG_TO_RAD) / c->cfg.planner.wheelbase_cm * RAD_TO_DEG;
                out->yaw_rate_dps = base + (bearing > 0.0f ? c->cfg.peek_rate_dps : -c->cfg.peek_rate_dps);
                out->explicit_yaw_rate = true;
            }
        }
    }
    /* Invertir el sentido sin frenar antes: el motor tarda ~0,1 s en obedecer y el chasis sigue 1-2 cm hacia delante,
     * justo contra el pilar que obligo a retroceder. Un ciclo de parada y despues la reversa. */
    const float v_now = c->odom_in_use ? c->odom.v_cm_s : c->last_speed_cm_s;
    if (out->speed_cm_s < -0.5f && v_now > 3.0f && !out->explicit_yaw_rate) out->speed_cm_s = 0.0f;
    /* Linea de carrera: la velocidad sigue el perfil (el planificador la baja solo si hace falta). */
    if (c->race_active && c->cfg.race_speed_enabled && out->speed_cm_s > 0.5f && !out->explicit_yaw_rate &&
        !c->planner_state.escaping && c->sidestep_phase == 0 && !c->pivoting) {
        const float f = out->speed_cm_s / pc->speed_cm_s;
        out->speed_cm_s = c->race_v_cmd * (f < 1.0f ? f : 1.0f);
    }
    c->last_speed_cm_s = out->speed_cm_s;
    fill_status(c, &per, out);
}

bool wro_controller_watchdog(wro_controller_t *c, uint64_t now_us, wro_command_t *out)
{
    if (!c || !out) return false;
    if (c->phase == WRO_PHASE_IDLE || c->phase == WRO_PHASE_DONE) return false;
    if (c->last_scan_us && now_us > c->last_scan_us &&
        now_us - c->last_scan_us > c->cfg.sensor_timeout_us) {
        stop_command(c, out);
        out->phase = c->phase;
        c->last_speed_cm_s = 0.0f;
        return true;
    }
    return false;
}

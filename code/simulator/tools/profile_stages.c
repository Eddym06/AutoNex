#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include "wro_controller.h"
#include "park_ackermann.h"
#include "park_localize.h"
#include "arena_localizer.h"
#define SAMPLES 4096
static double values[6][SAMPLES];
static unsigned counts[6];
static double now_ms(void) { LARGE_INTEGER n, f; QueryPerformanceCounter(&n); QueryPerformanceFrequency(&f); return 1000.0 * (double)n.QuadPart / (double)f.QuadPart; }
static void record(int i, double start) { if (counts[i] < SAMPLES) values[i][counts[i]++] = now_ms() - start; }
void __real_local_obstacle_map_update_cfg(local_obstacle_map_t *, const local_planner_config_t *, const local_lidar_point_t *, size_t, uint64_t, const local_camera_object_t *, size_t, uint64_t, const local_imu_sample_t *);
void __wrap_local_obstacle_map_update_cfg(local_obstacle_map_t *m, const local_planner_config_t *c, const local_lidar_point_t *p, size_t n, uint64_t t, const local_camera_object_t *o, size_t no, uint64_t ct, const local_imu_sample_t *imu) { double s = now_ms(); __real_local_obstacle_map_update_cfg(m,c,p,n,t,o,no,ct,imu); record(1,s); }
uint8_t __real_local_obstacle_map_recommend_cfg(const local_obstacle_map_t *, const local_planner_config_t *, local_planner_state_t *, uint8_t, float);
uint8_t __wrap_local_obstacle_map_recommend_cfg(const local_obstacle_map_t *m, const local_planner_config_t *c, local_planner_state_t *st, uint8_t a, float h) { double s = now_ms(); uint8_t r = __real_local_obstacle_map_recommend_cfg(m,c,st,a,h); record(2,s); return r; }
bool __real_park_localize_track(const park_loc_t *, const local_lidar_point_t *, size_t, const park_pose_t *, park_fix_t *);
bool __wrap_park_localize_track(const park_loc_t *l, const local_lidar_point_t *p, size_t n, const park_pose_t *prior, park_fix_t *fix) { double s = now_ms(); bool r = __real_park_localize_track(l,p,n,prior,fix); record(3,s); return r; }
park_ackermann_result_t __real_park_ackermann_plan(const park_params_t *, const park_ackermann_config_t *, const park_pose_t *, park_goal_t);
park_ackermann_result_t __wrap_park_ackermann_plan(const park_params_t *p, const park_ackermann_config_t *c, const park_pose_t *pose, park_goal_t g) { double s = now_ms(); park_ackermann_result_t r = __real_park_ackermann_plan(p,c,pose,g); record(4,s); return r; }
bool __real_arena_acquire(arena_loc_t *, const arena_point_t *, size_t, float);
bool __wrap_arena_acquire(arena_loc_t *a, const arena_point_t *p, size_t n, float f) { double s=now_ms(); bool r=__real_arena_acquire(a,p,n,f); record(5,s); return r; }
bool __real_arena_update(arena_loc_t *, const arena_point_t *, size_t, float);
bool __wrap_arena_update(arena_loc_t *a, const arena_point_t *p, size_t n, float f) { double s=now_ms(); bool r=__real_arena_update(a,p,n,f); record(5,s); return r; }
static int cmp(const void *a, const void *b) { double x=*(const double *)a, y=*(const double *)b; return (x>y)-(x<y); }
void profile_scan(wro_controller_t *c, const local_lidar_point_t *p, size_t n, uint64_t t, wro_command_t *out) { double s=now_ms(); wro_controller_scan(c,p,n,t,out); record(0,s); }
int profile_report(void) { int failed = 0; const char *names[]={"scan", "map", "local_planner", "park_localizer", "park_astar", "arena_localizer"}; for (int i=0;i<6;i++) { unsigned n=counts[i]; if (!n) { printf("%s: no samples\n",names[i]); continue; } qsort(values[i],n,sizeof(double),cmp); printf("%s: n=%u p50=%.3f ms p95=%.3f ms max=%.3f ms\n",names[i],n,values[i][(n-1)/2],values[i][(95*n+99)/100-1],values[i][n-1]); const char *limit = i == 0 ? getenv("AUTONEX_SCAN_P95_MS") : (i == 2 ? getenv("AUTONEX_PLANNER_P95_MS") : NULL); if (limit && *limit && values[i][(95*n+99)/100-1] > strtod(limit, NULL)) { fprintf(stderr, "%s: p95 exceeds configured limit %s ms\n", names[i], limit); failed = 1; } } return failed; }

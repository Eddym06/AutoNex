/* Prueba de trace_map con un campo sintetico (paredes, bloque central, letreros, cajon con dos topes).
 *   gcc -O2 -std=c11 -Wall -Wextra -I"LIDAR System/main" "LIDAR System/main/trace_map.c" "LIDAR System/tests/test_trace_map.c" -o test_trace_map -lm */
#include "trace_map.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static trace_map_t map, loaded;
static local_obstacle_map_t local;

typedef struct { float x0, x1, z0, z1; } box_t;
static box_t g_boxes[8];
static int g_nbox;

static unsigned g_seed = 12345;
static float rnd(void) { g_seed = g_seed * 1664525u + 1013904223u; return (float)((g_seed >> 8) & 0xffff) / 65535.0f - 0.5f; }

/* Distancia del rayo (origen, direccion) a lo primero: paredes (desde dentro), bloque central y cajas. */
static float cast(float ox, float oz, float dx, float dz)
{
    float best = 1e9f;
    /* pared exterior +-150 desde dentro */
    if (dx > 1e-6f) best = fminf(best, (150.0f - ox) / dx); else if (dx < -1e-6f) best = fminf(best, (-150.0f - ox) / dx);
    if (dz > 1e-6f) best = fminf(best, (150.0f - oz) / dz); else if (dz < -1e-6f) best = fminf(best, (-150.0f - oz) / dz);
    /* cajas, incluido el bloque central +-50 */
    box_t all[10]; int n = 0;
    all[n++] = (box_t){-50, 50, -50, 50};
    for (int i = 0; i < g_nbox; ++i) all[n++] = g_boxes[i];
    for (int i = 0; i < n; ++i) {
        float tmin = 0, tmax = 1e9f;
        const float o[2] = {ox, oz}, d[2] = {dx, dz}, lo[2] = {all[i].x0, all[i].z0}, hi[2] = {all[i].x1, all[i].z1};
        int ok = 1;
        for (int a = 0; a < 2 && ok; ++a) {
            if (fabsf(d[a]) < 1e-9f) { if (o[a] < lo[a] || o[a] > hi[a]) ok = 0; }
            else {
                float t1 = (lo[a] - o[a]) / d[a], t2 = (hi[a] - o[a]) / d[a];
                if (t1 > t2) { float t = t1; t1 = t2; t2 = t; }
                if (t1 > tmin) tmin = t1;
                if (t2 < tmax) tmax = t2;
                if (tmin > tmax) ok = 0;
            }
        }
        if (ok && tmin > 0 && tmin < best) best = tmin;
    }
    return best;
}

static void scan(const arena_pose_t *pose, float lf, int moved_pillar)
{
    local_lidar_point_t pts[360];
    const float th = pose->theta_deg * 0.017453292519943295f, ox = pose->x_cm + sinf(th) * lf, oz = pose->z_cm + cosf(th) * lf;
    for (int i = 0; i < 360; ++i) {
        const float ang = (float)i - 180.0f, a = th + ang * 0.017453292519943295f;
        const float d = cast(ox, oz, sinf(a), cosf(a)) + rnd() * 1.0f;
        pts[i] = (local_lidar_point_t){.angle_deg = ang, .distance_cm = d, .valid = d > 5.0f && d < 450.0f};
    }
    /* letreros confirmados que el robot ve de frente (sin oclusion fina): rojo en (70,0) y verde en (-95,40) */
    local.colored_count = 0;
    const float c = cosf(th), s = sinf(th);
    const struct { float x, z; int color; } signs[2] = {{70.0f, 0.0f, 1}, {-95.0f, 40.0f, 2}};
    for (int k = 0; k < 2; ++k) {
        if (moved_pillar && k == 0) continue;
        const float dx = signs[k].x - pose->x_cm, dz = signs[k].z - pose->z_cm;
        const float lat = c * dx - s * dz, fwd = s * dx + c * dz;
        if (fwd > 5.0f && hypotf(lat, fwd) < 120.0f && fabsf(atan2f(lat, fwd)) < 0.7f)
            local.colored[local.colored_count++] = (local_colored_object_t){.x_cm = lat, .forward_cm = fwd, .color = (uint8_t)signs[k].color, .confirmed = true};
    }
    arena_pose_t noisy = *pose;
    noisy.x_cm += rnd() * 2.0f; noisy.z_cm += rnd() * 2.0f;              /* error de localizacion de +-1 cm */
    trace_map_update(&map, pts, 360, &local, &noisy, lf);
}

static void drive_lap(int lap_scans, float lf, int moved)
{
    /* cuadrado de lado 200 centrado en el campo: el carril entre el bloque (+-50) y la pared (+-150) */
    for (int i = 0; i < lap_scans; ++i) {
        const float u = 800.0f * (float)i / (float)lap_scans;
        float x, z, theta;
        if (u < 200) { x = -100 + u; z = 100; theta = 90; }
        else if (u < 400) { x = 100; z = 100 - (u - 200); theta = 180; }
        else if (u < 600) { x = 100 - (u - 400); z = -100; theta = 270; }
        else { x = -100; z = -100 + (u - 600); theta = 0; }
        const arena_pose_t p = {.x_cm = x, .z_cm = z, .theta_deg = theta};
        scan(&p, lf, moved);
    }
}

static const trace_object_t *find_near(float x, float z, float tol, size_t *idx)
{
    size_t n; const trace_object_t *o = trace_map_objects(&map, &n);
    for (size_t i = 0; i < n; ++i) if (hypotf(o[i].x_cm - x, o[i].z_cm - z) <= tol) { if (idx) *idx = i; return &o[i]; }
    return NULL;
}

int main(void)
{
    const float lf = 10.0f;
    /* mundo: letrero rojo 5x5 en (70,0), verde en (-95,40), cajon en la pared -z con topes en x = -20.5 y 20.5 (centro 0) */
    g_boxes[g_nbox++] = (box_t){67.5f, 72.5f, -2.5f, 2.5f};
    g_boxes[g_nbox++] = (box_t){-97.5f, -92.5f, 37.5f, 42.5f};
    g_boxes[g_nbox++] = (box_t){-21.5f, -19.5f, -150.0f, -130.0f};
    g_boxes[g_nbox++] = (box_t){19.5f, 21.5f, -150.0f, -130.0f};

    clock_t t0 = clock();
    assert(!trace_map_is_built(&map));
    drive_lap(400, lf, 0);
    trace_map_lap_complete(&map);                 /* 1.a vuelta: el mapa queda construido */
    assert(trace_map_is_built(&map) && map.laps_mapped == 1);
    drive_lap(400, lf, 0);
    trace_map_lap_complete(&map);                 /* 2.a vuelta: se refina, los objetos que siguen suman una vuelta */
    assert(map.laps_mapped == 2);
    const double ms_update = 1000.0 * (double)(clock() - t0) / CLOCKS_PER_SEC / 800.0;
    t0 = clock();
    for (int i = 0; i < 200; ++i) trace_map_refresh(&map);
    const double ms_refresh = 1000.0 * (double)(clock() - t0) / CLOCKS_PER_SEC / 200.0;

    size_t n; const trace_object_t *objs = trace_map_objects(&map, &n);
    printf("objetos: %zu\n", n);
    for (size_t i = 0; i < n; ++i)
        printf("  id %u tipo %u color %u (%.1f, %.1f) %.1f x %.1f cm, eje %.0f, conf %u\n", objs[i].id, objs[i].kind, objs[i].color,
               objs[i].x_cm, objs[i].z_cm, objs[i].length_cm, objs[i].width_cm, objs[i].axis_deg, objs[i].confidence);

    size_t ir, ig;
    const trace_object_t *red = find_near(70.0f, 0.0f, 4.0f, &ir), *green = find_near(-95.0f, 40.0f, 4.0f, &ig);
    assert(red && red->kind == TRACE_OBJ_PILLAR && red->color == TRACE_COLOR_RED);
    assert(red->laps_seen == 2);                 /* lo vio la vuelta de construccion y la de refinado */
    assert(green && green->kind == TRACE_OBJ_PILLAR && green->color == TRACE_COLOR_GREEN);
    printf("rojo (%.1f,%.1f) error %.1f cm; verde (%.1f,%.1f) error %.1f cm\n", red->x_cm, red->z_cm, hypotf(red->x_cm - 70, red->z_cm),
           green->x_cm, green->z_cm, hypotf(green->x_cm + 95, green->z_cm - 40));

    const trace_spot_t *spot = trace_map_spot(&map);
    printf("plaza: estado %d centro (%.1f, %.1f) largo %.1f, topes (%.1f,%.1f) y (%.1f,%.1f), confianza %u\n", spot->status, spot->x_cm, spot->z_cm,
           spot->length_cm, spot->bar_x_cm[0], spot->bar_z_cm[0], spot->bar_x_cm[1], spot->bar_z_cm[1], spot->confidence);
    assert(spot->status == TRACE_SPOT_CONFIRMED);
    assert(hypotf(spot->x_cm - 0.0f, spot->z_cm - (-140.0f)) < 3.0f);
    assert(fabsf(spot->length_cm - 39.0f) < 4.0f);

    /* espacio libre e info de ocupacion */
    assert(trace_map_is_free(&map, 0.0f, 100.0f));
    assert(!trace_map_is_free(&map, 70.0f, 0.0f));
    assert(trace_map_cell_at(&map, 70.0f, 0.0f) & TRACE_MAP_OBSERVED);
    assert(trace_map_clearance_cm(&map, 70.0f, 20.0f, 60.0f) < 22.0f);

    /* guardado: ida y vuelta */
    static uint8_t buf[70000];
    const size_t bytes = trace_map_save(&map, buf, sizeof buf);
    assert(bytes > 0 && trace_map_load(&loaded, buf, bytes));
    assert(loaded.object_count == map.object_count && loaded.spot.status == map.spot.status && loaded.occupied_cells == map.occupied_cells);
    assert(memcmp(loaded.cells, map.cells, sizeof map.cells) == 0);
    assert(loaded.laps_mapped == map.laps_mapped && loaded.objects[0].laps_seen == map.objects[0].laps_seen);
    buf[100] ^= 0x55;
    assert(!trace_map_load(&loaded, buf, bytes));            /* el CRC detecta un byte corrupto */

    /* algo se mueve: se quita el letrero rojo y se da mas vueltas; debe desaparecer del mapa */
    g_boxes[0] = (box_t){1000, 1001, 1000, 1001};
    for (int lap = 0; lap < 3; ++lap) drive_lap(400, lf, 1);
    trace_map_refresh(&map);
    assert(!find_near(70.0f, 0.0f, 4.0f, NULL));
    assert(find_near(-95.0f, 40.0f, 4.0f, NULL));
    printf("cleared_cells=%u; el letrero retirado desaparece, el verde se conserva\n", map.cleared_cells);

    printf("tiempos (PC): update %.3f ms/barrido (360 rayos con espacio libre), refresh %.3f ms; guardado %zu bytes; RAM %zu bytes\n",
           ms_update, ms_refresh, bytes, sizeof map);
    printf("PASS: trace_map (vueltas, objetos, color, plaza, espacio libre, borrado, guardado)\n");
    return 0;
}

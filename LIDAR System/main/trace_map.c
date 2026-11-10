#include "trace_map.h"
#include <math.h>
#include <string.h>

#define RAD 0.017453292519943295f
#define N TRACE_MAP_SIZE
#define CELL TRACE_MAP_CELL_CM
#define ORIGIN TRACE_MAP_ORIGIN_CM
#define TRACE_CARVE_MAX_CM 260.0f      /* alcance con el que se traza espacio libre */
#define TRACE_CARVE_SKIP_CM 6.0f       /* no se traza dentro del propio coche */
#define TRACE_MIN_SUPPORT 2u           /* retornos repetidos para que una celda cuente como objeto */
#define TRACE_LIST_CAP 2048
#define TRACE_MIN_OBJECT_CELLS 4       /* un letrero de 5 cm son 4-14 celdas; menos es ruido o un resto de pared */
#define TRACE_MIN_OBJECT_CONF 100
#define TRACE_DEFAULT_OUTER 150.0f
#define TRACE_DEFAULT_INNER 50.0f
#define TRACE_DEFAULT_BAND 6.0f        /* grosor de la pared que se resta (error de pose 3,5 cm + celda) */
#define TRACE_DEFAULT_SPACING 41.0f    /* cajon de 39 cm + tope de 2 cm */
#define TRACE_SPOT_KEEP_SCANS 300u     /* una plaza confirmada se conserva este tiempo sin verla de nuevo */

static uint8_t g_visited[N * N / 8];
static uint16_t g_list[TRACE_LIST_CAP];

static int cell_of(float v) { return (int)floorf((v - ORIGIN) / CELL); }
static float coord(int i) { return ORIGIN + ((float)i + 0.5f) * CELL; }
static int bit_test_set(uint8_t *bits, unsigned i)
{
    const uint8_t mask = (uint8_t)(1u << (i & 7u));
    const int was = (bits[i >> 3] & mask) != 0;
    bits[i >> 3] |= mask;
    return was;
}

static void clear_cell(trace_map_t *m, int r, int c)
{
    if (m->cells[r][c] & TRACE_MAP_OBSERVED) { --m->occupied_cells; ++m->cleared_cells; }
    m->cells[r][c] = 0;
    m->miss[r][c] = 0;
}

static void mark(trace_map_t *m, float x, float z, uint8_t bits)
{
    if (!isfinite(x) || !isfinite(z)) return;
    const float limit = ORIGIN + N * CELL;
    if (x < ORIGIN || x >= limit || z < ORIGIN || z >= limit) { ++m->clipped_points; return; }
    const int col = cell_of(x), row = cell_of(z);
    if (col < 0 || col >= N || row < 0 || row >= N) { ++m->clipped_points; return; }
    uint8_t *cell = &m->cells[row][col];
    if ((bits & TRACE_MAP_OBSERVED) && !(*cell & TRACE_MAP_OBSERVED)) ++m->occupied_cells;
    if (bits & TRACE_MAP_OBSERVED) {
        const unsigned i = (unsigned)(row * N + col);
        if (!bit_test_set(m->scan_seen, i)) {
            if ((*cell >> TRACE_MAP_SUPPORT_SHIFT) < 15u) *cell = (uint8_t)(*cell + 16u);
            m->miss[row][col] >>= 1;                  /* un retorno desmiente a los rayos que la atravesaron */
        }
    }
    *cell |= bits;
}

/* Traza el rayo del sensor al retorno por recorrido de rejilla (Amanatides-Woo, solo sumas y comparaciones): las
 * celdas que atraviesa son espacio libre; una celda ocupada que se atraviesa mas veces de las que se vio acaba
 * borrada (un obstaculo que se movio, un fantasma). */
static void carve(trace_map_t *m, float sx, float sz, float hx, float hz)
{
    const float x0 = (sx - ORIGIN) / CELL, z0 = (sz - ORIGIN) / CELL, x1 = (hx - ORIGIN) / CELL, z1 = (hz - ORIGIN) / CELL;
    const float dx = x1 - x0, dz = z1 - z0, len_cells = hypotf(dx, dz);
    const float skip = TRACE_CARVE_SKIP_CM / CELL, maxc = TRACE_CARVE_MAX_CM / CELL;
    if (!(len_cells > skip + 1.0f)) return;
    int cx = (int)floorf(x0), cz = (int)floorf(z0);
    const int hit_c = (int)floorf(x1), hit_r = (int)floorf(z1);
    const int sxn = dx > 0.0f ? 1 : -1, szn = dz > 0.0f ? 1 : -1;
    const float inf = 1e30f;
    const float tdx = dx != 0.0f ? fabsf(1.0f / dx) : inf, tdz = dz != 0.0f ? fabsf(1.0f / dz) : inf;
    float tmx = dx > 0.0f ? ((float)(cx + 1) - x0) / dx : (dx < 0.0f ? (x0 - (float)cx) / -dx : inf);
    float tmz = dz > 0.0f ? ((float)(cz + 1) - z0) / dz : (dz < 0.0f ? (z0 - (float)cz) / -dz : inf);
    const float skip_t = skip / len_cells, max_t = maxc / len_cells;
    while (cx != hit_c || cz != hit_r) {
        float t;
        if (tmx < tmz) { cx += sxn; t = tmx; tmx += tdx; } else { cz += szn; t = tmz; tmz += tdz; }
        if (cx < 0 || cx >= N || cz < 0 || cz >= N || t > max_t) break;
        if (cx == hit_c && cz == hit_r) break;
        if (t < skip_t) continue;
        if (bit_test_set(m->miss_seen, (unsigned)(cz * N + cx))) continue;
        if (m->miss[cz][cx] < 255u) ++m->miss[cz][cx];
        const uint8_t cell = m->cells[cz][cx];
        if ((cell & TRACE_MAP_OBSERVED) && m->miss[cz][cx] > 3u * (unsigned)(cell >> TRACE_MAP_SUPPORT_SHIFT) + 6u)
            clear_cell(m, cz, cx);
    }
}

void trace_map_update(trace_map_t *m, const local_lidar_point_t *points, size_t count,
                      const local_obstacle_map_t *local, const arena_pose_t *pose,
                      float lidar_forward_cm)
{
    if (!m || !points || !pose || !isfinite(pose->x_cm) || !isfinite(pose->z_cm) || !isfinite(pose->theta_deg)) return;
    memset(m->scan_seen, 0, sizeof m->scan_seen);
    memset(m->miss_seen, 0, sizeof m->miss_seen);
    const float a = pose->theta_deg * RAD, sn = sinf(a), co = cosf(a);
    const float sx = pose->x_cm + sn * lidar_forward_cm, sz = pose->z_cm + co * lidar_forward_cm;
    for (size_t i = 0; i < count; i++) {
        if (!points[i].valid || !isfinite(points[i].angle_deg) || !isfinite(points[i].distance_cm) || points[i].distance_cm <= 0) continue;
        const float t = points[i].angle_deg * RAD, x = sinf(t) * points[i].distance_cm;
        const float f = lidar_forward_cm + cosf(t) * points[i].distance_cm;
        const float hx = pose->x_cm + co * x + sn * f, hz = pose->z_cm - sn * x + co * f;
        const uint32_t clipped = m->clipped_points;
        mark(m, hx, hz, TRACE_MAP_OBSERVED);
        if (m->clipped_points == clipped) carve(m, sx, sz, hx, hz);
    }
    if (local) {
        for (size_t i = 0; i < local->colored_count && i < LOCAL_MAP_MAX_CAMERA_OBJECTS; i++) {
            const local_colored_object_t *o = &local->colored[i];
            uint8_t bits = 0;
            if (o->confirmed && o->color == 1) bits = TRACE_MAP_RED;
            else if (o->confirmed && o->color == 2) bits = TRACE_MAP_GREEN;
            else if (o->color == 3) bits = TRACE_MAP_MAGENTA_CANDIDATE;
            if (bits) mark(m, pose->x_cm + co * o->x_cm + sn * o->forward_cm, pose->z_cm - sn * o->x_cm + co * o->forward_cm, bits);
        }
    }
    ++m->scans;
}

/* ------------------------------------------------------------------ objetos */

static float cfg_outer(const trace_map_t *m) { return m->field_outer_half_cm > 0.0f ? m->field_outer_half_cm : TRACE_DEFAULT_OUTER; }
static float cfg_inner(const trace_map_t *m) { return m->field_inner_half_cm > 0.0f ? m->field_inner_half_cm : TRACE_DEFAULT_INNER; }
static float cfg_band(const trace_map_t *m) { return m->wall_band_cm > 0.0f ? m->wall_band_cm : TRACE_DEFAULT_BAND; }

/* Pared exterior, bloque central y todo lo que cae fuera del campo: no son objetos. */
static int is_field(const trace_map_t *m, float x, float z)
{
    const float outer = cfg_outer(m), inner = cfg_inner(m), band = cfg_band(m);
    const float ax = fabsf(x), az = fabsf(z);
    if (ax > outer - band || az > outer - band) return 1;
    if (ax > outer - 12.0f && az > outer - 12.0f) return 1;      /* rincones: restos de pared, nunca un objeto */
    if (ax < inner + band && az < inner + band) return 1;
    return 0;
}

static int qualifies(const trace_map_t *m, int r, int c)
{
    const uint8_t v = m->cells[r][c];
    return (v & TRACE_MAP_OBSERVED) && (v >> TRACE_MAP_SUPPORT_SHIFT) >= TRACE_MIN_SUPPORT && !is_field(m, coord(c), coord(r));
}

static float heading_deg(float dx, float dz)
{
    float h = atan2f(dx, dz) / RAD;
    while (h < 0.0f) h += 180.0f;
    while (h >= 180.0f) h -= 180.0f;
    return h;
}

static float axis_diff(float a, float b)
{
    float d = fabsf(a - b);
    while (d >= 180.0f) d -= 180.0f;
    return d > 90.0f ? 180.0f - d : d;
}

static void wall_geometry(const trace_map_t *m, const trace_object_t *o, int *side, float *along, float *perp);

static int build_object(const trace_map_t *m, int n, trace_object_t *o)
{
    double sx = 0, sz = 0, sxx = 0, szz = 0, sxz = 0, sup = 0;
    int red = 0, green = 0, mag = 0;
    for (int k = 0; k < n; ++k) {
        const int r = g_list[k] / N, c = g_list[k] % N;
        const double x = coord(c), z = coord(r);
        sx += x; sz += z; sxx += x * x; szz += z * z; sxz += x * z;
        sup += m->cells[r][c] >> TRACE_MAP_SUPPORT_SHIFT;
        for (int dr = -1; dr <= 1; ++dr) for (int dc = -1; dc <= 1; ++dc) {
            const int rr = r + dr, cc = c + dc;
            if (rr < 0 || rr >= N || cc < 0 || cc >= N) continue;
            const uint8_t v = m->cells[rr][cc];
            red += (v & TRACE_MAP_RED) != 0; green += (v & TRACE_MAP_GREEN) != 0; mag += (v & TRACE_MAP_MAGENTA_CANDIDATE) != 0;
        }
    }
    const double mx = sx / n, mz = sz / n;
    const double vxx = sxx / n - mx * mx, vzz = szz / n - mz * mz, vxz = sxz / n - mx * mz;
    const double t = 0.5 * atan2(2.0 * vxz, vxx - vzz);
    const float ux = (float)cos(t), uz = (float)sin(t);
    float mnu = 1e9f, mxu = -1e9f, mnv = 1e9f, mxv = -1e9f;
    for (int k = 0; k < n; ++k) {
        const int r = g_list[k] / N, c = g_list[k] % N;
        const float dx = coord(c) - (float)mx, dz = coord(r) - (float)mz;
        const float pu = dx * ux + dz * uz, pv = -dx * uz + dz * ux;
        if (pu < mnu) mnu = pu;
        if (pu > mxu) mxu = pu;
        if (pv < mnv) mnv = pv;
        if (pv > mxv) mxv = pv;
    }
    o->x_cm = (float)mx; o->z_cm = (float)mz;
    o->length_cm = (mxu - mnu) + CELL;
    o->width_cm = (mxv - mnv) + CELL;
    o->axis_deg = heading_deg(ux, uz);
    o->cells = (uint16_t)(n > 65535 ? 65535 : n);
    {
        double conf = sup / n * 17.0;
        o->confidence = (uint8_t)(conf > 255.0 ? 255.0 : conf);
    }
    o->color = red > 0 && red >= green ? TRACE_COLOR_RED : (green > 0 ? TRACE_COLOR_GREEN : (mag > 0 ? TRACE_COLOR_MAGENTA : TRACE_COLOR_NONE));
    /* Una barra pegada a la pared (un tope) se ensancha con los errores de pose de distintas pasadas: ahi se admite
     * una relacion largo/ancho menor que para una barra suelta. */
    int side; float along, perp;
    wall_geometry(m, o, &side, &along, &perp);
    const int near_wall = perp >= 4.0f && perp <= 26.0f;
    if (o->length_cm >= 40.0f) o->kind = TRACE_OBJ_WALL;
    else if (o->length_cm >= 8.0f && o->length_cm >= 2.2f * o->width_cm) o->kind = TRACE_OBJ_BAR;
    else if (near_wall && o->length_cm >= 12.0f && o->length_cm <= 26.0f && o->length_cm >= 1.4f * o->width_cm) o->kind = TRACE_OBJ_BAR;
    else if (o->length_cm <= 15.0f) o->kind = TRACE_OBJ_PILLAR;
    else o->kind = TRACE_OBJ_UNKNOWN;
    return 1;
}

static void wall_geometry(const trace_map_t *m, const trace_object_t *o, int *side, float *along, float *perp)
{
    const float outer = cfg_outer(m);
    if (fabsf(o->x_cm) >= fabsf(o->z_cm)) { *side = o->x_cm > 0.0f ? 0 : 1; *along = o->z_cm; *perp = outer - fabsf(o->x_cm); }
    else { *side = o->z_cm > 0.0f ? 2 : 3; *along = o->x_cm; *perp = outer - fabsf(o->z_cm); }
}

static int bar_at_wall(const trace_map_t *m, const trace_object_t *o, int *side, float *along, float *perp)
{
    if (o->kind != TRACE_OBJ_BAR) return 0;
    wall_geometry(m, o, side, along, perp);
    if (*perp < 5.0f || *perp > 24.0f) return 0;
    const float normal_heading = *side < 2 ? 90.0f : 0.0f;       /* la barra es perpendicular a la pared */
    return axis_diff(o->axis_deg, normal_heading) <= 30.0f;
}

static void find_spot(trace_map_t *m)
{
    const float spacing = m->spot_spacing_cm > 0.0f ? m->spot_spacing_cm : TRACE_DEFAULT_SPACING;
    const float tol = 4.5f, outer = cfg_outer(m);
    trace_spot_t best;
    memset(&best, 0, sizeof best);
    best.bar_x_cm[1] = best.bar_z_cm[1] = NAN;
    float best_err = 1e9f;
    int bar_side[TRACE_MAP_MAX_OBJECTS];
    float bar_along[TRACE_MAP_MAX_OBJECTS], bar_perp[TRACE_MAP_MAX_OBJECTS];
    for (int i = 0; i < m->object_count; ++i)
        if (!bar_at_wall(m, &m->objects[i], &bar_side[i], &bar_along[i], &bar_perp[i])) bar_side[i] = -1;
    for (int i = 0; i < m->object_count; ++i) {
        if (bar_side[i] < 0) continue;
        for (int j = i + 1; j < m->object_count; ++j) {
            if (bar_side[j] != bar_side[i]) continue;
            const float sep = fabsf(bar_along[i] - bar_along[j]);
            if (fabsf(sep - spacing) > tol || fabsf(bar_perp[i] - bar_perp[j]) > 4.0f) continue;
            const float err = fabsf(sep - spacing);
            if (err >= best_err) continue;
            best_err = err;
            const int side = bar_side[i];
            const float along = 0.5f * (bar_along[i] + bar_along[j]);
            const float depth = outer - 10.0f;                    /* centro de la plaza: a media anchura de la pared */
            best.status = TRACE_SPOT_CONFIRMED;
            best.wall_side = (uint8_t)side;
            best.axis_deg = side < 2 ? 0.0f : 90.0f;
            best.x_cm = side < 2 ? (side == 0 ? depth : -depth) : along;
            best.z_cm = side < 2 ? along : (side == 2 ? depth : -depth);
            best.length_cm = sep - 2.0f;
            best.width_cm = 20.0f;
            best.bar_x_cm[0] = m->objects[i].x_cm; best.bar_z_cm[0] = m->objects[i].z_cm;
            best.bar_x_cm[1] = m->objects[j].x_cm; best.bar_z_cm[1] = m->objects[j].z_cm;
            best.bar_id[0] = m->objects[i].id; best.bar_id[1] = m->objects[j].id;
            int conf = m->objects[i].confidence < m->objects[j].confidence ? m->objects[i].confidence : m->objects[j].confidence;
            if (m->objects[i].color == TRACE_COLOR_MAGENTA && m->objects[j].color == TRACE_COLOR_MAGENTA) conf += 40;
            best.confidence = (uint8_t)(conf > 255 ? 255 : conf);
        }
    }
    if (best.status == TRACE_SPOT_NONE) {
        for (int i = 0; i < m->object_count; ++i) {                 /* un tope magenta junto a la pared: hipotesis */
            if (bar_side[i] < 0 || m->objects[i].color != TRACE_COLOR_MAGENTA) continue;
            best.status = TRACE_SPOT_HYPOTHESIS;
            best.wall_side = (uint8_t)bar_side[i];
            best.axis_deg = bar_side[i] < 2 ? 0.0f : 90.0f;
            best.bar_x_cm[0] = m->objects[i].x_cm; best.bar_z_cm[0] = m->objects[i].z_cm;
            best.bar_id[0] = m->objects[i].id;
            best.confidence = m->objects[i].confidence;
            break;
        }
    }
    best.length_cm = best.status == TRACE_SPOT_NONE ? 0.0f : best.length_cm;
    if (best.status == TRACE_SPOT_CONFIRMED) {
        m->spot = best;
        m->spot_scan = m->scans;
    } else if (m->spot.status == TRACE_SPOT_CONFIRMED && m->scans - m->spot_scan <= TRACE_SPOT_KEEP_SCANS) {
        /* se conserva la plaza ya vista: un topes tapados un rato no la borran */
    } else {
        m->spot = best;
    }
}

void trace_map_refresh(trace_map_t *m)
{
    if (!m) return;
    trace_object_t found[TRACE_MAP_MAX_OBJECTS];
    int nfound = 0;
    memset(g_visited, 0, sizeof g_visited);
    for (int r0 = 0; r0 < N && nfound < TRACE_MAP_MAX_OBJECTS; ++r0) {
        for (int c0 = 0; c0 < N && nfound < TRACE_MAP_MAX_OBJECTS; ++c0) {
            if (!qualifies(m, r0, c0) || bit_test_set(g_visited, (unsigned)(r0 * N + c0))) continue;
            int head = 0, tail = 0;
            g_list[tail++] = (uint16_t)(r0 * N + c0);
            while (head < tail) {
                const int r = g_list[head] / N, c = g_list[head] % N;
                ++head;
                for (int dr = -1; dr <= 1; ++dr) for (int dc = -1; dc <= 1; ++dc) {
                    const int rr = r + dr, cc = c + dc;
                    if ((dr == 0 && dc == 0) || rr < 0 || rr >= N || cc < 0 || cc >= N) continue;
                    if (!qualifies(m, rr, cc) || bit_test_set(g_visited, (unsigned)(rr * N + cc))) continue;
                    if (tail < TRACE_LIST_CAP) g_list[tail++] = (uint16_t)(rr * N + cc);
                }
            }
            if (tail < TRACE_MIN_OBJECT_CELLS) continue;
            trace_object_t o;
            memset(&o, 0, sizeof o);
            build_object(m, tail, &o);
            if (o.confidence < TRACE_MIN_OBJECT_CONF) continue;
            o.last_scan = m->scans;
            found[nfound++] = o;
        }
    }
    /* identidad estable: el objeto nuevo hereda el id del anterior mas cercano (<= 8 cm) que siga sin asignar */
    uint8_t taken[TRACE_MAP_MAX_OBJECTS];
    memset(taken, 0, sizeof taken);
    for (int i = 0; i < nfound; ++i) {
        int best = -1;
        float bd = 8.0f;
        for (int j = 0; j < m->object_count; ++j) {
            if (taken[j]) continue;
            const float d = hypotf(found[i].x_cm - m->objects[j].x_cm, found[i].z_cm - m->objects[j].z_cm);
            if (d < bd) { bd = d; best = j; }
        }
        if (best >= 0) { taken[best] = 1; found[i].id = m->objects[best].id; }
        else found[i].id = ++m->next_id;
    }
    memcpy(m->objects, found, (size_t)nfound * sizeof(trace_object_t));
    m->object_count = (uint8_t)nfound;
    m->refreshed_scan = m->scans;
    find_spot(m);
}

/* ------------------------------------------------------------------ consultas */

const trace_object_t *trace_map_objects(const trace_map_t *m, size_t *count)
{
    if (count) *count = m ? m->object_count : 0;
    return m ? m->objects : NULL;
}

const trace_spot_t *trace_map_spot(const trace_map_t *m) { return m ? &m->spot : NULL; }

uint8_t trace_map_cell_at(const trace_map_t *m, float x, float z)
{
    if (!m || !isfinite(x) || !isfinite(z)) return 0;
    const int c = cell_of(x), r = cell_of(z);
    return (c < 0 || c >= N || r < 0 || r >= N) ? 0 : m->cells[r][c];
}

int trace_map_is_free(const trace_map_t *m, float x, float z)
{
    if (!m || !isfinite(x) || !isfinite(z)) return 0;
    const int c = cell_of(x), r = cell_of(z);
    if (c < 0 || c >= N || r < 0 || r >= N) return 0;
    return !(m->cells[r][c] & TRACE_MAP_OBSERVED) && m->miss[r][c] >= 2u;
}

float trace_map_clearance_cm(const trace_map_t *m, float x, float z, float max_cm)
{
    if (!m || !isfinite(x) || !isfinite(z)) return max_cm;
    const int c0 = cell_of(x), r0 = cell_of(z), kmax = (int)(max_cm / CELL) + 1;
    float best = max_cm;
    for (int k = 0; k <= kmax; ++k) {
        if ((float)(k - 1) * CELL > best) break;
        for (int dr = -k; dr <= k; ++dr) for (int dc = -k; dc <= k; ++dc) {
            if ((dr < 0 ? -dr : dr) != k && (dc < 0 ? -dc : dc) != k) continue;   /* solo el anillo */
            const int r = r0 + dr, c = c0 + dc;
            if (r < 0 || r >= N || c < 0 || c >= N) continue;
            const uint8_t v = m->cells[r][c];
            if (!(v & TRACE_MAP_OBSERVED) || (v >> TRACE_MAP_SUPPORT_SHIFT) < TRACE_MIN_SUPPORT) continue;
            const float d = hypotf(coord(c) - x, coord(r) - z);
            if (d < best) best = d;
        }
    }
    return best;
}

/* ------------------------------------------------------------------ guardado */

#define TRACE_MAGIC 0x324D5254u   /* "TRM2" */
static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; ++i) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t trace_map_save(const trace_map_t *m, uint8_t *buf, size_t cap)
{
    if (!m || !buf) return 0;
    size_t used = 0;
    for (int r = 0; r < N; ++r) for (int c = 0; c < N; ++c) used += m->cells[r][c] ? 1u : 0u;
    const size_t need = 16 + used * 4 + sizeof(m->scan_seen) + 1 + sizeof(m->objects[0]) * m->object_count + sizeof(m->spot) + 2;
    if (cap < need) return 0;
    size_t p = 0;
    const uint32_t magic = TRACE_MAGIC, n_used = (uint32_t)used, scans = m->scans;
    memcpy(buf + p, &magic, 4); p += 4;
    buf[p++] = 1; buf[p++] = (uint8_t)(N & 0xFF); buf[p++] = (uint8_t)(N >> 8); buf[p++] = 0;
    memcpy(buf + p, &n_used, 4); p += 4;
    memcpy(buf + p, &scans, 4); p += 4;
    for (int r = 0; r < N; ++r) for (int c = 0; c < N; ++c) {
        if (!m->cells[r][c]) continue;
        const uint16_t idx = (uint16_t)(r * N + c);
        memcpy(buf + p, &idx, 2); p += 2;
        buf[p++] = m->cells[r][c]; buf[p++] = m->miss[r][c];
    }
    /* espacio libre conocido: un bit por celda (rayos >= 2 y sin ocupar) */
    memset(buf + p, 0, sizeof(m->scan_seen));
    for (int r = 0; r < N; ++r) for (int c = 0; c < N; ++c)
        if (!(m->cells[r][c] & TRACE_MAP_OBSERVED) && m->miss[r][c] >= 2u) { const unsigned i = (unsigned)(r * N + c); buf[p + (i >> 3)] |= (uint8_t)(1u << (i & 7u)); }
    p += sizeof(m->scan_seen);
    buf[p++] = m->object_count;
    memcpy(buf + p, m->objects, sizeof(m->objects[0]) * m->object_count); p += sizeof(m->objects[0]) * m->object_count;
    memcpy(buf + p, &m->spot, sizeof m->spot); p += sizeof m->spot;
    const uint16_t crc = crc16(buf, p);
    memcpy(buf + p, &crc, 2); p += 2;
    return p;
}

int trace_map_load(trace_map_t *m, const uint8_t *buf, size_t len)
{
    if (!m || !buf || len < 18) return 0;
    uint16_t crc;
    memcpy(&crc, buf + len - 2, 2);
    if (crc16(buf, len - 2) != crc) return 0;
    uint32_t magic, n_used, scans;
    memcpy(&magic, buf, 4);
    if (magic != TRACE_MAGIC || buf[4] != 1 || (buf[5] | (buf[6] << 8)) != N) return 0;
    memcpy(&n_used, buf + 8, 4);
    memcpy(&scans, buf + 12, 4);
    size_t p = 16;
    if (n_used > (uint32_t)N * N || len < p + (size_t)n_used * 4 + sizeof(m->scan_seen) + 1 + sizeof m->spot + 2) return 0;
    memset(m->cells, 0, sizeof m->cells);
    memset(m->miss, 0, sizeof m->miss);
    m->occupied_cells = 0;
    for (uint32_t k = 0; k < n_used; ++k) {
        uint16_t idx;
        memcpy(&idx, buf + p, 2); p += 2;
        if (idx >= N * N) return 0;
        m->cells[idx / N][idx % N] = buf[p++];
        m->miss[idx / N][idx % N] = buf[p++];
        if (m->cells[idx / N][idx % N] & TRACE_MAP_OBSERVED) ++m->occupied_cells;
    }
    for (unsigned i = 0; i < (unsigned)(N * N); ++i)
        if (buf[p + (i >> 3)] & (1u << (i & 7u))) { const int r = (int)(i / N), c = (int)(i % N); if (m->miss[r][c] < 2u) m->miss[r][c] = 2u; }
    p += sizeof(m->scan_seen);
    const unsigned n_obj = buf[p++];
    if (n_obj > TRACE_MAP_MAX_OBJECTS || len < p + sizeof(m->objects[0]) * n_obj + sizeof m->spot + 2) return 0;
    memcpy(m->objects, buf + p, sizeof(m->objects[0]) * n_obj); p += sizeof(m->objects[0]) * n_obj;
    m->object_count = (uint8_t)n_obj;
    memcpy(&m->spot, buf + p, sizeof m->spot);
    m->scans = scans;
    m->spot_scan = scans;
    for (unsigned i = 0; i < n_obj; ++i) if (m->objects[i].id > m->next_id) m->next_id = m->objects[i].id;
    return 1;
}

#include <math.h>
#include <stdio.h>

#include "race_line.h"

static int failures;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FALLA: %s (linea %d)\n", msg, __LINE__); ++failures; } } while (0)

static race_line_t line;

static void check_inside(const race_line_t *r, const char *tag)
{
    const race_config_t *c = &r->cfg;
    const float m = c->half_width_cm + c->wall_margin_cm - 0.01f;
    int bad = 0;
    for (int i = 0; i < r->n; ++i) {
        const float rho = fmaxf(fabsf(r->x[i]), fabsf(r->z[i]));
        if (rho > c->outer_half_cm - m || rho < c->inner_half_cm + m) ++bad;
    }
    if (bad) printf("FALLA: %s: %d estaciones fuera del carril\n", tag, bad), ++failures;
}

static void check_signs(const race_line_t *r, const race_sign_t *s, int n, const char *tag)
{
    const race_config_t *c = &r->cfg;
    const float rs = c->sign_half_cm * 1.4142f + c->half_width_cm + c->sign_margin_cm - 0.05f;
    for (int j = 0; j < n; ++j) {
        const int side = s[j].color == 1 ? c->pass_side_red : c->pass_side_green;
        float near = 1.0e9f;
        for (int i = 0; i < r->n; ++i) {
            const float d = hypotf(r->x[i] - s[j].x_cm, r->z[i] - s[j].z_cm);
            if (d < near) near = d;
        }
        if (near < rs) printf("FALLA: %s: la linea pasa a %.1f cm del letrero %d (minimo %.1f)\n", tag, near, j, rs), ++failures;
        /* Del lado correcto: la linea, al pasar por el letrero, queda a su derecha (side +1) o a su izquierda. */
        int best = 0;
        float bd = 1.0e9f;
        for (int i = 0; i < r->n; ++i) {
            const float d = hypotf(r->x[i] - s[j].x_cm, r->z[i] - s[j].z_cm);
            if (d < bd) { bd = d; best = i; }
        }
        const float hd = r->heading_deg[best] * 3.14159265f / 180.0f;
        const float lateral = (r->x[best] - s[j].x_cm) * cosf(hd) + (r->z[best] - s[j].z_cm) * (-sinf(hd));
        if (lateral * (float)side <= 0.0f) printf("FALLA: %s: letrero %d pasado por el lado prohibido (%.1f)\n", tag, j, lateral), ++failures;
    }
}

int main(void)
{
    race_config_t cfg;
    race_config_default(&cfg);
    CHECK(race_config_sanitize(&cfg), "la configuracion por defecto es valida");

    /* Campo vacio, los dos sentidos. */
    for (int dir = -1; dir <= 1; dir += 2) {
        CHECK(race_line_build(&line, &cfg, dir, NULL, 0), "linea sin letreros");
        CHECK(line.valid && line.n >= 100 && line.n <= RACE_MAX_STATIONS, "numero de estaciones");
        check_inside(&line, "campo vacio");
        CHECK(line.time_s <= line.time_center_s + 0.01f, "la linea optima no es mas lenta que la central");
        CHECK(line.length_cm > 450.0f && line.length_cm < 900.0f, "longitud de la vuelta");
        CHECK(line.time_s > 15.0f && line.time_s < 45.0f, "tiempo de vuelta plausible");
        float vmin = 1.0e9f, vmax = 0.0f;
        for (int i = 0; i < line.n; ++i) { if (line.v[i] < vmin) vmin = line.v[i]; if (line.v[i] > vmax) vmax = line.v[i]; }
        CHECK(vmin >= 5.0f && vmax <= cfg.v_max_cm_s + 0.01f, "velocidades dentro de los topes");
        CHECK(vmax > cfg.v_corner_cm_s + 5.0f, "hay tramos mas rapidos que la esquina");
        /* Acelera y frena dentro de lo admitido. */
        for (int i = 0; i < line.n; ++i) {
            const int j = (i + 1) % line.n;
            CHECK(line.v[j] * line.v[j] - line.v[i] * line.v[i] <= 2.0f * cfg.a_acc_cm_s2 * line.h[i] + 1.0f, "aceleracion acotada");
            CHECK(line.v[i] * line.v[i] - line.v[j] * line.v[j] <= 2.0f * cfg.a_dec_cm_s2 * line.h[i] + 1.0f, "frenada acotada");
        }
        printf("vacio sentido %+d: %d estaciones, %.0f cm, %.1f s (central %.1f s), mezcla %.1f\n", dir, line.n, line.length_cm,
               line.time_s, line.time_center_s, line.mix);
    }

    /* Letreros reales: rojo pasa por la derecha, verde por la izquierda. */
    const race_sign_t signs[] = {
        {-40.0f, -105.0f, 1}, {30.0f, -95.0f, 2}, {105.0f, -20.0f, 2}, {95.0f, 50.0f, 1},
        {40.0f, 105.0f, 1}, {-30.0f, 95.0f, 2}, {-105.0f, 30.0f, 1}, {-95.0f, -40.0f, 2},
    };
    const int ns = (int)(sizeof(signs) / sizeof(signs[0]));
    for (int dir = -1; dir <= 1; dir += 2) {
        CHECK(race_line_build(&line, &cfg, dir, signs, ns), "linea con letreros");
        check_inside(&line, "con letreros");
        check_signs(&line, signs, ns, "con letreros");
        CHECK(line.time_s <= line.time_center_s + 0.01f, "con letreros: no mas lenta que la central");
        /* Junto a un letrero la velocidad no pasa del tope de la zona. */
        for (int j = 0; j < ns; ++j) {
            int best = 0;
            float bd = 1.0e9f;
            for (int i = 0; i < line.n; ++i) {
                const float d = hypotf(line.x[i] - signs[j].x_cm, line.z[i] - signs[j].z_cm);
                if (d < bd) { bd = d; best = i; }
            }
            CHECK(line.v[best] <= cfg.v_sign_cm_s + 0.01f, "velocidad en un letrero");
        }
        printf("8 letreros sentido %+d: %.0f cm, %.1f s (central %.1f s), mezcla %.1f\n", dir, line.length_cm, line.time_s,
               line.time_center_s, line.mix);
    }

    /* Consulta: cada estacion se localiza a si misma, tambien con ventana. */
    CHECK(race_line_build(&line, &cfg, -1, signs, ns), "linea para consultas");
    for (int i = 0; i < line.n; i += 7) {
        float lat = 9.0f, dist = 9.0f;
        const int g = race_line_locate(&line, line.x[i], line.z[i], -1, 0.0f, &lat, &dist);
        CHECK(g == i && dist < 0.01f && fabsf(lat) < 0.01f, "localizar sin ventana");
        const int w = race_line_locate(&line, line.x[i] + 1.0f, line.z[i] - 1.0f, (i + 3) % line.n, 40.0f, NULL, NULL);
        CHECK(w >= 0 && abs(w - i) <= 2, "localizar con ventana");
    }
    {
        const int i = 10;
        const int j = race_line_advance(&line, i, 50.0f);
        float travelled = 0.0f;
        for (int k = i; k != j; k = (k + 1) % line.n) travelled += line.h[k];
        CHECK(travelled <= 50.0f && travelled > 50.0f - 2.0f * 6.0f, "avanzar a lo largo de la linea");
        float px, pz, hd;
        race_line_point(&line, i, 50.0f, &px, &pz, &hd);
        CHECK(isfinite(px) && isfinite(pz) && isfinite(hd), "punto a lo largo de la linea");
        CHECK(race_line_speed(&line, i, 20.0f) > 0.0f, "velocidad por delante");
        CHECK(race_line_locate(&line, 0.0f, 0.0f, -1, 0.0f, NULL, NULL) >= 0, "localizar desde el centro del campo");
    }

    /* Disenos aleatorios como los del reglamento: hasta 2 letreros por recta (a 0,4 y 0,8 de la recta), a -20, 0 o +20 del centro. */
    {
        unsigned rng = 12345u;
        int built = 0, tried = 0;
        for (int layout = 0; layout < 120; ++layout) {
            race_sign_t s[RACE_MAX_SIGNS];
            int ns2 = 0;
            for (int side = 0; side < 4; ++side) {
                for (int k = 0; k < 2; ++k) {
                    rng = rng * 1664525u + 1013904223u;
                    if (((rng >> 16) & 3u) == 0u) continue;
                    rng = rng * 1664525u + 1013904223u;
                    const float along = -100.0f + 200.0f * (k == 0 ? 0.4f : 0.8f);
                    const float lat = (float)((int)((rng >> 16) % 3u) - 1) * 20.0f - 100.0f;
                    rng = rng * 1664525u + 1013904223u;
                    float x = along, z = lat;
                    for (int q = 0; q < side; ++q) { const float nx = -z, nz = x; x = nx; z = nz; }   /* gira 90 grados */
                    s[ns2].x_cm = x; s[ns2].z_cm = z; s[ns2].color = (uint8_t)(1 + ((rng >> 16) & 1u));
                    ++ns2;
                }
            }
            for (int dir = -1; dir <= 1; dir += 2) {
                ++tried;
                if (!race_line_build(&line, &cfg, dir, s, ns2)) continue;
                ++built;
                float mh = 0.0f;
                for (int i = 0; i < line.n; ++i) if (line.h[i] > mh) mh = line.h[i];
                if (mh > 16.0f) printf("FALLA: diseno %d sentido %d: salto de %.1f cm entre estaciones\n", layout, dir, mh), ++failures;
                check_inside(&line, "aleatorio");
                check_signs(&line, s, ns2, "aleatorio");
                if (line.time_s > line.time_center_s + 0.01f) printf("FALLA: diseno %d: mas lenta que la central\n", layout), ++failures;
            }
        }
        printf("aleatorios: %d de %d lineas construidas\n", built, tried);
        CHECK(built * 10 >= tried * 7, "la mayoria de los disenos tiene linea");
    }

    /* Distancia a esquina: 0 en una esquina, 1 lejos, lineal entre medias. */
    {
        int c0 = -1;
        for (int i = 0; i < line.n && c0 < 0; ++i) if (line.corner[i]) c0 = i;
        CHECK(c0 >= 0, "la linea tiene estaciones de esquina");
        CHECK(race_line_corner_factor(&line, line.s[c0], 50.0f, 40.0f) < 0.01f, "factor 0 en la esquina");
        /* Una recta lejos de las esquinas (mitad de la primera recta de la linea central) vale 1. */
        int far_i = -1;
        float far_d = 0.0f;
        for (int i = 0; i < line.n; ++i) {
            float d = 1.0e9f;
            for (int k = 0; k < line.n; ++k) if (line.corner[k]) { float q = fabsf(line.s[k] - line.s[i]); if (q > 0.5f * line.length_cm) q = line.length_cm - q; if (q < d) d = q; }
            if (d > far_d) { far_d = d; far_i = i; }
        }
        CHECK(far_d > 60.0f && race_line_corner_factor(&line, line.s[far_i], 50.0f, 40.0f) > 0.99f, "factor 1 lejos de las esquinas");
        const float mid = race_line_corner_factor(&line, line.s[c0] - 25.0f, 50.0f, 40.0f);
        CHECK(mid > 0.3f && mid < 0.7f, "factor intermedio a mitad del tramo");
    }

    /* Sin paso: un rojo y un verde en el mismo sitio cierran los dos lados. */
    const race_sign_t blocked[] = {{0.0f, -100.0f, 1}, {0.0f, -100.0f, 2}};
    CHECK(!race_line_build(&line, &cfg, -1, blocked, 2), "un paso imposible se rechaza");
    CHECK(!line.valid && race_line_locate(&line, 0.0f, 0.0f, -1, 0.0f, NULL, NULL) == -1, "sin linea valida no se consulta");

    if (failures) { printf("%d fallos\n", failures); return 1; }
    printf("PASS: test_race_line\n");
    return 0;
}

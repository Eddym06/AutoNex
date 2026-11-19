#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arena_localizer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FALLA: %s (linea %d)\n", msg, __LINE__); ++failures; } } while (0)

static unsigned long long rng_state = 88172645463325252ULL;
static double urand(void)
{
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) / 9007199254740992.0;
}
static double gauss(void)
{
    const double u = urand() + 1e-12, v = urand();
    return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

typedef struct { double x0, z0, x1, z1; } seg_t;
static seg_t segs[64];
static int nseg;

static void add_seg(double x0, double z0, double x1, double z1) { segs[nseg++] = (seg_t){x0, z0, x1, z1}; }
static void add_box(double cx, double cz, double h)
{
    add_seg(cx - h, cz - h, cx + h, cz - h); add_seg(cx + h, cz - h, cx + h, cz + h);
    add_seg(cx + h, cz + h, cx - h, cz + h); add_seg(cx - h, cz + h, cx - h, cz - h);
}
static void build_arena(double O, double I)
{
    nseg = 0;
    add_box(0, 0, 0);            /* placeholder para mantener el indice 0..3 libres */
    nseg = 0;
    add_seg(-O, -O, O, -O); add_seg(O, -O, O, O); add_seg(O, O, -O, O); add_seg(-O, O, -O, -O);
    add_seg(-I, -I, I, -I); add_seg(I, -I, I, I); add_seg(I, I, -I, I); add_seg(-I, I, -I, -I);
}

static double cast(double px, double pz, double dx, double dz)
{
    double best = 1e9;
    for (int i = 0; i < nseg; ++i) {
        const double sx = segs[i].x1 - segs[i].x0, sz = segs[i].z1 - segs[i].z0;
        const double den = dx * sz - dz * sx;
        if (fabs(den) < 1e-12) continue;
        const double t = ((segs[i].x0 - px) * sz - (segs[i].z0 - pz) * sx) / den;
        const double u = ((segs[i].x0 - px) * dz - (segs[i].z0 - pz) * dx) / den;
        if (t > 0.0 && u >= 0.0 && u <= 1.0 && t < best) best = t;
    }
    return best;
}

/* Un barrido de 360 rayos desde el sensor (lidar_forward delante del eje) con ruido. */
static void scan(double x, double z, double th_deg, double lf, double noise, double maxr, arena_point_t *out)
{
    const double th = th_deg * M_PI / 180.0;
    const double sx = x + lf * sin(th), sz = z + lf * cos(th);
    for (int i = 0; i < 360; ++i) {
        const double a = (i - 180) * M_PI / 180.0;
        const double d = cast(sx, sz, sin(th + a), cos(th + a));
        out[i].angle_deg = (float)(i - 180);
        out[i].valid = d < maxr;
        out[i].distance_cm = (float)(d + gauss() * noise);
    }
}

static double wrap(double a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }

int main(void)
{
    arena_config_t cfg; arena_config_default(&cfg);
    arena_point_t pts[360];
    const double LF = 3.0;

    /* 1. Adquisicion en la seccion de salida, mirando al este y al oeste, con ruido. */
    {
        int ok = 0, total = 0;
        double worst_p = 0, worst_t = 0;
        for (int dirn = 0; dirn < 2; ++dirn)
            for (int k = 0; k < 30; ++k) {
                build_arena(150, 50);
                const double x = -120 + 240 * urand(), z = -125 + 50 * urand();
                const double th = (dirn == 0 ? 90.0 : -90.0) + (urand() - 0.5) * 6.0;
                scan(x, z, th, LF, 0.5, 300.0, pts);
                arena_loc_t a; arena_init(&a, &cfg);
                ++total;
                if (!arena_acquire(&a, pts, 360, (float)LF)) continue;
                const double ep = hypot(a.pose.x_cm - x, a.pose.z_cm - z), et = fabs(wrap(a.pose.theta_deg - th));
                if (ep < 3.0 && et < 1.5) ++ok;
                if (ep > worst_p) worst_p = ep;
                if (et > worst_t) worst_t = et;
            }
        printf("adquisicion: %d/%d, peor error %.1f cm / %.1f grados\n", ok, total, worst_p, worst_t);
        CHECK(ok >= total - 1, "adquiere la pose inicial (este y oeste)");
    }

    /* 2. Adquisicion con letreros y topes en el carril (valores atipicos). */
    {
        int ok = 0, total = 0;
        for (int k = 0; k < 20; ++k) {
            build_arena(150, 50);
            add_box(-60 + 20 * urand(), -105, 2.5); add_box(30, -90 - 10 * urand(), 2.5);
            add_seg(100, -148, 100, -130); add_seg(120, -148, 120, -130);          /* topes del cajon */
            const double x = -120 + 150 * urand(), z = -125 + 50 * urand(), th = 90.0 + (urand() - 0.5) * 6.0;
            scan(x, z, th, LF, 0.5, 300.0, pts);
            arena_loc_t a; arena_init(&a, &cfg);
            ++total;
            if (arena_acquire(&a, pts, 360, (float)LF) && hypot(a.pose.x_cm - x, a.pose.z_cm - z) < 3.5 &&
                fabs(wrap(a.pose.theta_deg - th)) < 2.0) ++ok;
        }
        printf("adquisicion con letreros: %d/%d\n", ok, total);
        CHECK(ok >= total - 2, "adquiere con letreros y topes");
    }

    /* 3. Seguimiento: la pose previa tiene un error y la correccion lo reduce. */
    {
        build_arena(150, 50);
        add_box(0, -100, 2.5);
        double worst_p = 0, worst_t = 0;
        for (int k = 0; k < 40; ++k) {
            const double x = -100 + 200 * urand(), z = -125 + 50 * urand(), th = 90.0 + (urand() - 0.5) * 20.0;
            arena_loc_t a; arena_init(&a, &cfg);
            a.valid = true;
            a.pose = (arena_pose_t){(float)(x + 8 * (urand() - 0.5) * 2), (float)(z + 6 * (urand() - 0.5) * 2), (float)(th + 3 * (urand() - 0.5) * 2)};
            for (int it = 0; it < 6; ++it) {
                scan(x, z, th, LF, 0.5, 300.0, pts);
                arena_update(&a, pts, 360, (float)LF);
            }
            const double ep = hypot(a.pose.x_cm - x, a.pose.z_cm - z), et = fabs(wrap(a.pose.theta_deg - th));
            if (ep > 2.0) printf("  caso malo: real (%.0f,%.0f,%.0f) est (%.1f,%.1f,%.1f) soportes x=%.0f z=%.0f rot=%.0f inl=%d/%d\n", x, z, th, a.pose.x_cm, a.pose.z_cm, a.pose.theta_deg, a.sup_x, a.sup_z, a.sup_rot, a.inliers, a.used);
            if (ep > worst_p) worst_p = ep;
            if (et > worst_t) worst_t = et;
        }
        printf("seguimiento: peor error %.2f cm / %.2f grados\n", worst_p, worst_t);
        CHECK(worst_p < 2.0 && worst_t < 1.0, "converge a la pose real");
    }

    /* 4. Se mueve con la odometria y se mantiene pegada a la verdad aunque la odometria derive. */
    {
        build_arena(150, 50);
        add_box(-20, -100, 2.5);
        arena_loc_t a; arena_init(&a, &cfg);
        double x = -130, z = -100, th = 90.0;
        scan(x, z, th, LF, 0.5, 300.0, pts);
        CHECK(arena_acquire(&a, pts, 360, (float)LF), "adquiere antes de moverse");
        double worst = 0;
        for (int step = 0; step < 100; ++step) {
            const double ds = 2.0;
            x += ds * sin(th * M_PI / 180); z += ds * cos(th * M_PI / 180);
            /* La odometria cree que avanzo 1 % de mas y que el rumbo deriva 0,05 grados por paso. */
            arena_predict(&a, (float)(ds * 1.01), (float)(a.pose.theta_deg + 0.05));
            scan(x, z, th, LF, 0.5, 300.0, pts);
            arena_update(&a, pts, 360, (float)LF);
            const double e = hypot(a.pose.x_cm - x, a.pose.z_cm - z);
            if (step > 5 && e > worst) worst = e;
        }
        printf("seguimiento en marcha (200 cm): peor error %.2f cm\n", worst);
        CHECK(worst < 3.0, "no se desvia con odometria imperfecta");
    }

    /* 5. Un barrido basura no mueve la pose. */
    {
        build_arena(150, 50);
        arena_loc_t a; arena_init(&a, &cfg);
        a.valid = true; a.pose = (arena_pose_t){-50.0f, -100.0f, 90.0f};
        for (int i = 0; i < 360; ++i) { pts[i].angle_deg = (float)(i - 180); pts[i].distance_cm = (float)(20 + 200 * urand()); pts[i].valid = true; }
        const arena_pose_t before = a.pose;
        arena_update(&a, pts, 360, (float)LF);
        CHECK(hypot(a.pose.x_cm - before.x_cm, a.pose.z_cm - before.z_cm) < cfg.max_shift_cm + 1e-3, "basura: no salta mas del limite");
        arena_loc_t b; arena_init(&b, &cfg);
        CHECK(!arena_acquire(&b, pts, 360, (float)LF), "basura: no adquiere");
        arena_point_t none[10]; memset(none, 0, sizeof(none));
        CHECK(!arena_acquire(&b, none, 10, (float)LF), "pocos puntos: no adquiere");
    }

    /* 6. Sin pared al frente (alcance corto) el eje longitudinal no se corrige, el lateral si. */
    {
        build_arena(150, 50);
        arena_loc_t a; arena_init(&a, &cfg);
        a.valid = true;
        const double x = 0, z = -100, th = 90.0;
        a.pose = (arena_pose_t){(float)(x + 10), (float)(z + 5), (float)th};
        for (int it = 0; it < 6; ++it) { scan(x, z, th, LF, 0.3, 100.0, pts); arena_update(&a, pts, 360, (float)LF); }
        printf("sin pared al frente: error x %.1f cm (no observable), z %.1f cm\n", a.pose.x_cm - x, a.pose.z_cm - z);
        CHECK(fabs(a.pose.z_cm - z) < 2.0, "el lateral se corrige");
        CHECK(fabs(a.pose.x_cm - (x + 10)) < 3.5, "lo no observable no se inventa");
    }

    /* 7. Configuracion fuera de rango. */
    {
        arena_config_t c; arena_config_default(&c);
        c.outer_half_cm = -5; c.inner_half_cm = 900; c.gain = 7; c.min_support = 0;
        CHECK(arena_config_sanitize(&c), "sanea");
        CHECK(c.outer_half_cm >= 60 && c.inner_half_cm <= c.outer_half_cm - 30 && c.gain <= 1 && c.min_support >= 3, "valores acotados");
    }

    if (failures) { printf("localizador del campo: %d fallos\n", failures); return 1; }
    printf("PASS: localizador del campo (adquisicion, letreros como atipicos, seguimiento, odometria imperfecta, eje no observable, basura)\n");
    return 0;
}

#include "imu_yaw.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    imu_yaw_t y;
    imu_yaw_init(&y);

    /* Con el carrito quieto se mide el sesgo: 0,6 grados por segundo de deriva. */
    imu_yaw_begin_calibration(&y);
    uint64_t t = 0;
    for (int i = 0; i < 150; ++i) { t += 10000; imu_yaw_update(&y, t, 0.6f, NAN); }
    assert(!imu_yaw_end_calibration(&y, 400));          /* muy pocas muestras */
    imu_yaw_begin_calibration(&y);
    for (int i = 0; i < 150; ++i) { t += 10000; imu_yaw_update(&y, t, 0.6f, NAN); }
    assert(imu_yaw_end_calibration(&y, 100));
    assert(fabsf(y.bias_dps - 0.6f) < 0.01f);

    /* Con el sesgo restado, un minuto quieto no mueve el rumbo. */
    imu_yaw_set(&y, 0.0f);
    for (int i = 0; i < 6000; ++i) { t += 10000; imu_yaw_update(&y, t, 0.6f, NAN); }
    assert(fabsf(y.yaw_deg) < 0.2f);

    /* Giro horario de 90 grados/s durante 1 s: 90 grados. */
    imu_yaw_set(&y, 0.0f);
    for (int i = 0; i < 100; ++i) { t += 10000; imu_yaw_update(&y, t, 90.6f, NAN); }
    assert(fabsf(y.yaw_deg - 90.0f) < 1.0f);
    /* El rumbo da la vuelta a +-180. */
    for (int i = 0; i < 100; ++i) { t += 10000; imu_yaw_update(&y, t, 90.6f, NAN); }
    assert(fabsf(fabsf(y.yaw_deg) - 180.0f) < 1.5f);

    /* Una pausa del bus (un segundo sin muestras) no inventa un giro. */
    imu_yaw_set(&y, 10.0f);
    t += 1000000; imu_yaw_update(&y, t, 100.0f, NAN);
    assert(fabsf(y.yaw_deg - 10.0f) < 1e-3f);
    /* Lecturas imposibles se descartan. */
    t += 10000; imu_yaw_update(&y, t, 9000.0f, NAN);
    assert(fabsf(y.yaw_deg - 10.0f) < 1e-3f);

    /* Reanclaje contra las paredes: la referencia absoluta corrige la deriva. */
    imu_yaw_set(&y, 0.0f);
    for (int i = 0; i < 20; ++i) imu_yaw_correct(&y, 8.0f, 0.25f);
    assert(fabsf(y.yaw_deg - 8.0f) < 0.2f);
    imu_yaw_set(&y, 179.0f);
    imu_yaw_correct(&y, -179.0f, 0.5f);               /* el camino corto cruza +-180 */
    assert(fabsf(fabsf(y.yaw_deg) - 180.0f) < 0.01f);

    /* El magnetometro es opcional: sin ganancia no influye. */
    imu_yaw_set(&y, 0.0f);
    t += 10000; imu_yaw_update(&y, t, 0.6f, 90.0f);
    assert(fabsf(y.yaw_deg) < 0.1f);
    y.mag_gain = 0.5f;
    t += 10000; imu_yaw_update(&y, t, 0.6f, 90.0f);
    assert(y.yaw_deg > 30.0f);
    puts("PASS: giroscopio (sesgo medido, deriva, vuelta a +-180, pausas, reanclaje contra paredes)");
    return 0;
}

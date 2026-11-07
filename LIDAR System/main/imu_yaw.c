#include "imu_yaw.h"

#include <math.h>
#include <string.h>

float imu_wrap_deg(float deg)
{
    if (!isfinite(deg)) return 0.0f;
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void imu_yaw_init(imu_yaw_t *y)
{
    if (!y) return;
    memset(y, 0, sizeof(*y));
    y->max_rate_dps = 2000.0f;
}

void imu_yaw_begin_calibration(imu_yaw_t *y)
{
    if (!y) return;
    y->calibrating = true;
    y->bias_sum = 0.0;
    y->bias_count = 0;
}

bool imu_yaw_end_calibration(imu_yaw_t *y, uint32_t min_samples)
{
    if (!y) return false;
    y->calibrating = false;
    if (y->bias_count < min_samples || y->bias_count == 0) return false;
    y->bias_dps = (float)(y->bias_sum / (double)y->bias_count);
    y->bias_valid = true;
    return true;
}

void imu_yaw_update(imu_yaw_t *y, uint64_t t_us, float gyro_dps, float mag_heading_deg)
{
    if (!y || !isfinite(gyro_dps) || fabsf(gyro_dps) > y->max_rate_dps) return;
    if (y->calibrating) {
        y->bias_sum += (double)gyro_dps;
        ++y->bias_count;
    }
    if (y->has_last && t_us > y->last_us) {
        const float dt = (float)(t_us - y->last_us) * 1.0e-6f;
        /* Una pausa larga entre muestras (el bus se colgo) no se integra: el
         * rumbo se quedaria con un salto inventado. */
        if (dt < 0.2f) {
            y->rate_dps = gyro_dps - y->bias_dps;
            y->yaw_deg = imu_wrap_deg(y->yaw_deg + y->rate_dps * dt);
        }
    }
    y->last_us = t_us;
    y->has_last = true;
    if (y->mag_gain > 0.0f && isfinite(mag_heading_deg))
        imu_yaw_correct(y, mag_heading_deg, y->mag_gain);
}

void imu_yaw_correct(imu_yaw_t *y, float measured_deg, float gain)
{
    if (!y || !isfinite(measured_deg) || !(gain > 0.0f)) return;
    if (gain > 1.0f) gain = 1.0f;
    y->yaw_deg = imu_wrap_deg(y->yaw_deg + gain * imu_wrap_deg(measured_deg - y->yaw_deg));
}

void imu_yaw_set(imu_yaw_t *y, float yaw_deg)
{
    if (y) y->yaw_deg = imu_wrap_deg(yaw_deg);
}

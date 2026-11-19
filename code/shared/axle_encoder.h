#ifndef AXLE_ENCODER_H
#define AXLE_ENCODER_H

#include <stdint.h>

/* Cuentas x4 del motorreductor convertidas a grados medios del diferencial. */
static inline float axle_encoder_degrees(int32_t counts, float counts_per_output_rev,
                                         float drive_teeth, float driven_teeth, int sign)
{
    if (!(counts_per_output_rev > 0) || !(drive_teeth > 0) || !(driven_teeth > 0)) return 0.0f;
    return (sign < 0 ? -1.0f : 1.0f) * (float)counts * 360.0f * drive_teeth /
           (counts_per_output_rev * driven_teeth);
}
#endif

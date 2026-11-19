#include "axle_encoder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    assert(fabsf(axle_encoder_degrees(440, 440, 1, 1, 1) - 360.0f) < 0.001f);
    assert(fabsf(axle_encoder_degrees(440, 440, 10, 20, 1) - 180.0f) < 0.001f);
    assert(fabsf(axle_encoder_degrees(-880, 440, 10, 20, 1) + 360.0f) < 0.001f);
    assert(fabsf(axle_encoder_degrees(880, 440, 10, 20, -1) + 360.0f) < 0.001f);
    assert(axle_encoder_degrees(10, 440, 0, 20, 1) == 0);
    puts("PASS: encoder A/B y relación de engranajes");
    return 0;
}

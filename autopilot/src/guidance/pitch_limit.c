#include "pitch_limit.h"

#include <math.h>
#include <stddef.h>

bool ap_guidance_limit_pitch(float requested_rad, float max_pitch_rad,
                             float *effective_rad, bool *limited)
{
    if (effective_rad == NULL || limited == NULL ||
        !isfinite(requested_rad) || !isfinite(max_pitch_rad) ||
        max_pitch_rad <= 0.0f || max_pitch_rad >= 1.570796327f) return false;

    float effective = requested_rad;
    if (effective > max_pitch_rad) effective = max_pitch_rad;
    if (effective < -max_pitch_rad) effective = -max_pitch_rad;
    *effective_rad = effective;
    *limited = effective != requested_rad;
    return true;
}

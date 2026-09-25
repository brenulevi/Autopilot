#include "bank.h"

#include <math.h>
#include <stddef.h>

bool ap_guidance_limit_bank(float requested_rad, float max_bank_rad,
                            float *effective_rad, bool *limited)
{
    if (effective_rad == NULL || limited == NULL ||
        !isfinite(requested_rad) || !isfinite(max_bank_rad) ||
        max_bank_rad <= 0.0f || max_bank_rad >= 1.570796327f) {
        return false;
    }

    float effective = requested_rad;
    if (effective > max_bank_rad) effective = max_bank_rad;
    if (effective < -max_bank_rad) effective = -max_bank_rad;
    *effective_rad = effective;
    *limited = effective != requested_rad;
    return true;
}

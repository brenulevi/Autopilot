#ifndef AUTOPILOT_INTERNAL_GUIDANCE_BANK_LIMIT_H
#define AUTOPILOT_INTERNAL_GUIDANCE_BANK_LIMIT_H

#include "autopilot/autopilot.h"

bool ap_guidance_limit_bank(float requested_rad, float max_bank_rad,
                            float *effective_rad, bool *limited);

#endif

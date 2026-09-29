#ifndef AUTOPILOT_INTERNAL_GUIDANCE_PITCH_LIMIT_H
#define AUTOPILOT_INTERNAL_GUIDANCE_PITCH_LIMIT_H

#include "autopilot/autopilot.h"

bool ap_guidance_limit_pitch(float requested_rad, float max_pitch_rad,
                             float *effective_rad, bool *limited);

#endif

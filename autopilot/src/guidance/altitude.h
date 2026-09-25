#ifndef AUTOPILOT_INTERNAL_GUIDANCE_ALTITUDE_H
#define AUTOPILOT_INTERNAL_GUIDANCE_ALTITUDE_H

#include "autopilot/autopilot.h"

bool ap_guidance_altitude(const ap_config_t *config, const ap_input_t *input,
                          float *pitch_command_rad, bool *limited);

#endif

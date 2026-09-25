#ifndef AUTOPILOT_INTERNAL_CONTROL_ROLL_H
#define AUTOPILOT_INTERNAL_CONTROL_ROLL_H

#include "autopilot/autopilot.h"

bool ap_roll_compute(const ap_config_t *config, const ap_state_t *state,
                     float bank_command_rad, float trim_aileron,
                     float *aileron, bool *saturated);

#endif

#ifndef AUTOPILOT_INTERNAL_CONTROL_PITCH_H
#define AUTOPILOT_INTERNAL_CONTROL_PITCH_H

#include "autopilot/autopilot.h"

bool ap_pitch_compute(const ap_config_t *config, const ap_state_t *state,
                      float pitch_command_rad, float trim_elevator,
                      float *elevator, bool *saturated);

#endif

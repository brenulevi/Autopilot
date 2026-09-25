#ifndef AUTOPILOT_INTERNAL_CONTROL_AIRSPEED_H
#define AUTOPILOT_INTERNAL_CONTROL_AIRSPEED_H

#include "autopilot/autopilot.h"

bool ap_airspeed_compute(const ap_config_t *config, const ap_input_t *input,
                         ap_runtime_t *runtime, ap_output_t *result);

#endif

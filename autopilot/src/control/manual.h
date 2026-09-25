#ifndef AUTOPILOT_INTERNAL_CONTROL_MANUAL_H
#define AUTOPILOT_INTERNAL_CONTROL_MANUAL_H

#include "autopilot/autopilot.h"

void ap_manual_bound(const ap_controls_t *requested, ap_output_t *result);

#endif

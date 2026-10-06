#ifndef AUTOPILOT_CONTROL_PITCH_H
#define AUTOPILOT_CONTROL_PITCH_H
#include "autopilot/control/pitch_attitude.h"
#include "autopilot/control/pitch_rate.h"

typedef struct {
    ap_pitch_attitude_config_t attitude;
    ap_pitch_rate_config_t rate;
} ap_pitch_config_t;
#endif

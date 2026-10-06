#ifndef AUTOPILOT_CONTROL_ROLL_H
#define AUTOPILOT_CONTROL_ROLL_H
#include "autopilot/control/roll_attitude.h"
#include "autopilot/control/roll_rate.h"

typedef struct {
    ap_roll_attitude_config_t attitude;
    ap_roll_rate_config_t rate;
} ap_roll_config_t;
#endif

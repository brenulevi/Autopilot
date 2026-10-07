#include "autopilot/control/airspeed.h"

#include <math.h>
#include <stddef.h>

bool control_airspeed_step(float target_airspeed_m_s,
                           float measured_airspeed_m_s,
                           float dt_s,
                           const control_airspeed_config_t *airspeed,
                           const control_throttle_config_t *throttle,
                           control_airspeed_state_t *state,
                           float *throttle_command)
{
    if (airspeed == NULL || throttle == NULL || state == NULL ||
        throttle_command == NULL ||
        !isfinite(target_airspeed_m_s) || target_airspeed_m_s < 0.0f ||
        !isfinite(measured_airspeed_m_s) || measured_airspeed_m_s < 0.0f ||
        !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(airspeed->proportional_gain) || airspeed->proportional_gain <= 0.0f ||
        !isfinite(airspeed->integral_gain) || airspeed->integral_gain < 0.0f ||
        !isfinite(throttle->trim) || !isfinite(throttle->min_command) ||
        !isfinite(throttle->max_command) ||
        throttle->min_command < 0.0f || throttle->max_command > 1.0f ||
        throttle->min_command > throttle->trim ||
        throttle->trim > throttle->max_command ||
        throttle->min_command >= throttle->max_command ||
        !isfinite(state->integral_command)) return false;

    const float error = target_airspeed_m_s - measured_airspeed_m_s;
    const float proportional = airspeed->proportional_gain * error;
    const float proposed_integral = state->integral_command +
        airspeed->integral_gain * error * dt_s;
    const float proposed_command = throttle->trim + proportional + proposed_integral;
    if (!isfinite(proposed_command) || !isfinite(proposed_integral)) return false;

    float integral = proposed_integral;
    if ((proposed_command > throttle->max_command && error > 0.0f) ||
        (proposed_command < throttle->min_command && error < 0.0f))
        integral = state->integral_command;

    const float demand = throttle->trim + proportional + integral;
    if (!isfinite(demand)) return false;
    float bounded = demand;
    if (bounded < throttle->min_command) bounded = throttle->min_command;
    if (bounded > throttle->max_command) bounded = throttle->max_command;

    state->integral_command = integral;
    *throttle_command = bounded;
    return true;
}

#include "roll.h"

#include <math.h>
#include <stddef.h>

bool ap_roll_compute(const ap_config_t *config, const ap_state_t *state,
                     float bank_command_rad, float trim_aileron,
                     float *aileron, bool *saturated)
{
    if (config == NULL || state == NULL || aileron == NULL || saturated == NULL ||
        !isfinite(config->roll_angle_gain) || config->roll_angle_gain <= 0.0f ||
        !isfinite(config->roll_rate_gain) || config->roll_rate_gain < 0.0f ||
        !isfinite(config->max_aileron) || config->max_aileron <= 0.0f ||
        config->max_aileron > 1.0f) {
        return false;
    }

    /* Proportional angle error plus damping from measured body roll rate.
     * Body p is not exactly the derivative of Euler bank in every maneuver. */
    const float demand = trim_aileron
        + config->roll_angle_gain * (bank_command_rad - state->roll_rad)
        - config->roll_rate_gain * state->p_rad_s;
    if (!isfinite(demand)) return false;

    float bounded = demand;
    if (bounded > config->max_aileron) bounded = config->max_aileron;
    if (bounded < -config->max_aileron) bounded = -config->max_aileron;
    *aileron = bounded;
    *saturated = bounded != demand;
    return true;
}

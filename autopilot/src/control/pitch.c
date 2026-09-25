#include "pitch.h"

#include <math.h>
#include <stddef.h>

bool ap_pitch_compute(const ap_config_t *config, const ap_state_t *state,
                      float pitch_command_rad, float trim_elevator,
                      float *elevator, bool *saturated)
{
    if (config == NULL || state == NULL || elevator == NULL || saturated == NULL ||
        !isfinite(config->pitch_angle_gain) || config->pitch_angle_gain <= 0.0f ||
        !isfinite(config->pitch_rate_gain) || config->pitch_rate_gain < 0.0f ||
        !isfinite(config->max_elevator) || config->max_elevator <= 0.0f ||
        config->max_elevator > 1.0f) return false;

    /* Negative elevator produces nose-up pitch in the C172X model. */
    const float demand = trim_elevator
        - config->pitch_angle_gain * (pitch_command_rad - state->pitch_rad)
        + config->pitch_rate_gain * state->q_rad_s;
    if (!isfinite(demand)) return false;

    float bounded = demand;
    if (bounded > config->max_elevator) bounded = config->max_elevator;
    if (bounded < -config->max_elevator) bounded = -config->max_elevator;
    *elevator = bounded;
    *saturated = bounded != demand;
    return true;
}

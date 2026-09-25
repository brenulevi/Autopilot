#include "airspeed.h"

#include <math.h>
#include <stddef.h>

static float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

bool ap_airspeed_compute(const ap_config_t *config, const ap_input_t *input,
                         ap_runtime_t *runtime, ap_output_t *result)
{
    if (config == NULL || input == NULL || runtime == NULL || result == NULL ||
        !isfinite(config->airspeed_kp) || config->airspeed_kp < 0.0f ||
        !isfinite(config->airspeed_ki) || config->airspeed_ki <= 0.0f ||
        !isfinite(config->min_throttle) || config->min_throttle < 0.0f ||
        config->min_throttle >= 1.0f ||
        !isfinite(config->max_throttle) || config->max_throttle <= config->min_throttle ||
        config->max_throttle > 1.0f ||
        !isfinite(input->airspeed_command_m_s) || input->airspeed_command_m_s <= 0.0f ||
        input->state.airspeed_m_s <= 0.0f ||
        !isfinite(runtime->airspeed_integral_norm)) return false;

    const float error = input->airspeed_command_m_s - input->state.airspeed_m_s;
    const float proportional = config->airspeed_kp * error;
    const float integral_step = config->airspeed_ki * error * input->dt_s;
    const float proposed_integral = runtime->airspeed_integral_norm + integral_step;
    const float proposed = input->requested.throttle + proportional + proposed_integral;
    if (!isfinite(proposed) || !isfinite(proposed_integral)) return false;

    /* Do not integrate farther into an active limit; allow integration back out. */
    const bool outward_high = proposed > config->max_throttle && error > 0.0f;
    const bool outward_low = proposed < config->min_throttle && error < 0.0f;
    const float integral = outward_high || outward_low ? runtime->airspeed_integral_norm : proposed_integral;
    const float demand = input->requested.throttle + proportional + integral;
    if (!isfinite(demand)) return false;
    const float bounded = clamp(demand, config->min_throttle, config->max_throttle);

    runtime->airspeed_integral_norm = integral;
    result->controls.throttle = bounded;
    result->airspeed_command_m_s = input->airspeed_command_m_s;
    result->airspeed_error_m_s = error;
    result->throttle_saturated = bounded != demand;
    return true;
}

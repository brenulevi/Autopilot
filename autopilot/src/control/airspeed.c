#include "autopilot/control/airspeed.h"

#include <math.h>
#include <stddef.h>

static float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

bool ap_airspeed_compute(const ap_airspeed_config_t *config,
                         const ap_airspeed_input_t *input,
                         ap_airspeed_runtime_t *runtime,
                         ap_airspeed_output_t *output)
{
    if (config == NULL || input == NULL || runtime == NULL || output == NULL ||
        !isfinite(config->kp) || config->kp < 0.0f ||
        !isfinite(config->ki) || config->ki <= 0.0f ||
        !isfinite(config->min_throttle_norm) || config->min_throttle_norm < 0.0f ||
        config->min_throttle_norm >= 1.0f ||
        !isfinite(config->max_throttle_norm) || config->max_throttle_norm <= config->min_throttle_norm ||
        config->max_throttle_norm > 1.0f ||
        !isfinite(input->target_airspeed_m_s) || input->target_airspeed_m_s <= 0.0f ||
        !isfinite(input->measured_airspeed_m_s) || input->measured_airspeed_m_s <= 0.0f ||
        !isfinite(input->trim_throttle_norm) ||
        !isfinite(input->dt_s) || input->dt_s <= 0.0f ||
        !isfinite(runtime->integral_norm)) return false;

    const float error = input->target_airspeed_m_s - input->measured_airspeed_m_s;
    const float proportional = config->kp * error;
    const float integral_step = config->ki * error * input->dt_s;
    const float proposed_integral = runtime->integral_norm + integral_step;
    const float proposed = input->trim_throttle_norm + proportional + proposed_integral;
    if (!isfinite(proposed) || !isfinite(proposed_integral)) return false;

    /* Do not integrate farther into an active limit; allow integration back out. */
    const bool outward_high = proposed > config->max_throttle_norm && error > 0.0f;
    const bool outward_low = proposed < config->min_throttle_norm && error < 0.0f;
    const float integral = outward_high || outward_low ? runtime->integral_norm : proposed_integral;
    const float demand = input->trim_throttle_norm + proportional + integral;
    if (!isfinite(demand)) return false;
    const float bounded = clamp(demand, config->min_throttle_norm, config->max_throttle_norm);

    const ap_airspeed_output_t result = {bounded, error, bounded != demand};
    runtime->integral_norm = integral;
    *output = result;
    return true;
}

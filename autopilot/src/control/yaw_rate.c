#include "autopilot/control/yaw_rate.h"
#include <math.h>
#include <stddef.h>

static float bound(float value, float limit)
{ return value < -limit ? -limit : (value > limit ? limit : value); }

bool ap_yaw_rate_compute(const ap_yaw_rate_config_t *config,
                         const ap_yaw_rate_input_t *input,
                         ap_yaw_rate_runtime_t *runtime,
                         ap_yaw_rate_output_t *output)
{
    if (config == NULL || input == NULL || runtime == NULL || output == NULL ||
        !isfinite(config->kp) || config->kp <= 0.0f ||
        !isfinite(config->ki) || config->ki < 0.0f ||
        !isfinite(config->max_rudder_norm) || config->max_rudder_norm <= 0.0f || config->max_rudder_norm > 1.0f ||
        (config->rudder_sign != 1.0f && config->rudder_sign != -1.0f) ||
        !isfinite(input->measured_body_rate_rad_s) || !isfinite(input->target_body_rate_rad_s) ||
        !isfinite(input->trim_rudder_norm) || !isfinite(input->dt_s) || input->dt_s <= 0.0f ||
        !isfinite(runtime->integral_norm) || fabsf(runtime->integral_norm) > config->max_rudder_norm)
        return false;
    const float error = input->target_body_rate_rad_s - input->measured_body_rate_rad_s;
    const float proportional = config->kp * error;
    const float raw_integral = runtime->integral_norm + config->ki * error * input->dt_s;
    if (!isfinite(proportional) || !isfinite(raw_integral)) return false;
    const float candidate = bound(raw_integral, config->max_rudder_norm);
    const float proposed = input->trim_rudder_norm + config->rudder_sign * (proportional + candidate);
    if (!isfinite(proposed)) return false;
    const float actuator_error = config->rudder_sign * error;
    const bool outward = (proposed > config->max_rudder_norm && actuator_error > 0.0f) ||
                         (proposed < -config->max_rudder_norm && actuator_error < 0.0f);
    const float integral = outward ? runtime->integral_norm : candidate;
    const float demand = input->trim_rudder_norm + config->rudder_sign * (proportional + integral);
    if (!isfinite(demand)) return false;
    const float bounded = bound(demand, config->max_rudder_norm);
    const ap_yaw_rate_output_t result = {bounded, error, bounded != demand};
    runtime->integral_norm = integral;
    *output = result;
    return true;
}

#include "autopilot/control/pitch_attitude.h"
#include <math.h>
#include <stddef.h>

bool ap_pitch_attitude_compute(const ap_pitch_attitude_config_t *config,
                              const ap_pitch_attitude_input_t *input,
                              ap_pitch_attitude_output_t *output)
{
    if (config == NULL || input == NULL || output == NULL ||
        !isfinite(config->gain) || config->gain <= 0.0f ||
        !isfinite(config->max_rate_rad_s) || config->max_rate_rad_s <= 0.0f ||
        !isfinite(input->measured_pitch_rad) || !isfinite(input->target_pitch_rad)) return false;
    const float requested = config->gain * (input->target_pitch_rad - input->measured_pitch_rad);
    if (!isfinite(requested)) return false;
    float bounded = requested;
    if (bounded > config->max_rate_rad_s) bounded = config->max_rate_rad_s;
    if (bounded < -config->max_rate_rad_s) bounded = -config->max_rate_rad_s;
    const ap_pitch_attitude_output_t result = {bounded, bounded != requested};
    *output = result;
    return true;
}

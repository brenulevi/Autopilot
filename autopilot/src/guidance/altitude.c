#include "autopilot/guidance/altitude.h"

#include <math.h>
#include <stddef.h>

bool ap_guidance_altitude(const ap_altitude_config_t *config,
                          const ap_altitude_input_t *input,
                          ap_altitude_output_t *output)
{
    if (config == NULL || input == NULL || output == NULL ||
        !isfinite(config->altitude_gain) || config->altitude_gain <= 0.0f ||
        !isfinite(config->climb_rate_gain) || config->climb_rate_gain < 0.0f ||
        !isfinite(config->max_pitch_offset_rad) || config->max_pitch_offset_rad <= 0.0f ||
        config->max_pitch_offset_rad >= 1.570796327f ||
        !isfinite(input->target_altitude_m) || !isfinite(input->trim_pitch_rad) ||
        !isfinite(input->measured_altitude_m) ||
        !isfinite(input->measured_climb_rate_m_s)) return false;

    const float error = input->target_altitude_m - input->measured_altitude_m;
    const float raw_offset = config->altitude_gain * error
        - config->climb_rate_gain * input->measured_climb_rate_m_s;
    if (!isfinite(raw_offset)) return false;
    float offset = raw_offset;
    if (offset > config->max_pitch_offset_rad) offset = config->max_pitch_offset_rad;
    if (offset < -config->max_pitch_offset_rad) offset = -config->max_pitch_offset_rad;
    const float command = input->trim_pitch_rad + offset;
    if (!isfinite(command)) return false;
    const ap_altitude_output_t result = {command, offset != raw_offset};
    *output = result;
    return true;
}

#include "altitude.h"

#include <math.h>
#include <stddef.h>

bool ap_guidance_altitude(const ap_config_t *config, const ap_input_t *input,
                          float *pitch_command_rad, bool *limited)
{
    if (config == NULL || input == NULL || pitch_command_rad == NULL || limited == NULL ||
        !isfinite(config->altitude_gain) || config->altitude_gain <= 0.0f ||
        !isfinite(config->climb_rate_gain) || config->climb_rate_gain < 0.0f ||
        !isfinite(config->max_pitch_offset_rad) || config->max_pitch_offset_rad <= 0.0f ||
        config->max_pitch_offset_rad >= 1.570796327f ||
        !isfinite(input->altitude_command_m) || !isfinite(input->pitch_command_rad)) return false;

    const float error = input->altitude_command_m - input->state.altitude_m;
    const float raw_offset = config->altitude_gain * error
        - config->climb_rate_gain * input->state.climb_rate_m_s;
    if (!isfinite(raw_offset)) return false;
    float offset = raw_offset;
    if (offset > config->max_pitch_offset_rad) offset = config->max_pitch_offset_rad;
    if (offset < -config->max_pitch_offset_rad) offset = -config->max_pitch_offset_rad;
    const float command = input->pitch_command_rad + offset;
    if (!isfinite(command)) return false;
    *pitch_command_rad = command;
    *limited = offset != raw_offset;
    return true;
}

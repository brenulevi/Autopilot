#include "autopilot/control/roll.h"

#include <math.h>
#include <stddef.h>

bool control_roll_step(float target_bank_rad, float measured_bank_rad,
                       float measured_roll_rate_rad_s,
                       const control_roll_step_config_t *roll,
                       const control_surface_config_t *aileron,
                       float *aileron_command)
{
    if (roll == NULL || aileron == NULL || aileron_command == NULL ||
        !isfinite(target_bank_rad) || !isfinite(measured_bank_rad) ||
        !isfinite(measured_roll_rate_rad_s) ||
        !isfinite(roll->angle_gain) || roll->angle_gain <= 0.0f ||
        !isfinite(roll->rate_gain) || roll->rate_gain < 0.0f ||
        !isfinite(aileron->trim) || !isfinite(aileron->min_command) ||
        !isfinite(aileron->max_command) ||
        aileron->min_command < -1.0f || aileron->max_command > 1.0f ||
        aileron->min_command >= aileron->trim ||
        aileron->trim >= aileron->max_command) {
        return false;
    }

    const float demand = aileron->trim
        + roll->angle_gain * (target_bank_rad - measured_bank_rad)
        - roll->rate_gain * measured_roll_rate_rad_s;
    if (!isfinite(demand)) return false;

    float bounded = demand;
    if (bounded < aileron->min_command) bounded = aileron->min_command;
    if (bounded > aileron->max_command) bounded = aileron->max_command;
    *aileron_command = bounded;
    return true;
}

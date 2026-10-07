#include "autopilot/control/pitch.h"

#include <math.h>
#include <stddef.h>

bool control_pitch_step(float target_pitch_rad, float measured_pitch_rad,
                        float measured_pitch_rate_rad_s,
                        const control_pitch_step_config_t *pitch,
                        const control_surface_config_t *elevator,
                        float *elevator_command)
{
    if (pitch == NULL || elevator == NULL || elevator_command == NULL ||
        !isfinite(target_pitch_rad) || !isfinite(measured_pitch_rad) ||
        !isfinite(measured_pitch_rate_rad_s) ||
        !isfinite(pitch->angle_gain) || pitch->angle_gain <= 0.0f ||
        !isfinite(pitch->rate_gain) || pitch->rate_gain < 0.0f ||
        !isfinite(elevator->trim) || !isfinite(elevator->min_command) ||
        !isfinite(elevator->max_command) ||
        elevator->min_command < -1.0f || elevator->max_command > 1.0f ||
        elevator->min_command >= elevator->trim ||
        elevator->trim >= elevator->max_command) {
        return false;
    }

    const float demand = elevator->trim
        - pitch->angle_gain * (target_pitch_rad - measured_pitch_rad)
        + pitch->rate_gain * measured_pitch_rate_rad_s;
    if (!isfinite(demand)) return false;

    float bounded = demand;
    if (bounded < elevator->min_command) bounded = elevator->min_command;
    if (bounded > elevator->max_command) bounded = elevator->max_command;
    *elevator_command = bounded;
    return true;
}

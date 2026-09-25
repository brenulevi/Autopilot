#include "manual.h"

static float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

void ap_manual_bound(const ap_controls_t *requested, ap_output_t *result)
{
    result->controls.aileron = clamp(requested->aileron, -1.0f, 1.0f);
    result->controls.elevator = clamp(requested->elevator, -1.0f, 1.0f);
    result->controls.rudder = clamp(requested->rudder, -1.0f, 1.0f);
    result->controls.throttle = clamp(requested->throttle, 0.0f, 1.0f);
    result->aileron_saturated = result->controls.aileron != requested->aileron;
    result->elevator_saturated = result->controls.elevator != requested->elevator;
    result->throttle_saturated = result->controls.throttle != requested->throttle;
}

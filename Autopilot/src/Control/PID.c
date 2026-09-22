#include "Autopilot/Control/PID.h"

#include <float.h>
#include <math.h>

static float PID_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

void PID_Init(PID_t * pid)
{
    pid->gains.kp = 0.0f;
    pid->gains.ki = 0.0f;
    pid->gains.kd = 0.0f;
    pid->output_min = -FLT_MAX;
    pid->output_max = FLT_MAX;
    pid->integral_min = -FLT_MAX;
    pid->integral_max = FLT_MAX;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->initialized = 0;
}

void PID_SetGains(PID_t *pid, PID_Gains_t gains)
{
    pid->gains = gains;
}

void PID_SetOutputLimits(PID_t *pid, float minimum, float maximum)
{
    if (minimum > maximum)
    {
        return;
    }

    pid->output_min = minimum;
    pid->output_max = maximum;
}

void PID_SetIntegralLimits(PID_t *pid, float minimum, float maximum)
{
    if (minimum > maximum)
    {
        return;
    }

    pid->integral_min = minimum;
    pid->integral_max = maximum;
    pid->integral = PID_Clamp(pid->integral, minimum, maximum);
}

float PID_Step(PID_t *pid, float target, float measurement, float dt)
{
    float error = target - measurement;
    float derivative = 0.0f;
    float new_integral;
    float unclamped_output;
    float output;

    if (!isfinite(target) || !isfinite(measurement) || !isfinite(dt) || dt <= 0.0f)
    {
        return 0.0f;
    }

    if (pid->initialized)
    {
        derivative = -(measurement - pid->previous_measurement) / dt;
    }

    new_integral = PID_Clamp(
        pid->integral + error * dt,
        pid->integral_min,
        pid->integral_max);

    unclamped_output = pid->gains.kp * error
        + pid->gains.ki * new_integral
        + pid->gains.kd * derivative;
    output = PID_Clamp(unclamped_output, pid->output_min, pid->output_max);

    if (output == unclamped_output
        || (output >= pid->output_max && error < 0.0f)
        || (output <= pid->output_min && error > 0.0f))
    {
        pid->integral = new_integral;
    }

    pid->previous_measurement = measurement;
    pid->initialized = 1;

    return PID_Clamp(
        pid->gains.kp * error
            + pid->gains.ki * pid->integral
            + pid->gains.kd * derivative,
        pid->output_min,
        pid->output_max);
}

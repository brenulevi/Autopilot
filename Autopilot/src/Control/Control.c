#include "Autopilot/Control/Control.h"

void Control_Init(Control_t* control)
{
    PID_Init(&control->pitch_pid);
    PID_Init(&control->roll_pid);
}

void Control_SetPitchGains(Control_t* control, PID_Gains_t gains)
{
    PID_SetGains(&control->pitch_pid, gains);
}

void Control_SetRollGains(Control_t* control, PID_Gains_t gains)
{
    PID_SetGains(&control->roll_pid, gains);
}

void Control_SetPitchOutputLimits(Control_t* control, float minimum, float maximum)
{
    PID_SetOutputLimits(&control->pitch_pid, minimum, maximum);
}

void Control_SetRollOutputLimits(Control_t* control, float minimum, float maximum)
{
    PID_SetOutputLimits(&control->roll_pid, minimum, maximum);
}

void Control_SetPitchIntegralLimits(Control_t* control, float minimum, float maximum)
{
    PID_SetIntegralLimits(&control->pitch_pid, minimum, maximum);
}

void Control_SetRollIntegralLimits(Control_t* control, float minimum, float maximum)
{
    PID_SetIntegralLimits(&control->roll_pid, minimum, maximum);
}

void Control_ResetPitch(Control_t* control)
{
    PID_Reset(&control->pitch_pid);
}

void Control_ResetRoll(Control_t* control)
{
    PID_Reset(&control->roll_pid);
}

float Control_StepPitch(Control_t* control, float target_pitch, float current_pitch, float dt)
{
    return PID_Step(&control->pitch_pid, target_pitch, current_pitch, dt);
}

float Control_StepRoll(Control_t* control, float target_roll, float current_roll, float dt)
{
    return PID_Step(&control->roll_pid, target_roll, current_roll, dt);
}
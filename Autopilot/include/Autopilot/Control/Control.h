#ifndef AUTOPILOT_CONTROL_H
#define AUTOPILOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Autopilot/Control/PID.h"

typedef struct
{
    PID_t pitch_pid;
    PID_t roll_pid;
} Control_t;

void Control_Init(Control_t* control);

void Control_SetPitchGains(Control_t* control, PID_Gains_t gains);
void Control_SetRollGains(Control_t* control, PID_Gains_t gains);

void Control_SetPitchOutputLimits(Control_t* control, float minimum, float maximum);
void Control_SetRollOutputLimits(Control_t* control, float minimum, float maximum);

void Control_SetPitchIntegralLimits(Control_t* control, float minimum, float maximum);
void Control_SetRollIntegralLimits(Control_t* control, float minimum, float maximum);

void Control_ResetPitch(Control_t* control);
void Control_ResetRoll(Control_t* control);

float Control_StepPitch(Control_t* control, float target_pitch, float current_pitch, float dt);
float Control_StepRoll(Control_t* control, float target_roll, float current_roll, float dt);

#ifdef __cplusplus
}
#endif

#endif // AUTOPILOT_CONTROL_H
#ifndef AUTOPILOT_PID_H
#define AUTOPILOT_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
} PID_Gains_t;

typedef struct
{
    PID_Gains_t gains;

    float integral;
    float previous_measurement;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    int initialized;
} PID_t;

void PID_Init(PID_t *pid);
void PID_SetGains(PID_t *pid, PID_Gains_t gains);
void PID_SetOutputLimits(PID_t *pid, float minimum, float maximum);
void PID_SetIntegralLimits(PID_t *pid, float minimum, float maximum);
void PID_Reset(PID_t *pid);
float PID_Step(PID_t *pid, float target, float current, float dt);

#ifdef __cplusplus
}
#endif

#endif // AUTOPILOT_PID_H
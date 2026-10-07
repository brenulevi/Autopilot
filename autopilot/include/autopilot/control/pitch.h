#ifndef AUTOPILOT_CONTROL_PITCH_H
#define AUTOPILOT_CONTROL_PITCH_H

#include <stdbool.h>
#include "autopilot/control/surface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle_gain; /* Normalized elevator command per radian of pitch error. */
    float rate_gain;  /* Normalized elevator command per radian/second of pitch rate. */
} control_pitch_step_config_t;

/* Logical negative elevator requests nose-up pitch, matching the C172X.
 * Pitch rate is positive nose-up and damps motion through the positive term:
 * trim - Kp * (target - pitch) + Kd * pitch_rate.
 * Body pitch rate is only approximately the derivative of Euler pitch angle.
 * Returns false for invalid inputs and leaves elevator_command unchanged. */
bool control_pitch_step(float target_pitch_rad, float measured_pitch_rad,
                        float measured_pitch_rate_rad_s,
                        const control_pitch_step_config_t *pitch,
                        const control_surface_config_t *elevator,
                        float *elevator_command);

#ifdef __cplusplus
}
#endif

#endif

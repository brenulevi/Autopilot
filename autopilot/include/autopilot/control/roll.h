#ifndef AUTOPILOT_CONTROL_ROLL_H
#define AUTOPILOT_CONTROL_ROLL_H

#include <stdbool.h>
#include "autopilot/control/surface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle_gain; /* Normalized aileron command per radian of bank error. */
    float rate_gain;  /* Normalized aileron command per radian/second of roll rate. */
} control_roll_step_config_t;

/* Positive aileron and positive bank/roll rate mean right roll.
 * The rate term damps motion: trim + Kp * (target - bank) - Kd * roll_rate.
 * Returns false for invalid inputs and leaves aileron_command unchanged. */
bool control_roll_step(float target_bank_rad, float measured_bank_rad,
                       float measured_roll_rate_rad_s,
                       const control_roll_step_config_t *roll,
                       const control_surface_config_t *aileron,
                       float *aileron_command);

#ifdef __cplusplus
}
#endif

#endif

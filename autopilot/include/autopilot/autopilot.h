#ifndef AUTOPILOT_AUTOPILOT_H
#define AUTOPILOT_AUTOPILOT_H

#include <stdbool.h>
#include "autopilot/control/roll.h"
#include "autopilot/control/pitch.h"
#include "autopilot/control/airspeed.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Aircraft settings and controller gains, owned by the caller. */
typedef struct {
    control_roll_step_config_t roll;
    control_pitch_step_config_t pitch;
    control_airspeed_config_t airspeed;
    control_surface_config_t aileron;
    control_surface_config_t elevator;
    control_throttle_config_t throttle;
} autopilot_config_t;

/* One sample from the adapter. Angles use radians, rates radians/s, and
 * airspeed m/s. dt_s is the elapsed time since the previous sample. */
typedef struct {
    float target_bank_rad;
    float target_pitch_rad;
    float measured_bank_rad;
    float measured_pitch_rad;
    float measured_roll_rate_rad_s;
    float measured_pitch_rate_rad_s;
    float target_airspeed_m_s;
    float measured_airspeed_m_s;
    float dt_s;
} autopilot_input_t;

/* Initialize to {0} when engaging airspeed hold. Caller owns this state. */
typedef struct {
    control_airspeed_state_t airspeed;
} autopilot_state_t;

typedef struct {
    float aileron_command; /* Logical command normalized to [-1, 1]. */
    float elevator_command; /* Logical command normalized to [-1, 1]. */
    float throttle_command; /* Logical command normalized to [0, 1]. */
} autopilot_output_t;

/* One roll, pitch, and airspeed update. There is no flight mode or mission
 * behavior yet. Returns false if any controller rejects the sample and leaves
 * state and all outputs unchanged. */
bool autopilot_step(const autopilot_config_t *config,
                    const autopilot_input_t *input,
                    autopilot_state_t *state,
                    autopilot_output_t *output);

#ifdef __cplusplus
}
#endif

#endif

#ifndef AUTOPILOT_CONTROL_AIRSPEED_H
#define AUTOPILOT_CONTROL_AIRSPEED_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Gains use m/s for airspeed and normalized throttle in [0, 1]. */
typedef struct {
    float proportional_gain; /* Throttle command per m/s of speed error. */
    float integral_gain;     /* Throttle command per (m/s * s) of speed error. */
} control_airspeed_config_t;

typedef struct {
    float trim;
    float min_command;
    float max_command;
} control_throttle_config_t;

/* Initialize to {0} when engaging or resetting airspeed hold. */
typedef struct {
    float integral_command;
} control_airspeed_state_t;

/* PI airspeed hold: trim + Kp * error + integral. The integrator stops growing
 * when the throttle is saturated in the direction of the speed error.
 * Returns false for invalid input and leaves state and output unchanged. */
bool control_airspeed_step(float target_airspeed_m_s,
                           float measured_airspeed_m_s,
                           float dt_s,
                           const control_airspeed_config_t *airspeed,
                           const control_throttle_config_t *throttle,
                           control_airspeed_state_t *state,
                           float *throttle_command);

#ifdef __cplusplus
}
#endif

#endif

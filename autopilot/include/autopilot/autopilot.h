#ifndef AUTOPILOT_AUTOPILOT_H
#define AUTOPILOT_AUTOPILOT_H

#include <stdbool.h>
#include "autopilot/control/actuators.h"
#include "autopilot/estimation/state.h"
#include "autopilot/control/roll.h"
#include "autopilot/control/pitch.h"
#include "autopilot/control/airspeed.h"
#include "autopilot/guidance/altitude.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AP_MODE_MANUAL = 0,
    AP_MODE_ROLL_HOLD = 1,
    AP_MODE_PITCH_HOLD = 2,
    AP_MODE_ATTITUDE_HOLD = 3,
    AP_MODE_ATTITUDE_AIRSPEED_HOLD = 4,
    AP_MODE_ALTITUDE_AIRSPEED_HOLD = 5
} ap_mode_t;

/* Aircraft-specific tuning belongs to the caller, not this library.
 * Positive roll gains assume positive aileron produces positive body roll rate.
 * Pitch gains assume negative elevator produces positive body pitch rate. */
typedef struct {
    float max_bank_rad; /* command limit, (0, pi/2) */
    float max_pitch_rad; /* command limit, (0, pi/2) */
} ap_attitude_limits_t;

typedef struct {
    ap_roll_config_t roll;
    ap_pitch_config_t pitch;
    ap_airspeed_config_t airspeed;
    ap_altitude_config_t altitude;
    ap_attitude_limits_t attitude_limits;
} ap_config_t;

typedef struct {
    ap_state_t state;
    ap_controls_t requested; /* Manual demand, or trim feedforward in closed-loop modes. */
    float dt_s;
    ap_mode_t mode;
    float bank_command_rad; /* Used in roll, attitude, or altitude hold. */
    float pitch_command_rad; /* Absolute attitude, or trim reference in altitude hold. */
    float airspeed_command_m_s; /* True airspeed, used in stateful hold modes. */
    float altitude_command_m; /* MSL altitude for altitude + airspeed hold. */
} ap_input_t;

/* Caller-owned PI state. Initialize to {0}; reused on successive steps. */
typedef struct {
    ap_roll_rate_runtime_t roll;
    ap_pitch_rate_runtime_t pitch;
    ap_airspeed_runtime_t airspeed;
} ap_runtime_t;

/* Caller-owned controller instance. Configuration is copied at initialization;
 * runtime is retained across ticks. Use the functions below to initialize/update.
 * Public storage enables static/stack allocation; fields are not opaque. */
typedef struct {
    ap_config_t config;
    ap_runtime_t runtime;
} ap_controller_t;

typedef struct {
    ap_controls_t controls;
    float bank_command_rad; /* Effective limited command; zero in manual mode. */
    bool bank_command_limited;
    bool aileron_saturated;
    float pitch_command_rad; /* Effective limited command; zero outside pitch/attitude/altitude hold. */
    bool pitch_command_limited;
    bool elevator_saturated;
    float airspeed_command_m_s;
    float airspeed_error_m_s;
    bool throttle_saturated;
    float altitude_command_m;
    float altitude_error_m;
    bool altitude_pitch_limited;
    float roll_rate_command_rad_s;
    float pitch_rate_command_rad_s;
    float roll_rate_error_rad_s;
    float pitch_rate_error_rad_s;
    bool roll_rate_limited;
    bool pitch_rate_limited;
} ap_output_t;

/* Guidance limits attitude targets; attitude P loops produce limited body-rate
 * targets; roll/pitch PI rate loops produce signed actuator demands. These outer
 * loops use a near-level attitude/body-rate approximation without yaw coupling.
 * Roll/pitch integrals persist while their axes are active; inactive axes reset.
 * Airspeed PI persists in airspeed/altitude modes and resets in other modes.
 * Manual mode clears all integrals and ignores config (which may be NULL).
 * dt_s must be positive and finite. Zero-initialize caller-owned runtime before
 * the first call. No allocation, OS, simulator, or peripheral dependencies.
 * Failure leaves runtime and output unchanged; caller defines the failsafe. */
bool ap_step(const ap_config_t *config, const ap_input_t *input,
             ap_runtime_t *runtime, ap_output_t *output);

/* Checks all control settings, independently of storage/navigation metadata. */
bool ap_control_config_validate(const ap_config_t *config);

/* Copies configuration and clears runtime. Full configuration must be valid;
 * failure leaves controller unchanged. No allocation or I/O. */
bool ap_controller_init(ap_controller_t *controller, const ap_config_t *config);

/* Instance entry point; same modes and transactional failure behavior as ap_step.
 * Configuration remains unchanged; runtime is updated only on success. */
bool ap_controller_step(ap_controller_t *controller, const ap_input_t *input,
                        ap_output_t *output);

#ifdef __cplusplus
}
#endif
#endif

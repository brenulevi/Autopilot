#ifndef AUTOPILOT_AUTOPILOT_H
#define AUTOPILOT_AUTOPILOT_H

#include <stdbool.h>
#include "autopilot/control/actuators.h"
#include "autopilot/estimation/state.h"

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
    float roll_angle_gain; /* normalized command / rad, strictly positive */
    float roll_rate_gain;  /* normalized command / (rad/s), nonnegative */
    float max_bank_rad;    /* command limit, (0, pi/2) */
    float max_aileron;     /* absolute normalized command limit, (0, 1] */
    float pitch_angle_gain; /* normalized elevator / rad, strictly positive */
    float pitch_rate_gain;  /* normalized elevator / (rad/s), nonnegative */
    float max_pitch_rad;    /* absolute pitch command limit, (0, pi/2) */
    float max_elevator;     /* absolute normalized command limit, (0, 1] */
    float airspeed_kp;      /* normalized throttle / (m/s), nonnegative */
    float airspeed_ki;      /* normalized throttle / (m/s*s), strictly positive */
    float min_throttle;     /* normalized lower limit, [0, 1) */
    float max_throttle;     /* normalized upper limit, (min_throttle, 1] */
    float altitude_gain;     /* pitch-command rad / m, strictly positive */
    float climb_rate_gain;   /* pitch-command rad / (m/s), nonnegative */
    float max_pitch_offset_rad; /* altitude-loop offset limit, (0, pi/2) */
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
    float airspeed_integral_norm;
} ap_runtime_t;

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
} ap_output_t;

/* Stateless entry point: manual, individual axis, or combined attitude hold.
 * aileron = trim + angle_gain * (bank_command - roll) - rate_gain * p.
 * This is PD-like, not a PID: there is no integral state. Body p is not
 * identical to the derivative of Euler bank except near simple attitudes.
 * Pitch assumes negative elevator command produces positive pitch acceleration:
 * elevator = trim - angle_gain * (pitch_command - pitch) + rate_gain * q.
 * Verify that sign on each aircraft before using pitch hold.
 * No integrator or hidden state across mode switches. dt_s is
 * validated for the interface but not used by this algebraic control law.
 * config is required in closed-loop modes and ignored (may be NULL) in manual mode.
 * Airspeed and altitude modes require ap_step_with_runtime instead.
 * No allocation, OS, simulator, or peripheral dependencies.
 * Returns false on invalid input and leaves output unchanged. The caller
 * must handle failure; this API does not define an aircraft failsafe. */
bool ap_step(const ap_config_t *config, const ap_input_t *input, ap_output_t *output);

/* Stateful entry point for attitude + true-airspeed or altitude + airspeed hold.
 * Altitude hold creates a limited pitch command from altitude and climb rate.
 * Uses the same attitude laws, plus throttle PI with conditional anti-windup.
 * On a successful step in another mode, clears the airspeed integrator.
 * On any failure, both output and runtime are unchanged. */
bool ap_step_with_runtime(const ap_config_t *config, const ap_input_t *input,
                          ap_runtime_t *runtime, ap_output_t *output);

#ifdef __cplusplus
}
#endif
#endif

#ifndef AUTOPILOT_CONTROL_TURN_COORDINATION_H
#define AUTOPILOT_CONTROL_TURN_COORDINATION_H
#include <stdbool.h>

typedef struct {
    float max_rate_rad_s; /* body yaw-rate reference limit, > 0 */
    float min_airspeed_m_s; /* positive denominator floor, not stall protection */
    float max_bank_rad; /* measured-bank clamp for the steady-turn model, (0, pi/2) */
} ap_turn_coordination_config_t;
typedef struct {
    float measured_bank_rad;
    float measured_pitch_rad;
    float true_airspeed_m_s;
} ap_turn_coordination_input_t;
typedef struct {
    float yaw_rate_command_rad_s;
    float pitch_rate_feedforward_rad_s;
    bool rate_limited;
    bool airspeed_guarded;
    bool bank_limited;
} ap_turn_coordination_output_t;
#ifdef __cplusplus
extern "C" {
#endif
/* Approximate steady coordinated horizontal-turn body rates from measured
 * attitude and TAS: r = g*sin(bank)*cos(pitch)/V; q_ff = r*tan(bank).
 * No heading hold or sideslip feedback. Flags expose model guards.
 * Failure leaves output unchanged. */
bool ap_turn_coordination_compute(const ap_turn_coordination_config_t *config,
                                  const ap_turn_coordination_input_t *input,
                                  ap_turn_coordination_output_t *output);
#ifdef __cplusplus
}
#endif
#endif

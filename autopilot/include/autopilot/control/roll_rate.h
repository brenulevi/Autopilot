#ifndef AUTOPILOT_CONTROL_ROLL_RATE_H
#define AUTOPILOT_CONTROL_ROLL_RATE_H
#include <stdbool.h>

typedef struct {
    float kp; /* normalized effort / (rad/s), strictly positive */
    float ki; /* normalized effort / rad, nonnegative; zero disables integration */
    float max_aileron_norm; /* absolute actuator limit, (0, 1] */
} ap_roll_rate_config_t;

/* Integral is control effort: positive effort increases positive body rate.
 * Zero-initialize, retain across ticks, and reset when this axis is disabled. */
typedef struct { float integral_norm; } ap_roll_rate_runtime_t;

typedef struct {
    float measured_body_rate_rad_s;
    float target_body_rate_rad_s;
    float trim_aileron_norm;
    float dt_s;
} ap_roll_rate_input_t;

typedef struct {
    float aileron_norm;
    float error_rad_s;
    bool saturated;
} ap_roll_rate_output_t;

#ifdef __cplusplus
extern "C" {
#endif
/* PI with conditional anti-windup and integral bounded by actuator authority.
 * Runtime and output are unchanged on failure. Reset runtime after changing
 * gains/authority. The roll actuator sign is 1.0f times control effort. */
bool ap_roll_rate_compute(const ap_roll_rate_config_t *config,
                          const ap_roll_rate_input_t *input,
                          ap_roll_rate_runtime_t *runtime,
                          ap_roll_rate_output_t *output);
#ifdef __cplusplus
}
#endif
#endif

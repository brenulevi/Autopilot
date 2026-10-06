#ifndef AUTOPILOT_CONTROL_YAW_RATE_H
#define AUTOPILOT_CONTROL_YAW_RATE_H
#include <stdbool.h>

typedef struct {
    float kp; /* normalized effort/(rad/s), > 0 */
    float ki; /* normalized effort/rad, >= 0 */
    float max_rudder_norm; /* absolute command authority, (0, 1] */
    float rudder_sign; /* +1 or -1: actuator command producing positive body r */
} ap_yaw_rate_config_t;
typedef struct { float integral_norm; } ap_yaw_rate_runtime_t;
typedef struct {
    float measured_body_rate_rad_s;
    float target_body_rate_rad_s;
    float trim_rudder_norm;
    float dt_s;
} ap_yaw_rate_input_t;
typedef struct {
    float rudder_norm;
    float error_rad_s;
    bool saturated;
} ap_yaw_rate_output_t;
#ifdef __cplusplus
extern "C" {
#endif
/* Caller-owned PI memory, conditional anti-windup, signed actuator effort.
 * Initialize runtime to zero; reset after changing gains/authority/sign.
 * Failure leaves output and runtime unchanged. No I/O or allocation. */
bool ap_yaw_rate_compute(const ap_yaw_rate_config_t *config,
                         const ap_yaw_rate_input_t *input,
                         ap_yaw_rate_runtime_t *runtime,
                         ap_yaw_rate_output_t *output);
#ifdef __cplusplus
}
#endif
#endif

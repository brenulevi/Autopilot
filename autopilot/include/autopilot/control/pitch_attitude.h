#ifndef AUTOPILOT_CONTROL_PITCH_ATTITUDE_H
#define AUTOPILOT_CONTROL_PITCH_ATTITUDE_H
#include <stdbool.h>

typedef struct {
    float gain; /* desired body rate / attitude error, 1/s, strictly positive */
    float max_rate_rad_s; /* absolute desired body-rate limit, strictly positive */
} ap_pitch_attitude_config_t;

typedef struct {
    float measured_pitch_rad;
    float target_pitch_rad; /* Already limited by guidance. */
} ap_pitch_attitude_input_t;

typedef struct {
    float body_rate_command_rad_s;
    bool limited;
} ap_pitch_attitude_output_t;

#ifdef __cplusplus
extern "C" {
#endif
/* Near-level attitude-error to body-rate approximation, without yaw coupling.
 * This does not identify Euler angle derivatives with measured body rates.
 * Stateless; output is unchanged on invalid input or arithmetic overflow. */
bool ap_pitch_attitude_compute(const ap_pitch_attitude_config_t *config,
                              const ap_pitch_attitude_input_t *input,
                              ap_pitch_attitude_output_t *output);
#ifdef __cplusplus
}
#endif
#endif

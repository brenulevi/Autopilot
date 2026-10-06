#ifndef AUTOPILOT_GUIDANCE_ALTITUDE_H
#define AUTOPILOT_GUIDANCE_ALTITUDE_H

#include <stdbool.h>

typedef struct {
    float altitude_gain; /* pitch-command rad / m, strictly positive */
    float climb_rate_gain; /* pitch-command rad / (m/s), nonnegative */
    float max_pitch_offset_rad; /* (0, pi/2) */
} ap_altitude_config_t;

typedef struct {
    float measured_altitude_m; /* MSL, positive up. */
    float measured_climb_rate_m_s;
    float target_altitude_m;
    float trim_pitch_rad;
} ap_altitude_input_t;

typedef struct {
    float pitch_command_rad;
    bool limited;
} ap_altitude_output_t;

#ifdef __cplusplus
extern "C" {
#endif
/* Produces an attitude target; output unchanged on failure. */
bool ap_guidance_altitude(const ap_altitude_config_t *config,
                          const ap_altitude_input_t *input,
                          ap_altitude_output_t *output);
#ifdef __cplusplus
}
#endif
#endif

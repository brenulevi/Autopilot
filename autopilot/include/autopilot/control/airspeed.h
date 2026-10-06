#ifndef AUTOPILOT_CONTROL_AIRSPEED_H
#define AUTOPILOT_CONTROL_AIRSPEED_H

#include <stdbool.h>

typedef struct {
    float kp; /* normalized throttle / (m/s), nonnegative */
    float ki; /* normalized throttle / (m/s*s), strictly positive */
    float min_throttle_norm; /* [0, 1) */
    float max_throttle_norm; /* (min_throttle_norm, 1] */
} ap_airspeed_config_t;

/* Zero-initialize before the first tick; retain between ticks. */
typedef struct {
    float integral_norm;
} ap_airspeed_runtime_t;

typedef struct {
    float measured_airspeed_m_s;
    float target_airspeed_m_s;
    float trim_throttle_norm;
    float dt_s;
} ap_airspeed_input_t;

typedef struct {
    float throttle_norm;
    float error_m_s;
    bool saturated;
} ap_airspeed_output_t;

#ifdef __cplusplus
extern "C" {
#endif
/* PI with conditional integration; runtime and output unchanged on failure. */
bool ap_airspeed_compute(const ap_airspeed_config_t *config,
                         const ap_airspeed_input_t *input,
                         ap_airspeed_runtime_t *runtime,
                         ap_airspeed_output_t *output);
#ifdef __cplusplus
}
#endif
#endif

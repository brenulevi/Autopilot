#ifndef AUTOPILOT_CONTROL_SURFACE_H
#define AUTOPILOT_CONTROL_SURFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Aircraft-specific limits for a logical control command, normalized to [-1, 1].
 * Physical servo travel and PWM limits still belong to the I/O side. */
typedef struct {
    float trim;
    float min_command;
    float max_command;
} control_surface_config_t;

#ifdef __cplusplus
}
#endif

#endif

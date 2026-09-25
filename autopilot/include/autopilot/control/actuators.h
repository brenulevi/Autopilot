#ifndef AUTOPILOT_CONTROL_ACTUATORS_H
#define AUTOPILOT_CONTROL_ACTUATORS_H

/* Surface commands [-1, 1], throttle [0, 1]. These are normalized demands,
 * not surface angles or PWM. The adapter/board driver owns that mapping. */
typedef struct {
    float aileron;
    float elevator;
    float rudder;
    float throttle;
} ap_controls_t;

#endif

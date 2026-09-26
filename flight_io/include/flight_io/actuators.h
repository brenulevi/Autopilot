#ifndef FLIGHT_IO_ACTUATORS_H
#define FLIGHT_IO_ACTUATORS_H

#include "flight_common/control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FIO_MAX_OUTPUTS 8u

/* One physical output: weighted logical demands plus bias, clamped to [-1,1],
 * then mapped piecewise to min/neutral/max microseconds. Put reversal in the
 * weight signs. Multiple rows support dual ailerons, elevons, or V-tail mixing.
 * For a throttle output, neutral_us may equal minimum_us (zero -> minimum).
 * Mixing coefficients and endpoints must be validated for the actual airframe. */
typedef struct {
    float aileron_weight;
    float elevator_weight;
    float rudder_weight;
    float throttle_weight;
    float bias;
    uint16_t minimum_us;
    uint16_t neutral_us;
    uint16_t maximum_us;
} fio_actuator_channel_t;

typedef struct {
    uint8_t count;
    fio_actuator_channel_t channels[FIO_MAX_OUTPUTS];
} fio_actuator_config_t;

typedef struct {
    uint8_t count;
    uint16_t pulse_us[FIO_MAX_OUTPUTS];
    uint32_t saturated_mask;
} fio_actuator_output_t;

bool fio_actuator_config_valid(const fio_actuator_config_t *config);

/* Produces pulse-width demands only, not electrical PWM or output frequency.
 * Rejects invalid logical commands/config and overflow without changing output.
 * Firmware owns output enable/inhibit and timer programming. */
bool fio_actuators_map(const fio_actuator_config_t *config,
                       const flight_controls_t *controls, fio_actuator_output_t *output);

#ifdef __cplusplus
}
#endif
#endif

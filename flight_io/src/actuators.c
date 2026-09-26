#include "flight_io/actuators.h"
#include "internal.h"

bool fio_actuator_config_valid(const fio_actuator_config_t *config)
{
    if (config == NULL || config->count == 0 || config->count > FIO_MAX_OUTPUTS) return false;
    for (unsigned i = 0; i < config->count; ++i) {
        const fio_actuator_channel_t *c = &config->channels[i];
        if (!isfinite(c->aileron_weight) || !isfinite(c->elevator_weight) ||
            !isfinite(c->rudder_weight) || !isfinite(c->throttle_weight) || !isfinite(c->bias) ||
            c->minimum_us == 0 || c->minimum_us >= c->maximum_us ||
            c->neutral_us < c->minimum_us || c->neutral_us > c->maximum_us) return false;
    }
    return true;
}

bool fio_actuators_map(const fio_actuator_config_t *config,
                       const flight_controls_t *controls, fio_actuator_output_t *output)
{
    if (output == NULL || !fio_actuator_config_valid(config) || !fio_controls_valid(controls)) return false;
    fio_actuator_output_t result = {0};
    result.count = config->count;
    for (unsigned i = 0; i < config->count; ++i) {
        const fio_actuator_channel_t *c = &config->channels[i];
        float value = c->bias + c->aileron_weight * controls->aileron + c->elevator_weight * controls->elevator +
            c->rudder_weight * controls->rudder + c->throttle_weight * controls->throttle;
        if (!isfinite(value)) return false;
        if (value > 1.0f) { value = 1.0f; result.saturated_mask |= 1u << i; }
        if (value < -1.0f) { value = -1.0f; result.saturated_mask |= 1u << i; }
        float pulse = c->neutral_us + value *
            (float)(value >= 0.0f ? c->maximum_us - c->neutral_us : c->neutral_us - c->minimum_us);
        result.pulse_us[i] = (uint16_t)(pulse + 0.5f);
    }
    *output = result;
    return true;
}

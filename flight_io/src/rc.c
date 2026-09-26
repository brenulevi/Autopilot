#include "flight_io/rc.h"
#include <stddef.h>

static bool channel_valid(const fio_rc_channel_t *c)
{
    return c->channel < FIO_IBUS_CHANNELS && c->minimum > 0 &&
           c->minimum < c->center && c->center < c->maximum && c->maximum <= 4095;
}

bool fio_rc_config_valid(const fio_rc_config_t *config)
{
    if (config == NULL) return false;
    const fio_rc_channel_t *channels[] = {&config->aileron, &config->elevator,
        &config->rudder, &config->throttle, &config->mode};
    for (unsigned i = 0; i < 5; ++i) {
        if (!channel_valid(channels[i])) return false;
        for (unsigned j = 0; j < i; ++j)
            if (channels[i]->channel == channels[j]->channel) return false;
    }
    return true;
}

static float normalize(const fio_rc_channel_t *c, uint16_t value, bool throttle)
{
    if (throttle) {
        float scaled = (float)(value - c->minimum) / (float)(c->maximum - c->minimum);
        return c->reversed ? 1.0f - scaled : scaled;
    }
    float scaled = ((float)value - c->center) /
        (float)(value >= c->center ? c->maximum - c->center : c->center - c->minimum);
    return c->reversed ? -scaled : scaled;
}

bool fio_rc_map(const fio_rc_config_t *config, const fio_ibus_frame_t *frame,
                bool firmware_link_ok, flight_rc_sample_t *output)
{
    if (frame == NULL || output == NULL || !fio_rc_config_valid(config)) return false;
    flight_rc_sample_t result = {0};
    result.received_at_ms = frame->received_at_ms;
    bool healthy = firmware_link_ok && !frame->failsafe;
    const fio_rc_channel_t *channels[] = {&config->aileron, &config->elevator,
        &config->rudder, &config->throttle, &config->mode};
    for (unsigned i = 0; i < 5; ++i) {
        uint16_t value = frame->channels[channels[i]->channel];
        if (value < channels[i]->minimum || value > channels[i]->maximum) healthy = false;
    }
    if (healthy) {
        result.controls.aileron = normalize(&config->aileron, frame->channels[config->aileron.channel], false);
        result.controls.elevator = normalize(&config->elevator, frame->channels[config->elevator.channel], false);
        result.controls.rudder = normalize(&config->rudder, frame->channels[config->rudder.channel], false);
        result.controls.throttle = normalize(&config->throttle, frame->channels[config->throttle.channel], true);
        float mode = normalize(&config->mode, frame->channels[config->mode.channel], false);
        result.valid = mode <= -0.5f || mode >= 0.5f;
        result.request_auto = mode >= 0.5f;
    }
    *output = result;
    return true;
}

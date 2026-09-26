#ifndef FLIGHT_IO_RC_H
#define FLIGHT_IO_RC_H

#include "flight_common/control.h"
#include "flight_io/ibus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t channel; /* Zero-based transport slot, not a fixed transmitter order. */
    uint16_t minimum;
    uint16_t center;
    uint16_t maximum;
    bool reversed;
} fio_rc_channel_t;

typedef struct {
    fio_rc_channel_t aileron;
    fio_rc_channel_t elevator;
    fio_rc_channel_t rudder;
    fio_rc_channel_t throttle;
    fio_rc_channel_t mode;
} fio_rc_config_t;

bool fio_rc_config_valid(const fio_rc_config_t *config);

/* Piecewise center calibration for surfaces/mode; min..max for throttle.
 * All five assigned channels must be distinct and values inside their
 * calibrated endpoints (include measurement tolerance when configuring).
 * Mode <=-0.5 requests MANUAL; >=0.5 AUTO; the middle band is invalid.
 * firmware_link_ok is a caller-supplied RF-health gate based on verified
 * receiver behavior, e.g. a tested loss-marker channel. Never infer it solely
 * from fresh iBUS frames. Recognized frame failsafe also invalidates the sample.
 * On true, output may have valid=false: publish that invalidation immediately.
 * False means invalid config/pointers; output is unchanged. */
bool fio_rc_map(const fio_rc_config_t *config, const fio_ibus_frame_t *frame,
                bool firmware_link_ok, flight_rc_sample_t *output);

#ifdef __cplusplus
}
#endif
#endif

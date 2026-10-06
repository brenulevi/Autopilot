#ifndef AUTOPILOT_CONFIG_H
#define AUTOPILOT_CONFIG_H

#include "autopilot/autopilot.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* APCF v3: 12-byte header, 28 binary32 values, 4-byte CRC32; little endian.
 * Decoder also accepts v1/v2; legacy records disable yaw coordination.
 * See docs/configuration.md for the stable wire format. No raw struct storage.
 * Requires an IEEE-754 binary32 float target. No allocation or storage I/O. */
#define AP_CONFIG_V1_RECORD_SIZE 80u
#define AP_CONFIG_V2_RECORD_SIZE 96u
#define AP_CONFIG_RECORD_SIZE 128u

typedef struct {
    ap_config_t control;
    float l1_period_s;
    uint32_t sequence; /* Storage metadata; not used by the controllers. */
} ap_aircraft_config_t;

/* Checks numerical/API bounds, not suitability for a physical aircraft. */
bool ap_config_validate(const ap_aircraft_config_t *config);

/* Encode exactly AP_CONFIG_RECORD_SIZE bytes. Remaining capacity is untouched.
 * On failure, the destination is unchanged. Input/output must not overlap. */
bool ap_config_encode(const ap_aircraft_config_t *config, uint8_t *bytes, size_t capacity);

/* Requires exactly one record, a supported version, valid CRC and parameters.
 * On failure, config is unchanged. Input/output must not overlap. */
bool ap_config_decode(const uint8_t *bytes, size_t length, ap_aircraft_config_t *config);

#ifdef __cplusplus
}
#endif
#endif

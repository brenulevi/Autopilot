#include "flight_common/crc32.h"
#include "autopilot/config.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

_Static_assert(CHAR_BIT == 8 && sizeof(float) == 4 && FLT_RADIX == 2 &&
               FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && FLT_MIN_EXP == -125,
               "APCF requires IEEE-754 binary32 floats and 8-bit bytes");

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_u32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8 * i));
}


/* Explicit order is the wire schema; never walk struct memory as an array. */
static void to_fields(const ap_aircraft_config_t *record, float fields[20])
{
    const ap_config_t *c = &record->control;
    fields[0] = c->roll.attitude.gain;
    fields[1] = c->roll.rate.kp;
    fields[2] = c->attitude_limits.max_bank_rad;
    fields[3] = c->roll.rate.max_aileron_norm;
    fields[4] = c->pitch.attitude.gain;
    fields[5] = c->pitch.rate.kp;
    fields[6] = c->attitude_limits.max_pitch_rad;
    fields[7] = c->pitch.rate.max_elevator_norm;
    fields[8] = c->airspeed.kp;
    fields[9] = c->airspeed.ki;
    fields[10] = c->airspeed.min_throttle_norm;
    fields[11] = c->airspeed.max_throttle_norm;
    fields[12] = c->altitude.altitude_gain;
    fields[13] = c->altitude.climb_rate_gain;
    fields[14] = c->altitude.max_pitch_offset_rad;
    fields[15] = record->l1_period_s;
    fields[16] = c->roll.rate.ki;
    fields[17] = c->roll.attitude.max_rate_rad_s;
    fields[18] = c->pitch.rate.ki;
    fields[19] = c->pitch.attitude.max_rate_rad_s;
}

bool ap_control_config_validate(const ap_config_t *config)
{
    if (config == NULL) return false;
    const ap_aircraft_config_t record = {.control = *config};
    float fields[20];
    to_fields(&record, fields);
    for (unsigned i = 0; i < 20; ++i)
        if (i != 15 && !isfinite(fields[i])) return false;
    const ap_config_t *c = config;
    const float half_pi = 1.570796327f;
    return c->roll.attitude.gain > 0.0f && c->roll.rate.kp > 0.0f && c->roll.rate.ki >= 0.0f &&
           c->roll.attitude.max_rate_rad_s > 0.0f &&
           c->attitude_limits.max_bank_rad > 0.0f && c->attitude_limits.max_bank_rad < half_pi &&
           c->roll.rate.max_aileron_norm > 0.0f && c->roll.rate.max_aileron_norm <= 1.0f &&
           c->pitch.attitude.gain > 0.0f && c->pitch.rate.kp > 0.0f && c->pitch.rate.ki >= 0.0f &&
           c->pitch.attitude.max_rate_rad_s > 0.0f &&
           c->attitude_limits.max_pitch_rad > 0.0f && c->attitude_limits.max_pitch_rad < half_pi &&
           c->pitch.rate.max_elevator_norm > 0.0f && c->pitch.rate.max_elevator_norm <= 1.0f &&
           c->airspeed.kp >= 0.0f && c->airspeed.ki > 0.0f &&
           c->airspeed.min_throttle_norm >= 0.0f && c->airspeed.min_throttle_norm < 1.0f &&
           c->airspeed.max_throttle_norm > c->airspeed.min_throttle_norm && c->airspeed.max_throttle_norm <= 1.0f &&
           c->altitude.altitude_gain > 0.0f && c->altitude.climb_rate_gain >= 0.0f &&
           c->altitude.max_pitch_offset_rad > 0.0f && c->altitude.max_pitch_offset_rad < half_pi;
}

bool ap_config_validate(const ap_aircraft_config_t *record)
{
    return record != NULL && ap_control_config_validate(&record->control) &&
           isfinite(record->l1_period_s) &&
           record->l1_period_s >= 1.0f && record->l1_period_s <= 30.0f;
}

bool ap_config_encode(const ap_aircraft_config_t *config, uint8_t *bytes, size_t capacity)
{
    if (bytes == NULL || capacity < AP_CONFIG_RECORD_SIZE || !ap_config_validate(config))
        return false;
    uint8_t encoded[AP_CONFIG_RECORD_SIZE] = {'A', 'P', 'C', 'F', 2, 0, 80, 0};
    write_u32(encoded + 8, config->sequence);
    float fields[20];
    to_fields(config, fields);
    for (unsigned i = 0; i < 20; ++i) {
        uint32_t bits;
        memcpy(&bits, &fields[i], sizeof(bits));
        write_u32(encoded + 12 + 4 * i, bits);
    }
    write_u32(encoded + AP_CONFIG_RECORD_SIZE - 4, flight_crc32(encoded, AP_CONFIG_RECORD_SIZE - 4));
    memcpy(bytes, encoded, sizeof(encoded));
    return true;
}

bool ap_config_decode(const uint8_t *bytes, size_t length, ap_aircraft_config_t *config)
{
    if (bytes == NULL || config == NULL ||
        (length != AP_CONFIG_V1_RECORD_SIZE && length != AP_CONFIG_RECORD_SIZE)) return false;
    const bool legacy = length == AP_CONFIG_V1_RECORD_SIZE;
    if (memcmp(bytes, "APCF", 4) != 0 || bytes[4] != (legacy ? 1 : 2) || bytes[5] != 0 ||
        bytes[6] != (legacy ? 64 : 80) || bytes[7] != 0 ||
        read_u32(bytes + length - 4) != flight_crc32(bytes, length - 4)) return false;
    float f[20] = {0};
    for (unsigned i = 0; i < (legacy ? 16u : 20u); ++i) {
        const uint32_t bits = read_u32(bytes + 12 + 4 * i);
        memcpy(&f[i], &bits, sizeof(bits));
    }
    if (legacy) {
        /* A = K_rate * K_attitude. Old damping must be positive to define
         * a controllable rate loop. Legacy records start without integral action
         * and acquire explicit 1 rad/s roll and 0.5 rad/s pitch target limits. */
        if (!isfinite(f[1]) || f[1] <= 0.0f || !isfinite(f[5]) || f[5] <= 0.0f) return false;
        f[0] /= f[1];
        f[4] /= f[5];
        f[17] = 1.0f;
        f[19] = 0.5f;
    }
    ap_aircraft_config_t next = {
        .control = {
            .roll = {{f[0], f[17]}, {f[1], f[16], f[3]}},
            .pitch = {{f[4], f[19]}, {f[5], f[18], f[7]}},
            .airspeed = {f[8], f[9], f[10], f[11]},
            .altitude = {f[12], f[13], f[14]},
            .attitude_limits = {f[2], f[6]}
        },
        .l1_period_s = f[15],
        .sequence = read_u32(bytes + 8)
    };
    if (!ap_config_validate(&next)) return false;
    *config = next;
    return true;
}

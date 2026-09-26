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

static uint32_t crc32(const uint8_t *bytes, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}

/* Explicit order is the wire schema; never walk struct memory as an array. */
static void to_fields(const ap_aircraft_config_t *record, float fields[16])
{
    const ap_config_t *c = &record->control;
    fields[0] = c->roll_angle_gain;
    fields[1] = c->roll_rate_gain;
    fields[2] = c->max_bank_rad;
    fields[3] = c->max_aileron;
    fields[4] = c->pitch_angle_gain;
    fields[5] = c->pitch_rate_gain;
    fields[6] = c->max_pitch_rad;
    fields[7] = c->max_elevator;
    fields[8] = c->airspeed_kp;
    fields[9] = c->airspeed_ki;
    fields[10] = c->min_throttle;
    fields[11] = c->max_throttle;
    fields[12] = c->altitude_gain;
    fields[13] = c->climb_rate_gain;
    fields[14] = c->max_pitch_offset_rad;
    fields[15] = record->l1_period_s;
}

bool ap_config_validate(const ap_aircraft_config_t *record)
{
    if (record == NULL) return false;
    float fields[16];
    to_fields(record, fields);
    for (unsigned i = 0; i < 16; ++i)
        if (!isfinite(fields[i])) return false;
    const ap_config_t *c = &record->control;
    const float half_pi = 1.570796327f;
    return c->roll_angle_gain > 0.0f && c->roll_rate_gain >= 0.0f &&
           c->max_bank_rad > 0.0f && c->max_bank_rad < half_pi &&
           c->max_aileron > 0.0f && c->max_aileron <= 1.0f &&
           c->pitch_angle_gain > 0.0f && c->pitch_rate_gain >= 0.0f &&
           c->max_pitch_rad > 0.0f && c->max_pitch_rad < half_pi &&
           c->max_elevator > 0.0f && c->max_elevator <= 1.0f &&
           c->airspeed_kp >= 0.0f && c->airspeed_ki > 0.0f &&
           c->min_throttle >= 0.0f && c->min_throttle < 1.0f &&
           c->max_throttle > c->min_throttle && c->max_throttle <= 1.0f &&
           c->altitude_gain > 0.0f && c->climb_rate_gain >= 0.0f &&
           c->max_pitch_offset_rad > 0.0f && c->max_pitch_offset_rad < half_pi &&
           record->l1_period_s >= 1.0f && record->l1_period_s <= 30.0f;
}

bool ap_config_encode(const ap_aircraft_config_t *config, uint8_t *bytes, size_t capacity)
{
    if (bytes == NULL || capacity < AP_CONFIG_RECORD_SIZE || !ap_config_validate(config))
        return false;
    uint8_t encoded[AP_CONFIG_RECORD_SIZE] = {'A', 'P', 'C', 'F', 1, 0, 64, 0};
    write_u32(encoded + 8, config->sequence);
    float fields[16];
    to_fields(config, fields);
    for (unsigned i = 0; i < 16; ++i) {
        uint32_t bits;
        memcpy(&bits, &fields[i], sizeof(bits));
        write_u32(encoded + 12 + 4 * i, bits);
    }
    write_u32(encoded + 76, crc32(encoded, 76));
    memcpy(bytes, encoded, sizeof(encoded));
    return true;
}

bool ap_config_decode(const uint8_t *bytes, size_t length, ap_aircraft_config_t *config)
{
    if (bytes == NULL || config == NULL || length != AP_CONFIG_RECORD_SIZE ||
        memcmp(bytes, "APCF", 4) != 0 || bytes[4] != 1 || bytes[5] != 0 ||
        bytes[6] != 64 || bytes[7] != 0 || read_u32(bytes + 76) != crc32(bytes, 76))
        return false;
    float f[16];
    for (unsigned i = 0; i < 16; ++i) {
        const uint32_t bits = read_u32(bytes + 12 + 4 * i);
        memcpy(&f[i], &bits, sizeof(bits));
    }
    ap_aircraft_config_t next = {
        {f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
         f[8], f[9], f[10], f[11], f[12], f[13], f[14]},
        f[15], read_u32(bytes + 8)
    };
    if (!ap_config_validate(&next)) return false;
    *config = next;
    return true;
}

#include "autopilot/config.h"
#include "flight_common/crc32.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

static void put_u32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) bytes[i] = (uint8_t)(value >> (8 * i));
}

static void update_crc(uint8_t *bytes)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < (AP_CONFIG_RECORD_SIZE - 4); ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    put_u32(bytes + (AP_CONFIG_RECORD_SIZE - 4), ~crc);
}

int main(void)
{
    const ap_aircraft_config_t original = {
        .control = {
            .roll = {{8.0f, 1.0f}, {0.5f, 0.1f, 0.5f}},
            .pitch = {{10.0f / 3.0f, 0.5f}, {3.0f, 0.5f, 0.5f}},
            .airspeed = {0.08f, 0.005f, 0.0f, 1.0f},
            .altitude = {0.015f, 0.05f, 0.052f},
            .attitude_limits = {0.35f, 0.17f},
            .yaw = {true, {0.7f, 0.2f, 0.5f, 1.0f}, {0.3f, 25.0f, 0.785f}}
        },
        .l1_period_s = 4.0f,
        .sequence = 0x12345678u
    };
    uint8_t bytes[AP_CONFIG_RECORD_SIZE + 1];
    memset(bytes, 0xA5, sizeof(bytes));
    CHECK(ap_config_validate(&original));
    CHECK(ap_config_encode(&original, bytes, sizeof(bytes)));
    CHECK(bytes[AP_CONFIG_RECORD_SIZE] == 0xA5);
    const uint8_t header[] = {'A','P','C','F',3,0,112,0,0x78,0x56,0x34,0x12};
    CHECK(memcmp(bytes, header, sizeof(header)) == 0);
    CHECK(bytes[12] == 0 && bytes[13] == 0 && bytes[14] == 0 && bytes[15] == 0x41);
    /* The APCF v3 order is independent of the nested structs: verify every wire field
     * independently, so a symmetric encoder/decoder reorder cannot pass. */
    const float expected_fields[28] = {
        8.0f, 0.5f, 0.35f, 0.5f, 10.0f / 3.0f, 3.0f, 0.17f, 0.5f,
        0.08f, 0.005f, 0.0f, 1.0f, 0.015f, 0.05f, 0.052f, 4.0f, 0.1f, 1.0f, 0.5f, 0.5f,
        1.0f, 0.7f, 0.2f, 0.5f, 1.0f, 0.3f, 25.0f, 0.785f
    };
    for (unsigned field = 0; field < 28; ++field) {
        uint32_t bits;
        uint8_t expected_bytes[4];
        memcpy(&bits, &expected_fields[field], sizeof(bits));
        put_u32(expected_bytes, bits);
        CHECK(memcmp(bytes + 12 + 4 * field, expected_bytes, 4) == 0);
    }
    ap_aircraft_config_t decoded = {0};
    CHECK(ap_config_decode(bytes, AP_CONFIG_RECORD_SIZE, &decoded));
    CHECK(decoded.sequence == original.sequence);
    CHECK(decoded.control.roll.attitude.gain == 8.0f && decoded.l1_period_s == 4.0f);
    uint8_t roundtrip[AP_CONFIG_RECORD_SIZE];
    CHECK(ap_config_encode(&decoded, roundtrip, sizeof(roundtrip)));
    CHECK(memcmp(bytes, roundtrip, sizeof(roundtrip)) == 0);

    unsigned char previous[sizeof(decoded)];
    memcpy(previous, &decoded, sizeof(decoded));
    for (size_t length = 0; length < AP_CONFIG_RECORD_SIZE; ++length)
        CHECK(!ap_config_decode(bytes, length, &decoded));
    CHECK(!ap_config_decode(bytes, sizeof(bytes), &decoded));
    CHECK(!ap_config_decode(NULL, AP_CONFIG_RECORD_SIZE, &decoded));
    CHECK(!ap_config_decode(bytes, AP_CONFIG_RECORD_SIZE, NULL));
    CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);

    /* Every single-bit corruption, including the header and checksum. */
    for (unsigned i = 0; i < AP_CONFIG_RECORD_SIZE; ++i) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            uint8_t corrupted[AP_CONFIG_RECORD_SIZE];
            memcpy(corrupted, bytes, AP_CONFIG_RECORD_SIZE);
            corrupted[i] ^= (uint8_t)(1u << bit);
            CHECK(!ap_config_decode(corrupted, AP_CONFIG_RECORD_SIZE, &decoded));
            CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);
        }
    }
    /* Valid checksums do not excuse unsupported headers or invalid fields. */
    for (unsigned i = 0; i < 8; ++i) {
        uint8_t unsupported[AP_CONFIG_RECORD_SIZE];
        memcpy(unsupported, bytes, AP_CONFIG_RECORD_SIZE);
        unsupported[i] ^= 0x80;
        update_crc(unsupported);
        CHECK(!ap_config_decode(unsupported, AP_CONFIG_RECORD_SIZE, &decoded));
    }
    const uint32_t bad_values[] = {0x7FC00000u, 0x7F800000u, 0xFF800000u, 0xBF800000u};
    for (unsigned field = 0; field < 28; ++field) {
        for (unsigned bad = 0; bad < 4; ++bad) {
            if (field == 24 && bad == 3) continue; /* -1 is a valid actuator sign. */
            uint8_t invalid[AP_CONFIG_RECORD_SIZE];
            memcpy(invalid, bytes, AP_CONFIG_RECORD_SIZE);
            put_u32(invalid + 12 + 4 * field, bad_values[bad]);
            update_crc(invalid);
            CHECK(!ap_config_decode(invalid, AP_CONFIG_RECORD_SIZE, &decoded));
            CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);
        }
    }
    uint8_t untouched[AP_CONFIG_RECORD_SIZE];
    memset(untouched, 0xCC, sizeof(untouched));
    uint8_t snapshot[AP_CONFIG_RECORD_SIZE];
    memcpy(snapshot, untouched, sizeof(snapshot));
    CHECK(!ap_config_encode(NULL, untouched, AP_CONFIG_RECORD_SIZE));
    CHECK(!ap_config_encode(&original, NULL, AP_CONFIG_RECORD_SIZE));
    CHECK(!ap_config_encode(&original, untouched, 79));
    ap_aircraft_config_t invalid = original;
    invalid.control.airspeed.min_throttle_norm = invalid.control.airspeed.max_throttle_norm;
    CHECK(!ap_config_encode(&invalid, untouched, AP_CONFIG_RECORD_SIZE));
    invalid = original; invalid.control.attitude_limits.max_bank_rad = 1.570796327f;
    CHECK(!ap_config_encode(&invalid, untouched, AP_CONFIG_RECORD_SIZE));
    invalid = original; invalid.control.pitch.rate.max_elevator_norm = 1.01f;
    CHECK(!ap_config_encode(&invalid, untouched, AP_CONFIG_RECORD_SIZE));
    invalid = original; invalid.control.airspeed.ki = 0.0f;
    CHECK(!ap_config_encode(&invalid, untouched, AP_CONFIG_RECORD_SIZE));
    invalid = original; invalid.l1_period_s = 0.99f;
    CHECK(!ap_config_encode(&invalid, untouched, AP_CONFIG_RECORD_SIZE));
    invalid = original; invalid.l1_period_s = 30.01f;
    CHECK(!ap_config_encode(&invalid, untouched, AP_CONFIG_RECORD_SIZE));
    CHECK(memcmp(untouched, snapshot, sizeof(snapshot)) == 0);
    CHECK(!ap_config_validate(NULL));

    uint8_t v2[AP_CONFIG_V2_RECORD_SIZE] = {'A','P','C','F',2,0,80,0};
    memcpy(v2 + 12, bytes + 12, 80);
    put_u32(v2 + 92, flight_crc32(v2, 92));
    CHECK(ap_config_decode(v2, sizeof(v2), &decoded));
    CHECK(!decoded.control.yaw.enabled);
    CHECK(decoded.control.roll.rate.ki == original.control.roll.rate.ki);
    CHECK(decoded.control.pitch.rate.kp == original.control.pitch.rate.kp);

    /* Independent original-v1 record: convert gain units, supply rate limits,
     * leave integral action disabled, and never modify the input bytes. */
    uint8_t legacy[AP_CONFIG_V1_RECORD_SIZE] = {'A','P','C','F',1,0,64,0};
    const float legacy_fields[16] = {
        4.0f, 0.5f, 0.35f, 0.5f, 10.0f, 3.0f, 0.17f, 0.5f,
        0.08f, 0.005f, 0.0f, 1.0f, 0.015f, 0.05f, 0.052f, 4.0f
    };
    for (unsigned field = 0; field < 16; ++field) {
        uint32_t bits;
        memcpy(&bits, &legacy_fields[field], sizeof(bits));
        put_u32(legacy + 12 + 4 * field, bits);
    }
    put_u32(legacy + 76, flight_crc32(legacy, 76));
    uint8_t legacy_snapshot[sizeof(legacy)];
    memcpy(legacy_snapshot, legacy, sizeof(legacy));
    CHECK(ap_config_decode(legacy, sizeof(legacy), &decoded));
    CHECK(decoded.control.roll.attitude.gain == 8.0f);
    CHECK(!decoded.control.yaw.enabled);
    CHECK(decoded.control.pitch.attitude.gain == 10.0f / 3.0f);
    CHECK(decoded.control.roll.rate.kp == 0.5f && decoded.control.pitch.rate.kp == 3.0f);
    CHECK(decoded.control.roll.rate.ki == 0.0f && decoded.control.pitch.rate.ki == 0.0f);
    CHECK(decoded.control.roll.attitude.max_rate_rad_s == 1.0f);
    CHECK(decoded.control.pitch.attitude.max_rate_rad_s == 0.5f);
    CHECK(memcmp(legacy_snapshot, legacy, sizeof(legacy)) == 0);
    memcpy(previous, &decoded, sizeof(decoded));
    put_u32(legacy + 16, 0); /* Zero old roll damping cannot define a rate-feedback loop. */
    put_u32(legacy + 76, flight_crc32(legacy, 76));
    CHECK(!ap_config_decode(legacy, sizeof(legacy), &decoded));
    CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);
    return 0;
}

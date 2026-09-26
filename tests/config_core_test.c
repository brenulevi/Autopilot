#include "autopilot/config.h"

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
    for (unsigned i = 0; i < 76; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    put_u32(bytes + 76, ~crc);
}

int main(void)
{
    const ap_aircraft_config_t original = {
        {4.0f, 0.5f, 0.35f, 0.5f, 10.0f, 3.0f, 0.17f, 0.5f,
         0.08f, 0.005f, 0.0f, 1.0f, 0.015f, 0.05f, 0.052f},
        4.0f, 0x12345678u
    };
    uint8_t bytes[AP_CONFIG_RECORD_SIZE + 1];
    memset(bytes, 0xA5, sizeof(bytes));
    CHECK(ap_config_validate(&original));
    CHECK(ap_config_encode(&original, bytes, sizeof(bytes)));
    CHECK(bytes[80] == 0xA5);
    const uint8_t header[] = {'A','P','C','F',1,0,64,0,0x78,0x56,0x34,0x12};
    CHECK(memcmp(bytes, header, sizeof(header)) == 0);
    CHECK(bytes[12] == 0 && bytes[13] == 0 && bytes[14] == 0x80 && bytes[15] == 0x40);
    ap_aircraft_config_t decoded = {0};
    CHECK(ap_config_decode(bytes, 80, &decoded));
    CHECK(decoded.sequence == original.sequence);
    CHECK(decoded.control.roll_angle_gain == 4.0f && decoded.l1_period_s == 4.0f);
    uint8_t roundtrip[80];
    CHECK(ap_config_encode(&decoded, roundtrip, sizeof(roundtrip)));
    CHECK(memcmp(bytes, roundtrip, sizeof(roundtrip)) == 0);

    unsigned char previous[sizeof(decoded)];
    memcpy(previous, &decoded, sizeof(decoded));
    for (size_t length = 0; length < 80; ++length)
        CHECK(!ap_config_decode(bytes, length, &decoded));
    CHECK(!ap_config_decode(bytes, sizeof(bytes), &decoded));
    CHECK(!ap_config_decode(NULL, 80, &decoded));
    CHECK(!ap_config_decode(bytes, 80, NULL));
    CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);

    /* Every single-bit corruption, including the header and checksum. */
    for (unsigned i = 0; i < 80; ++i) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            uint8_t corrupted[80];
            memcpy(corrupted, bytes, 80);
            corrupted[i] ^= (uint8_t)(1u << bit);
            CHECK(!ap_config_decode(corrupted, 80, &decoded));
            CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);
        }
    }
    /* Valid checksums do not excuse unsupported headers or invalid fields. */
    for (unsigned i = 0; i < 8; ++i) {
        uint8_t unsupported[80];
        memcpy(unsupported, bytes, 80);
        unsupported[i] ^= 0x80;
        update_crc(unsupported);
        CHECK(!ap_config_decode(unsupported, 80, &decoded));
    }
    const uint32_t bad_values[] = {0x7FC00000u, 0x7F800000u, 0xFF800000u, 0xBF800000u};
    for (unsigned field = 0; field < 16; ++field) {
        for (unsigned bad = 0; bad < 4; ++bad) {
            uint8_t invalid[80];
            memcpy(invalid, bytes, 80);
            put_u32(invalid + 12 + 4 * field, bad_values[bad]);
            update_crc(invalid);
            CHECK(!ap_config_decode(invalid, 80, &decoded));
            CHECK(memcmp(previous, &decoded, sizeof(decoded)) == 0);
        }
    }
    uint8_t untouched[80];
    memset(untouched, 0xCC, sizeof(untouched));
    uint8_t snapshot[80];
    memcpy(snapshot, untouched, sizeof(snapshot));
    CHECK(!ap_config_encode(NULL, untouched, 80));
    CHECK(!ap_config_encode(&original, NULL, 80));
    CHECK(!ap_config_encode(&original, untouched, 79));
    ap_aircraft_config_t invalid = original;
    invalid.control.min_throttle = invalid.control.max_throttle;
    CHECK(!ap_config_encode(&invalid, untouched, 80));
    invalid = original; invalid.control.max_bank_rad = 1.570796327f;
    CHECK(!ap_config_encode(&invalid, untouched, 80));
    invalid = original; invalid.control.max_elevator = 1.01f;
    CHECK(!ap_config_encode(&invalid, untouched, 80));
    invalid = original; invalid.control.airspeed_ki = 0.0f;
    CHECK(!ap_config_encode(&invalid, untouched, 80));
    invalid = original; invalid.l1_period_s = 0.99f;
    CHECK(!ap_config_encode(&invalid, untouched, 80));
    invalid = original; invalid.l1_period_s = 30.01f;
    CHECK(!ap_config_encode(&invalid, untouched, 80));
    CHECK(memcmp(untouched, snapshot, sizeof(snapshot)) == 0);
    CHECK(!ap_config_validate(NULL));
    return 0;
}

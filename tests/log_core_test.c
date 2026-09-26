#include "flight_common/log.h"
#include "autopilot/autopilot.h"
#include "flight_io/supervisor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

typedef struct { uint8_t bytes[4096]; size_t used; bool reject; } memory_sink_t;
static bool memory_sink(void *context, const uint8_t *bytes, size_t length)
{
    memory_sink_t *memory = context;
    if (memory->reject || length > sizeof(memory->bytes) - memory->used) return false;
    memcpy(memory->bytes + memory->used, bytes, length);
    memory->used += length;
    return true;
}
static uint32_t checksum(const uint8_t *bytes, size_t n)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < n; ++i) {
        crc ^= bytes[i];
        for (unsigned j = 0; j < 8; ++j)
            crc = (crc >> 1) ^ ((crc & 1u) ? UINT32_C(0xedb88320) : 0u);
    }
    return ~crc;
}
static void repair_crc(uint8_t *bytes, size_t n)
{
    uint32_t crc = checksum(bytes, n - 4);
    for (unsigned i = 0; i < 4; ++i) bytes[n - 4 + i] = (uint8_t)(crc >> (8u * i));
}

int main(int argc, char **argv)
{
    /* Golden record produced independently with Python struct + zlib.crc32. */
    const uint8_t golden[] = {
        0x46,0x4c,0x47,0x31,0x01,0x01,0x01,0x00,0x0c,0x00,0x00,0x00,
        0x04,0x03,0x02,0x01,0x07,0x00,0x00,0x00,0x08,0x07,0x06,0x05,
        0x04,0x03,0x02,0x01,0x09,0x00,0x00,0x00,0x78,0x56,0x34,0x12,
        0x0b,0x00,0x00,0x00,0x0c,0x00,0x00,0x00,0x2c,0x56,0xf0,0xce
    };
    flight_log_record_t r = {0}, decoded = {0};
    r.source = FLIGHT_LOG_H723; r.session_id = 7; r.sequence = UINT32_C(0x01020304);
    r.timestamp_us = UINT64_C(0x0102030405060708); r.dropped_total = 9;
    r.entry.type = FLIGHT_LOG_BOOT;
    r.entry.data.boot.firmware_id = UINT32_C(0x12345678);
    r.entry.data.boot.config_id = 11; r.entry.data.boot.reset_reason = 12;
    uint8_t bytes[FLIGHT_LOG_MAX_RECORD]; size_t n = 0;
    CHECK(flight_log_encode(&r, bytes, sizeof(bytes), &n));
    CHECK(n == sizeof(golden) && memcmp(bytes, golden, n) == 0);
    CHECK(flight_log_decode(golden, sizeof(golden), &decoded));
    CHECK(decoded.timestamp_us == r.timestamp_us && decoded.entry.data.boot.firmware_id == UINT32_C(0x12345678));
    unsigned char saved[sizeof(decoded)]; memcpy(saved, &decoded, sizeof(saved));
    for (size_t i = 0; i < sizeof(golden); ++i) {
        CHECK(!flight_log_decode(golden, i, &decoded));
        memcpy(bytes, golden, sizeof(golden)); bytes[i] ^= 1;
        CHECK(!flight_log_decode(bytes, sizeof(golden), &decoded));
        CHECK(memcmp(saved, &decoded, sizeof(saved)) == 0);
    }
    memcpy(bytes, golden, sizeof(golden)); bytes[10] = 1; repair_crc(bytes, sizeof(golden));
    CHECK(!flight_log_decode(bytes, sizeof(golden), &decoded));
    memcpy(bytes, golden, sizeof(golden)); bytes[4] = 2; repair_crc(bytes, sizeof(golden));
    CHECK(!flight_log_decode(bytes, sizeof(golden), &decoded));
    memset(bytes, 0xa5, sizeof(bytes)); n = 123;
    CHECK(!flight_log_encode(&r, bytes, 1, &n) && bytes[0] == 0xa5 && n == 123);
    CHECK(!flight_log_encode(NULL, bytes, sizeof(bytes), &n));
    CHECK(!flight_log_decode(NULL, 0, &decoded));
    r.entry.type = 65535;
    CHECK(!flight_log_encode(&r, bytes, sizeof(bytes), &n));

    /* Exercise each payload, including intentionally invalid sensor values. */
    r.entry.type = FLIGHT_LOG_CONTROLS;
    r.entry.data.controls.values = (flight_controls_t){.25f, -.5f, NAN, .75f};
    r.entry.data.controls.stage = FLIGHT_LOG_COMPUTED; r.entry.data.controls.valid = false;
    CHECK(flight_log_encode(&r, bytes, sizeof(bytes), &n) && flight_log_decode(bytes, n, &decoded));
    CHECK(decoded.entry.data.controls.values.elevator == -.5f && isnan(decoded.entry.data.controls.values.rudder));
    bytes[49] = 2; repair_crc(bytes, n);
    CHECK(!flight_log_decode(bytes, n, &decoded));
    memset(&r.entry, 0, sizeof(r.entry)); r.entry.type = FLIGHT_LOG_STATE;
    r.entry.data.state.attitude_rad[0] = .125f; r.entry.data.state.body_rate_rad_s[2] = -2;
    r.entry.data.state.airspeed_m_s = 21; r.entry.data.state.altitude_m = 120;
    r.entry.data.state.climb_rate_m_s = -1; r.entry.data.state.valid_fields = 0x1ff;
    CHECK(flight_log_encode(&r, bytes, sizeof(bytes), &n) && flight_log_decode(bytes, n, &decoded));
    CHECK(decoded.entry.data.state.body_rate_rad_s[2] == -2 && decoded.entry.data.state.valid_fields == 0x1ff);
    r.entry.data.state.valid_fields = 0x200;
    CHECK(!flight_log_encode(&r, bytes, sizeof(bytes), &n));
    memset(&r.entry, 0, sizeof(r.entry)); r.entry.type = FLIGHT_LOG_IO;
    r.entry.data.io.status = (flight_io_status_t){FLIGHT_AUTHORITY_MANUAL, FLIGHT_REASON_PILOT_REQUEST};
    r.entry.data.io.armed = true; r.entry.data.io.pulse_count = 2;
    r.entry.data.io.pulse_us[0] = 1000; r.entry.data.io.pulse_us[1] = 1750;
    r.entry.data.io.rc_age_ms = 5; r.entry.data.io.autopilot_age_ms = UINT32_MAX;
    r.entry.data.io.saturated_mask = 2;
    CHECK(flight_log_encode(&r, bytes, sizeof(bytes), &n) && flight_log_decode(bytes, n, &decoded));
    CHECK(decoded.entry.data.io.pulse_us[1] == 1750 && decoded.entry.data.io.autopilot_age_ms == UINT32_MAX);
    bytes[52] = 1; repair_crc(bytes, n); /* Unused pulse slot must be zero. */
    CHECK(!flight_log_decode(bytes, n, &decoded));
    r.entry.data.io.pulse_count = 9;
    CHECK(!flight_log_encode(&r, bytes, sizeof(bytes), &n));
    memset(&r.entry, 0, sizeof(r.entry)); r.entry.type = FLIGHT_LOG_CLOCK_PAIR;
    r.entry.data.clock_pair.peer_source = FLIGHT_LOG_F405;
    r.entry.data.clock_pair.peer_session_id = 99;
    r.entry.data.clock_pair.peer_timestamp_us = UINT64_C(0x100000123);
    r.entry.data.clock_pair.uncertainty_us = 500;
    CHECK(flight_log_encode(&r, bytes, sizeof(bytes), &n) && flight_log_decode(bytes, n, &decoded));
    CHECK(decoded.entry.data.clock_pair.peer_timestamp_us == UINT64_C(0x100000123));

    memory_sink_t memory = {0}; flight_logger_t h723 = {0}, f405 = {0};
    CHECK(!flight_log_init(&h723, 0, 1, memory_sink, &memory));
    CHECK(!flight_log_init(&h723, FLIGHT_LOG_H723, 0, memory_sink, &memory));
    CHECK(!flight_log_init(&h723, FLIGHT_LOG_H723, 1, NULL, &memory));
    CHECK(flight_log_init(&h723, FLIGHT_LOG_H723, 7, memory_sink, &memory));
    CHECK(flight_log_init(&f405, FLIGHT_LOG_F405, 99, memory_sink, &memory));
    flight_log_entry_t event = {0}; event.type = FLIGHT_LOG_EVENT;
    event.data.event.code = 42; event.data.event.severity = FLIGHT_LOG_WARNING;
    memory.reject = true;
    CHECK(flight_log_emit(&h723, 10, &event) == FLIGHT_LOG_DROPPED);
    CHECK(h723.next_sequence == 1 && h723.dropped_total == 1 && memory.used == 0);
    memory.reject = false;
    CHECK(flight_log_emit(&h723, 20, &event) == FLIGHT_LOG_ACCEPTED);
    CHECK(flight_log_decode(memory.bytes, memory.used, &decoded));
    CHECK(decoded.sequence == 1 && decoded.dropped_total == 1 && decoded.entry.data.event.code == 42);
    event.data.event.severity = 255;
    CHECK(flight_log_emit(&h723, 30, &event) == FLIGHT_LOG_INVALID && h723.next_sequence == 2);
    event.data.event.severity = FLIGHT_LOG_WARNING;

    /* Real API calls from both libraries feed the same sink/codec. Firmware owns
     * these calls; neither algorithm performs hidden writes. */
    ap_input_t input = {0}; ap_output_t ap_output = {0};
    input.mode = AP_MODE_MANUAL; input.dt_s = .01f; input.state.airspeed_m_s = 20;
    input.requested = (ap_controls_t){.25f, -.5f, 0, .75f};
    CHECK(ap_step(NULL, &input, &ap_output));
    flight_log_entry_t controls = {0}; controls.type = FLIGHT_LOG_CONTROLS;
    controls.data.controls.values = ap_output.controls;
    controls.data.controls.stage = FLIGHT_LOG_COMPUTED; controls.data.controls.valid = true;
    CHECK(flight_log_emit(&h723, 10000, &controls) == FLIGHT_LOG_ACCEPTED);
    fio_supervisor_config_t config = {100, 50, {0,0,0,0}, {0,0,0,0}};
    fio_supervisor_runtime_t runtime = {0}; fio_supervisor_output_t selected = {0};
    flight_rc_sample_t rc = {{-.25f,0,0,.5f}, 10, true, false};
    CHECK(fio_supervisor_step(&config, 10, true, &rc, NULL, &runtime, &selected));
    controls.data.controls.values = selected.controls; controls.data.controls.stage = FLIGHT_LOG_SELECTED;
    CHECK(flight_log_emit(&f405, 10000, &controls) == FLIGHT_LOG_ACCEPTED);
    CHECK(flight_log_emit(&h723, r.timestamp_us, &r.entry) == FLIGHT_LOG_ACCEPTED);
    /* Optional fixture for the independent PC decoder integration test. */
    if (argc == 2) {
        FILE *file = fopen(argv[1], "wb"); CHECK(file != NULL);
        CHECK(fwrite(memory.bytes, 1, memory.used, file) == memory.used);
        CHECK(fclose(file) == 0);
    }
    h723.next_sequence = UINT32_MAX; h723.dropped_total = UINT32_MAX;
    memory.reject = true;
    CHECK(flight_log_emit(&h723, 40, &event) == FLIGHT_LOG_DROPPED);
    CHECK(h723.next_sequence == 0 && h723.dropped_total == UINT32_MAX);
    return 0;
}

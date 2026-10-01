#include "flight_common/crc32.h"
#include "flight_common/log.h"
#include <float.h>
#include <limits.h>
#include <string.h>

_Static_assert(CHAR_BIT == 8 && sizeof(float) == 4 && FLT_RADIX == 2 &&
               FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128, "FLG1 needs binary32 floats");

static bool source_valid(uint8_t source)
{
    return source >= FLIGHT_LOG_H723 && source <= FLIGHT_LOG_SIM;
}
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static void put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8u * i));
}
static uint32_t get32(const uint8_t *p)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < 4; ++i) v |= (uint32_t)p[i] << (8u * i);
    return v;
}
static void put64(uint8_t *p, uint64_t v)
{
    put32(p, (uint32_t)v); put32(p + 4, (uint32_t)(v >> 32));
}
static uint64_t get64(const uint8_t *p)
{
    return (uint64_t)get32(p) | ((uint64_t)get32(p + 4) << 32);
}
static void put_float(uint8_t *p, float v)
{
    uint32_t bits; memcpy(&bits, &v, 4); put32(p, bits);
}
static float get_float(const uint8_t *p)
{
    uint32_t bits = get32(p); float v; memcpy(&v, &bits, 4); return v;
}
static size_t payload_size(uint16_t type)
{
    switch (type) {
    case FLIGHT_LOG_BOOT: return 12;
    case FLIGHT_LOG_CONTROLS: return 20;
    case FLIGHT_LOG_STATE: return 40;
    case FLIGHT_LOG_IO: return 32;
    case FLIGHT_LOG_EVENT: return 12;
    case FLIGHT_LOG_CLOCK_PAIR: return 20;
    default: return 0;
    }
}
static bool entry_valid(const flight_log_entry_t *e)
{
    switch (e->type) {
    case FLIGHT_LOG_BOOT: return true;
    case FLIGHT_LOG_CONTROLS:
        return e->data.controls.stage >= FLIGHT_LOG_COMPUTED &&
               e->data.controls.stage <= FLIGHT_LOG_SELECTED;
    case FLIGHT_LOG_STATE: return (e->data.state.valid_fields & ~UINT32_C(0x1ff)) == 0;
    case FLIGHT_LOG_IO:
        return e->data.io.status.authority >= FLIGHT_AUTHORITY_DISARMED &&
               e->data.io.status.authority <= FLIGHT_AUTHORITY_FAILSAFE &&
               e->data.io.status.reason >= FLIGHT_REASON_DISARMED &&
               e->data.io.status.reason <= FLIGHT_REASON_REENGAGEMENT_REQUIRED &&
               e->data.io.pulse_count <= FLIGHT_LOG_OUTPUTS &&
               (e->data.io.saturated_mask >> e->data.io.pulse_count) == 0;
    case FLIGHT_LOG_EVENT: return e->data.event.severity <= FLIGHT_LOG_ERROR;
    case FLIGHT_LOG_CLOCK_PAIR: return source_valid(e->data.clock_pair.peer_source) &&
                                       e->data.clock_pair.peer_session_id != 0;
    default: return false;
    }
}

bool flight_log_encode(const flight_log_record_t *r, uint8_t *bytes,
                       size_t capacity, size_t *written)
{
    if (!r || !bytes || !written || !source_valid(r->source) || !r->session_id ||
        !entry_valid(&r->entry)) return false;
    size_t n = payload_size(r->entry.type);
    size_t total = FLIGHT_LOG_HEADER_SIZE + n + 4;
    if (capacity < total) return false;
    uint8_t buffer[FLIGHT_LOG_MAX_RECORD] = {0};
    memcpy(buffer, "FLG1", 4);
    buffer[4] = 1; buffer[5] = r->source;
    put16(buffer + 6, r->entry.type); put16(buffer + 8, (uint16_t)n);
    put32(buffer + 12, r->sequence); put32(buffer + 16, r->session_id);
    put64(buffer + 20, r->timestamp_us); put32(buffer + 28, r->dropped_total);
    uint8_t *p = buffer + FLIGHT_LOG_HEADER_SIZE;
    const flight_log_entry_t *e = &r->entry;
    switch (e->type) {
    case FLIGHT_LOG_BOOT:
        put32(p, e->data.boot.firmware_id); put32(p + 4, e->data.boot.config_id);
        put32(p + 8, e->data.boot.reset_reason); break;
    case FLIGHT_LOG_CONTROLS:
        put_float(p, e->data.controls.values.aileron);
        put_float(p + 4, e->data.controls.values.elevator);
        put_float(p + 8, e->data.controls.values.rudder);
        put_float(p + 12, e->data.controls.values.throttle);
        p[16] = e->data.controls.stage; p[17] = e->data.controls.valid ? 1 : 0; break;
    case FLIGHT_LOG_STATE:
        for (unsigned i = 0; i < 3; ++i) {
            put_float(p + 4u * i, e->data.state.attitude_rad[i]);
            put_float(p + 12u + 4u * i, e->data.state.body_rate_rad_s[i]);
        }
        put_float(p + 24, e->data.state.airspeed_m_s);
        put_float(p + 28, e->data.state.altitude_m);
        put_float(p + 32, e->data.state.climb_rate_m_s);
        put32(p + 36, e->data.state.valid_fields); break;
    case FLIGHT_LOG_IO:
        p[0] = (uint8_t)e->data.io.status.authority; p[1] = (uint8_t)e->data.io.status.reason;
        p[2] = e->data.io.pulse_count; p[3] = e->data.io.armed ? 1 : 0;
        put32(p + 4, e->data.io.rc_age_ms); put32(p + 8, e->data.io.autopilot_age_ms);
        put32(p + 12, e->data.io.saturated_mask);
        for (unsigned i = 0; i < e->data.io.pulse_count; ++i)
            put16(p + 16u + 2u * i, e->data.io.pulse_us[i]);
        break;
    case FLIGHT_LOG_EVENT:
        put16(p, e->data.event.code); p[2] = e->data.event.severity;
        put32(p + 4, e->data.event.argument0); put32(p + 8, e->data.event.argument1); break;
    case FLIGHT_LOG_CLOCK_PAIR:
        p[0] = e->data.clock_pair.peer_source; put32(p + 4, e->data.clock_pair.peer_session_id);
        put64(p + 8, e->data.clock_pair.peer_timestamp_us);
        put32(p + 16, e->data.clock_pair.uncertainty_us); break;
    default: return false;
    }
    put32(buffer + total - 4, flight_crc32(buffer, total - 4));
    memcpy(bytes, buffer, total); *written = total;
    return true;
}

bool flight_log_decode(const uint8_t *bytes, size_t length, flight_log_record_t *record)
{
    if (!bytes || !record || length < FLIGHT_LOG_HEADER_SIZE + 4 ||
        length > FLIGHT_LOG_MAX_RECORD || memcmp(bytes, "FLG1", 4) != 0 ||
        bytes[4] != 1 || bytes[10] || bytes[11]) return false;
    uint16_t type = get16(bytes + 6);
    size_t n = payload_size(type);
    if (!n || get16(bytes + 8) != n || length != FLIGHT_LOG_HEADER_SIZE + n + 4 ||
        get32(bytes + length - 4) != flight_crc32(bytes, length - 4) ||
        !source_valid(bytes[5]) || !get32(bytes + 16)) return false;
    flight_log_record_t r = {0};
    r.source = bytes[5]; r.sequence = get32(bytes + 12); r.session_id = get32(bytes + 16);
    r.timestamp_us = get64(bytes + 20); r.dropped_total = get32(bytes + 28);
    flight_log_entry_t *e = &r.entry;
    const uint8_t *p = bytes + FLIGHT_LOG_HEADER_SIZE;
    e->type = type;
    switch (type) {
    case FLIGHT_LOG_BOOT:
        e->data.boot.firmware_id = get32(p); e->data.boot.config_id = get32(p + 4);
        e->data.boot.reset_reason = get32(p + 8); break;
    case FLIGHT_LOG_CONTROLS:
        if (p[17] > 1 || p[18] || p[19]) return false;
        e->data.controls.values.aileron = get_float(p);
        e->data.controls.values.elevator = get_float(p + 4);
        e->data.controls.values.rudder = get_float(p + 8);
        e->data.controls.values.throttle = get_float(p + 12);
        e->data.controls.stage = p[16]; e->data.controls.valid = p[17] != 0; break;
    case FLIGHT_LOG_STATE:
        for (unsigned i = 0; i < 3; ++i) {
            e->data.state.attitude_rad[i] = get_float(p + 4u * i);
            e->data.state.body_rate_rad_s[i] = get_float(p + 12u + 4u * i);
        }
        e->data.state.airspeed_m_s = get_float(p + 24);
        e->data.state.altitude_m = get_float(p + 28);
        e->data.state.climb_rate_m_s = get_float(p + 32);
        e->data.state.valid_fields = get32(p + 36); break;
    case FLIGHT_LOG_IO:
        if (p[3] > 1 || p[2] > FLIGHT_LOG_OUTPUTS) return false;
        e->data.io.status.authority = (flight_authority_t)p[0];
        e->data.io.status.reason = (flight_selection_reason_t)p[1];
        e->data.io.pulse_count = p[2]; e->data.io.armed = p[3] != 0;
        e->data.io.rc_age_ms = get32(p + 4); e->data.io.autopilot_age_ms = get32(p + 8);
        e->data.io.saturated_mask = get32(p + 12);
        for (unsigned i = 0; i < FLIGHT_LOG_OUTPUTS; ++i) {
            e->data.io.pulse_us[i] = get16(p + 16u + 2u * i);
            if (i >= p[2] && e->data.io.pulse_us[i]) return false;
        }
        break;
    case FLIGHT_LOG_EVENT:
        if (p[3]) return false;
        e->data.event.code = get16(p); e->data.event.severity = p[2];
        e->data.event.argument0 = get32(p + 4); e->data.event.argument1 = get32(p + 8); break;
    case FLIGHT_LOG_CLOCK_PAIR:
        if (p[1] || p[2] || p[3]) return false;
        e->data.clock_pair.peer_source = p[0]; e->data.clock_pair.peer_session_id = get32(p + 4);
        e->data.clock_pair.peer_timestamp_us = get64(p + 8);
        e->data.clock_pair.uncertainty_us = get32(p + 16); break;
    default: return false;
    }
    if (!entry_valid(e)) return false;
    *record = r; return true;
}

bool flight_log_init(flight_logger_t *logger, uint8_t source, uint32_t session_id,
                     flight_log_sink_t sink, void *context)
{
    if (!logger || !source_valid(source) || !session_id || !sink) return false;
    flight_logger_t initialized = {sink, context, source, session_id, 0, 0};
    *logger = initialized;
    return true;
}

flight_log_result_t flight_log_emit(flight_logger_t *logger, uint64_t timestamp_us,
                                    const flight_log_entry_t *entry)
{
    if (!logger || !entry || !logger->sink) return FLIGHT_LOG_INVALID;
    flight_log_record_t r = {0};
    r.source = logger->source; r.session_id = logger->session_id;
    r.sequence = logger->next_sequence; r.timestamp_us = timestamp_us;
    r.dropped_total = logger->dropped_total; r.entry = *entry;
    uint8_t bytes[FLIGHT_LOG_MAX_RECORD]; size_t n = 0;
    if (!flight_log_encode(&r, bytes, sizeof(bytes), &n)) return FLIGHT_LOG_INVALID;
    ++logger->next_sequence;
    if (logger->sink(logger->context, bytes, n)) return FLIGHT_LOG_ACCEPTED;
    if (logger->dropped_total != UINT32_MAX) ++logger->dropped_total;
    return FLIGHT_LOG_DROPPED;
}

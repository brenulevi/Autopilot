#ifndef FLIGHT_COMMON_LOG_H
#define FLIGHT_COMMON_LOG_H

#include "flight_common/control.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FLG1 v1: explicitly encoded little-endian integers/IEEE-754 binary32.
 * These structs are in-memory inputs, never flash or UART memory images. */
#define FLIGHT_LOG_HEADER_SIZE 32u
#define FLIGHT_LOG_MAX_PAYLOAD 64u
#define FLIGHT_LOG_MAX_RECORD (FLIGHT_LOG_HEADER_SIZE + FLIGHT_LOG_MAX_PAYLOAD + 4u)
#define FLIGHT_LOG_OUTPUTS 8u

enum { FLIGHT_LOG_H723 = 1, FLIGHT_LOG_F405 = 2, FLIGHT_LOG_SIM = 3 };
enum {
    FLIGHT_LOG_BOOT = 1, FLIGHT_LOG_CONTROLS = 2, FLIGHT_LOG_STATE = 3,
    FLIGHT_LOG_IO = 4, FLIGHT_LOG_EVENT = 5, FLIGHT_LOG_CLOCK_PAIR = 6
};
enum { FLIGHT_LOG_COMPUTED = 1, FLIGHT_LOG_PILOT = 2, FLIGHT_LOG_SELECTED = 3 };
enum { FLIGHT_LOG_INFO = 0, FLIGHT_LOG_WARNING = 1, FLIGHT_LOG_ERROR = 2 };

typedef struct {
    uint16_t type;
    union {
        struct { uint32_t firmware_id, config_id, reset_reason; } boot;
        struct {
            flight_controls_t values;
            uint8_t stage; /* COMPUTED, PILOT, or SELECTED. */
            bool valid;   /* False permits recording a rejected controller/RC sample. */
        } controls;
        struct {
            float attitude_rad[3]; /* roll, pitch, yaw relative to NED */
            float body_rate_rad_s[3]; /* p,q,r; forward/right/down axes */
            float airspeed_m_s, altitude_m, climb_rate_m_s; /* TAS, MSL, positive up */
            uint32_t valid_fields; /* Bits 0..8 match the nine floats in wire order. */
        } state;
        struct {
            flight_io_status_t status;
            bool armed; /* Firmware permission; not proof of electrical output. */
            uint8_t pulse_count;
            uint32_t rc_age_ms, autopilot_age_ms; /* UINT32_MAX means unavailable. */
            uint32_t saturated_mask;
            uint16_t pulse_us[FLIGHT_LOG_OUTPUTS]; /* Demands, not measured positions. */
        } io;
        struct {
            uint16_t code; /* Application-owned event dictionary, tied to firmware_id. */
            uint8_t severity;
            uint32_t argument0, argument1;
        } event;
        struct {
            uint8_t peer_source;
            uint32_t peer_session_id;
            uint64_t peer_timestamp_us;
            uint32_t uncertainty_us; /* UINT32_MAX if unknown. */
        } clock_pair; /* Header time and peer time describe the same estimated instant. */
    } data;
} flight_log_entry_t;

typedef struct {
    uint8_t source;
    uint32_t session_id; /* Firmware assigns a new nonzero ID on every boot. */
    uint32_t sequence;
    uint64_t timestamp_us; /* Source-local monotonic time; firmware extends timer wraps. */
    uint32_t dropped_total; /* Cumulative queue rejections, saturates at UINT32_MAX. */
    flight_log_entry_t entry;
} flight_log_record_t;

/* Exact-length codec. On failure, outputs (including written) are unchanged.
 * No allocation. Float NaNs/infinities are preserved for fault diagnosis.
 * Unsupported versions/types and malformed reserved fields are rejected. */
bool flight_log_encode(const flight_log_record_t *record, uint8_t *bytes,
                       size_t capacity, size_t *written);
bool flight_log_decode(const uint8_t *bytes, size_t length, flight_log_record_t *record);

/* Callback must COPY/enqueue the entire record before returning true. The byte
 * pointer expires on return. False means no bytes accepted; never partial success.
 * Enqueueing is not durability. Flash erase/program/flush belongs to a background
 * firmware worker, with its own persistent-error reporting and recovery policy. */
typedef bool (*flight_log_sink_t)(void *context, const uint8_t *bytes, size_t length);
typedef struct {
    flight_log_sink_t sink;
    void *context;
    uint8_t source;
    uint32_t session_id, next_sequence, dropped_total;
} flight_logger_t;
typedef enum {
    FLIGHT_LOG_ACCEPTED = 0, FLIGHT_LOG_DROPPED, FLIGHT_LOG_INVALID
} flight_log_result_t;

/* Caller-owned, single-producer, not ISR-safe/reentrant by itself. Initialize
 * once per boot, then serialize calls. Invalid calls consume no sequence number.
 * Valid attempts consume a sequence even on queue rejection (modulo 2^32).
 * No clocks, driver calls, retries, blocking, or automatic control-loop hooks. */
bool flight_log_init(flight_logger_t *logger, uint8_t source, uint32_t session_id,
                     flight_log_sink_t sink, void *context);
flight_log_result_t flight_log_emit(flight_logger_t *logger, uint64_t timestamp_us,
                                    const flight_log_entry_t *entry);

#ifdef __cplusplus
}
#endif
#endif

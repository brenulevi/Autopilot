#ifndef FLIGHT_COMMON_CONTROL_H
#define FLIGHT_COMMON_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* Shared in-memory contract, NOT a UART packet or EEPROM memory image.
 * Logical surfaces [-1,1], throttle [0,1]. Positive aileron requests right roll;
 * negative elevator requests nose-up, matching the existing autopilot law.
 * Firmware must establish physical directions, mixing, neutral and travel.
 * No simulator, autopilot, MCU, or allocation dependencies. */
typedef struct {
    float aileron;
    float elevator;
    float rudder;
    float throttle;
} flight_controls_t;

/* Timestamp is assigned by the receiving I/O side's monotonic millisecond
 * clock AFTER accepting a complete new message, never copied from the H723
 * clock or refreshed merely because the control loop ran. */
typedef struct {
    flight_controls_t controls;
    uint32_t received_at_ms;
    bool valid;
} flight_control_sample_t;

typedef struct {
    flight_controls_t controls;
    uint32_t received_at_ms;
    bool valid; /* Includes receiver-loss policy, not just a valid checksum. */
    bool request_auto;
} flight_rc_sample_t;

typedef enum {
    FLIGHT_AUTHORITY_DISARMED = 0,
    FLIGHT_AUTHORITY_MANUAL,
    FLIGHT_AUTHORITY_AUTOPILOT,
    FLIGHT_AUTHORITY_FAILSAFE
} flight_authority_t;

typedef enum {
    FLIGHT_REASON_DISARMED = 0,
    FLIGHT_REASON_PILOT_REQUEST,
    FLIGHT_REASON_AUTO_ACTIVE,
    FLIGHT_REASON_RC_UNAVAILABLE,
    FLIGHT_REASON_AUTOPILOT_UNAVAILABLE,
    FLIGHT_REASON_REENGAGEMENT_REQUIRED
} flight_selection_reason_t;

typedef struct {
    flight_authority_t authority;
    flight_selection_reason_t reason;
} flight_io_status_t;

#endif

#ifndef FLIGHT_IO_SUPERVISOR_H
#define FLIGHT_IO_SUPERVISOR_H

#include "flight_common/control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rc_timeout_ms; /* 1..INT32_MAX, using I/O-side local timestamps. */
    uint32_t autopilot_timeout_ms;
    flight_controls_t disarmed_controls;
    flight_controls_t failsafe_controls; /* Application-defined; no universal flight-safe default. */
} fio_supervisor_config_t;

typedef struct {
    bool auto_eligible; /* Initialize to {0}; only a healthy armed MANUAL step sets this. */
} fio_supervisor_runtime_t;

typedef struct {
    flight_controls_t controls;
    flight_io_status_t status;
} fio_supervisor_output_t;

bool fio_supervisor_config_valid(const fio_supervisor_config_t *config);

/* No estimator or AP_MODE_MANUAL call on the manual path.
 * NULL samples mean unavailable. Stale/invalid RC -> configured fixed failsafe
 * outputs; missing AP while RC is healthy -> manual fallback, latching AUTO out.
 * After boot, disarm, or a fault, observe MANUAL then AUTO to engage again.
 * 'armed' is permission from firmware; this function does not implement arming.
 * Ages use uint32 wrap arithmetic; clock must be monotonic modulo 2^32 and
 * samples must not be retained for a full clock wrap. Future timestamps fail.
 * True returns a decision even for faults. False is an API/config error and
 * leaves output/runtime unchanged; firmware must not keep stale AUTO outputs.
 * Caller owns controller reset/target initialization when re-engaging AUTO. */
bool fio_supervisor_step(const fio_supervisor_config_t *config, uint32_t now_ms,
                         bool armed, const flight_rc_sample_t *rc,
                         const flight_control_sample_t *autopilot,
                         fio_supervisor_runtime_t *runtime,
                         fio_supervisor_output_t *output);

#ifdef __cplusplus
}
#endif
#endif

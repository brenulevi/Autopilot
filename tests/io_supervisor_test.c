#include "flight_io/supervisor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

int main(void)
{
    /* Synthetic test outputs/timeouts, not physical-aircraft recommendations. */
    fio_supervisor_config_t config = {100, 50, {0,0,0,0}, {0,-.1f,0,.2f}};
    fio_supervisor_runtime_t runtime = {0};
    fio_supervisor_output_t output = {0};
    flight_rc_sample_t rc = {{.2f,-.3f,.1f,.6f}, 100, true, true};
    flight_control_sample_t ap = {{-.4f,.2f,0,.7f}, 100, true};
    CHECK(fio_supervisor_step(&config, 100, false, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_DISARMED && output.controls.throttle == 0);
    CHECK(fio_supervisor_step(&config, 100, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.reason == FLIGHT_REASON_REENGAGEMENT_REQUIRED);
    rc.request_auto = false;
    CHECK(fio_supervisor_step(&config, 100, true, &rc, NULL, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_MANUAL && output.controls.aileron == .2f);
    rc.request_auto = true;
    CHECK(fio_supervisor_step(&config, 150, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_AUTOPILOT && output.controls.aileron == -.4f);
    CHECK(fio_supervisor_step(&config, 151, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.reason == FLIGHT_REASON_AUTOPILOT_UNAVAILABLE && !runtime.auto_eligible);
    ap.received_at_ms = 151;
    CHECK(fio_supervisor_step(&config, 151, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.reason == FLIGHT_REASON_REENGAGEMENT_REQUIRED);
    rc.request_auto = false;
    CHECK(fio_supervisor_step(&config, 151, true, &rc, &ap, &runtime, &output));
    rc.request_auto = true;
    CHECK(fio_supervisor_step(&config, 151, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_AUTOPILOT);
    /* AP can produce invalid values while the manual path still works. */
    ap.controls.aileron = NAN;
    CHECK(fio_supervisor_step(&config, 151, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_MANUAL);
    rc.request_auto = false;
    CHECK(fio_supervisor_step(&config, 151, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.reason == FLIGHT_REASON_PILOT_REQUEST);
    CHECK(fio_supervisor_step(&config, 201, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_FAILSAFE && output.controls.throttle == .2f);
    CHECK(!runtime.auto_eligible);
    rc.received_at_ms = 201; rc.valid = false;
    CHECK(fio_supervisor_step(&config, 201, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.reason == FLIGHT_REASON_RC_UNAVAILABLE);
    rc.valid = true; rc.controls.aileron = 1.1f;
    CHECK(fio_supervisor_step(&config, 201, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_FAILSAFE);
    rc.controls.aileron = .2f; rc.received_at_ms = 202;
    CHECK(fio_supervisor_step(&config, 201, true, &rc, &ap, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_FAILSAFE); /* Future time rejected. */
    rc.received_at_ms = UINT32_MAX - 10;
    CHECK(fio_supervisor_step(&config, 5, true, &rc, NULL, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_MANUAL); /* Age=16 across wrap. */
    CHECK(fio_supervisor_step(&config, 5, false, NULL, NULL, &runtime, &output));
    CHECK(output.status.authority == FLIGHT_AUTHORITY_DISARMED && !runtime.auto_eligible);
    unsigned char previous[sizeof(output)]; memcpy(previous, &output, sizeof(output));
    config.rc_timeout_ms = 0;
    CHECK(!fio_supervisor_step(&config, 5, true, &rc, &ap, &runtime, &output));
    CHECK(memcmp(previous, &output, sizeof(output)) == 0 && !runtime.auto_eligible);
    config.rc_timeout_ms = UINT32_MAX;
    CHECK(!fio_supervisor_config_valid(&config));
    config.rc_timeout_ms = 100; config.disarmed_controls.throttle = .1f;
    CHECK(!fio_supervisor_config_valid(&config));
    CHECK(!fio_supervisor_config_valid(NULL));
    return 0;
}

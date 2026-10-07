#include "autopilot/control/airspeed.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; \
} } while (0)

int main(void)
{
    const control_airspeed_config_t airspeed = {0.1f, 0.02f};
    const control_throttle_config_t throttle = {0.5f, 0.0f, 1.0f};
    control_airspeed_state_t state = {0.0f};
    float command = 99.0f;

    CHECK(control_airspeed_step(50.0f, 50.0f, 1.0f, &airspeed, &throttle,
                                &state, &command));
    CHECK(fabsf(command - 0.5f) < 1e-6f);

    CHECK(control_airspeed_step(51.0f, 50.0f, 1.0f, &airspeed, &throttle,
                                &state, &command));
    CHECK(fabsf(command - 0.62f) < 1e-6f);
    CHECK(fabsf(state.integral_command - 0.02f) < 1e-6f);

    /* Saturation must not accumulate more integral in the same direction. */
    CHECK(control_airspeed_step(60.0f, 50.0f, 1.0f, &airspeed, &throttle,
                                &state, &command));
    CHECK(command == 1.0f);
    CHECK(fabsf(state.integral_command - 0.02f) < 1e-6f);
    CHECK(control_airspeed_step(40.0f, 50.0f, 1.0f, &airspeed, &throttle,
                                &state, &command));
    CHECK(command == 0.0f);
    CHECK(fabsf(state.integral_command - 0.02f) < 1e-6f);

    CHECK(control_airspeed_step(49.0f, 50.0f, 1.0f, &airspeed, &throttle,
                                &state, &command));
    CHECK(fabsf(command - 0.4f) < 1e-6f);
    CHECK(fabsf(state.integral_command) < 1e-6f);

    command = 0.123f;
    CHECK(!control_airspeed_step(51.0f, 50.0f, 0.0f, &airspeed, &throttle,
                                 &state, &command));
    CHECK(command == 0.123f && state.integral_command == 0.0f);
    CHECK(!control_airspeed_step(NAN, 50.0f, 1.0f, &airspeed, &throttle,
                                 &state, &command));
    CHECK(command == 0.123f && state.integral_command == 0.0f);
    return 0;
}

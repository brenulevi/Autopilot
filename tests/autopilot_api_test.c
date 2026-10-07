#include "autopilot/autopilot.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; \
} } while (0)

int main(void)
{
    const autopilot_config_t config = {
        {4.0f, 0.5f},
        {10.0f, 3.0f},
        {0.1f, 0.02f},
        {0.02f, -0.4f, 0.4f},
        {-0.05f, -0.5f, 0.5f},
        {0.5f, 0.0f, 1.0f}
    };
    const autopilot_input_t input = {
        0.05f, 0.02f, 0.0f, 0.0f, 0.2f, 0.01f,
        51.0f, 50.0f, 0.5f
    };
    autopilot_state_t state = {0};
    autopilot_output_t output = {0};

    CHECK(autopilot_step(&config, &input, &state, &output));
    CHECK(fabsf(output.aileron_command - 0.12f) < 1e-6f);
    CHECK(fabsf(output.elevator_command + 0.22f) < 1e-6f);
    CHECK(fabsf(output.throttle_command - 0.61f) < 1e-6f);
    CHECK(fabsf(state.airspeed.integral_command - 0.01f) < 1e-6f);

    output.aileron_command = 0.123f;
    output.elevator_command = -0.321f;
    output.throttle_command = 0.456f;
    autopilot_input_t invalid_input = input;
    invalid_input.measured_pitch_rate_rad_s = NAN;
    CHECK(!autopilot_step(&config, &invalid_input, &state, &output));
    CHECK(output.aileron_command == 0.123f);
    CHECK(output.elevator_command == -0.321f);
    CHECK(output.throttle_command == 0.456f);
    CHECK(fabsf(state.airspeed.integral_command - 0.01f) < 1e-6f);

    invalid_input = input;
    invalid_input.measured_airspeed_m_s = NAN;
    CHECK(!autopilot_step(&config, &invalid_input, &state, &output));
    CHECK(output.aileron_command == 0.123f);
    CHECK(output.elevator_command == -0.321f);
    CHECK(output.throttle_command == 0.456f);
    CHECK(fabsf(state.airspeed.integral_command - 0.01f) < 1e-6f);
    return 0;
}

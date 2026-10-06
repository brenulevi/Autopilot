#include "autopilot/autopilot.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

int main(void)
{
    ap_config_t config = {
        .roll = {{2.0f / 0.3f, 10.0f}, {0.3f, 0.0f, 0.5f}},
        .pitch = {{6.0f, 10.0f}, {0.5f, 0.0f, 0.5f}},
        .airspeed = {0.1f, 0.05f, 0.2f, 0.8f},
        .altitude = {0.015f, 0.05f, 0.06f},
        .attitude_limits = {0.35f, 0.2f}
    };
    ap_input_t input = {0};
    input.mode = AP_MODE_ALTITUDE_AIRSPEED_HOLD;
    input.dt_s = 0.01f;
    input.state.airspeed_m_s = 50.0f;
    input.state.altitude_m = 100.0f;
    input.requested.throttle = 0.5f;
    input.airspeed_command_m_s = 50.0f;
    input.altitude_command_m = 102.0f;
    input.pitch_command_rad = 0.01f;
    ap_runtime_t runtime = {0};
    ap_output_t output = {0};

    CHECK(!ap_step(&config, &input, NULL, &output));
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(fabsf(output.pitch_command_rad - 0.04f) < 1e-6f);
    CHECK(output.altitude_error_m == 2.0f);
    CHECK(!output.altitude_pitch_limited);
    CHECK(output.controls.elevator < 0.0f);

    input.state.climb_rate_m_s = 0.4f;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(fabsf(output.pitch_command_rad - 0.02f) < 1e-6f);

    input.state.climb_rate_m_s = 0.0f;
    input.altitude_command_m = 120.0f;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(fabsf(output.pitch_command_rad - 0.07f) < 1e-6f);
    CHECK(output.altitude_pitch_limited);

    input.altitude_command_m = 80.0f;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(fabsf(output.pitch_command_rad + 0.05f) < 1e-6f);
    CHECK(output.altitude_pitch_limited);

    const ap_output_t previous = output;
    const float previous_integral = runtime.airspeed.integral_norm;
    config.altitude.altitude_gain = NAN;
    CHECK(!ap_step(&config, &input, &runtime, &output));
    CHECK(output.pitch_command_rad == previous.pitch_command_rad);
    CHECK(runtime.airspeed.integral_norm == previous_integral);
    config.altitude.altitude_gain = 0.015f;
    input.state.climb_rate_m_s = NAN;
    CHECK(!ap_step(&config, &input, &runtime, &output));
    CHECK(output.pitch_command_rad == previous.pitch_command_rad);
    CHECK(runtime.airspeed.integral_norm == previous_integral);
    input.state.climb_rate_m_s = 0.0f;
    input.altitude_command_m = NAN;
    CHECK(!ap_step(&config, &input, &runtime, &output));
    CHECK(output.pitch_command_rad == previous.pitch_command_rad);
    CHECK(runtime.airspeed.integral_norm == previous_integral);
    return 0;
}

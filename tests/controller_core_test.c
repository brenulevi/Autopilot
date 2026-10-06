#include "autopilot/autopilot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

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
    ap_controller_t first, second;
    CHECK(ap_controller_init(&first, &config));
    CHECK(ap_controller_init(&second, &config));
    CHECK(first.runtime.airspeed.integral_norm == 0.0f);
    CHECK(!ap_controller_init(NULL, &config));
    CHECK(!ap_control_config_validate(NULL));

    /* The instance owns a copy of configuration and independent PI memory. */
    config.airspeed.ki = 0.2f;
    ap_input_t input = {0};
    input.mode = AP_MODE_ATTITUDE_AIRSPEED_HOLD;
    input.state.airspeed_m_s = 50.0f;
    input.airspeed_command_m_s = 51.0f;
    input.requested.throttle = 0.5f;
    input.dt_s = 1.0f;
    ap_output_t output = {0};
    CHECK(ap_controller_step(&first, &input, &output));
    CHECK(fabsf(output.controls.throttle - 0.65f) < 1e-6f);
    CHECK(fabsf(first.runtime.airspeed.integral_norm - 0.05f) < 1e-6f);
    CHECK(second.runtime.airspeed.integral_norm == 0.0f);
    CHECK(ap_controller_step(&first, &input, &output));
    CHECK(fabsf(output.controls.throttle - 0.70f) < 1e-6f);
    CHECK(ap_controller_step(&second, &input, &output));
    CHECK(fabsf(output.controls.throttle - 0.65f) < 1e-6f);

    /* A failed late-stage update commits neither earlier axis results nor PI state. */
    unsigned char saved_controller[sizeof(first)], saved_output[sizeof(output)];
    memcpy(saved_controller, &first, sizeof(first));
    memcpy(saved_output, &output, sizeof(output));
    input.bank_command_rad = 0.1f;
    input.airspeed_command_m_s = NAN;
    CHECK(!ap_controller_step(&first, &input, &output));
    CHECK(memcmp(saved_controller, &first, sizeof(first)) == 0);
    CHECK(memcmp(saved_output, &output, sizeof(output)) == 0);
    CHECK(!ap_controller_step(NULL, &input, &output));
    CHECK(!ap_controller_step(&first, NULL, &output));
    CHECK(!ap_controller_step(&first, &input, NULL));

    /* Invalid reinitialization preserves the current instance. */
    config.roll.attitude.gain = NAN;
    CHECK(!ap_controller_init(&first, &config));
    CHECK(!ap_controller_init(&first, NULL));
    CHECK(memcmp(saved_controller, &first, sizeof(first)) == 0);
    config.roll.attitude.gain = 2.0f / 0.3f;
    CHECK(ap_controller_init(&first, &config));
    CHECK(first.runtime.airspeed.integral_norm == 0.0f);
    input.airspeed_command_m_s = 51.0f;
    CHECK(ap_controller_step(&first, &input, &output));
    CHECK(first.runtime.airspeed.integral_norm > 0.0f);
    input.mode = AP_MODE_MANUAL;
    CHECK(ap_controller_step(&first, &input, &output));
    CHECK(first.runtime.airspeed.integral_norm == 0.0f);

    /* Each module can run without aircraft-wide input/config/output structs. */
    const ap_roll_attitude_input_t roll_input = {0.05f, 0.15f};
    ap_roll_attitude_output_t reference;
    CHECK(ap_roll_attitude_compute(&config.roll.attitude, &roll_input, &reference));
    ap_roll_rate_runtime_t roll_runtime = {0};
    const ap_roll_rate_input_t rate_input = {0.2f, reference.body_rate_command_rad_s, 0.02f, 0.01f};
    ap_roll_rate_output_t rate_output;
    CHECK(ap_roll_rate_compute(&config.roll.rate, &rate_input, &roll_runtime, &rate_output));
    CHECK(fabsf(rate_output.aileron_norm - 0.16f) < 1e-6f);
    CHECK(!rate_output.saturated);

    const ap_altitude_input_t altitude_input = {100.0f, 0.4f, 102.0f, 0.01f};
    ap_altitude_output_t altitude_output;
    CHECK(ap_guidance_altitude(&config.altitude, &altitude_input, &altitude_output));
    CHECK(fabsf(altitude_output.pitch_command_rad - 0.02f) < 1e-6f);
    CHECK(!altitude_output.limited);

    ap_airspeed_input_t speed_input = {50.0f, 51.0f, 0.5f, 1.0f};
    ap_airspeed_runtime_t speed_runtime = {0};
    ap_airspeed_output_t speed_output;
    config.airspeed.ki = 0.05f;
    CHECK(ap_airspeed_compute(&config.airspeed, &speed_input,
                              &speed_runtime, &speed_output));
    CHECK(fabsf(speed_output.throttle_norm - 0.65f) < 1e-6f);
    speed_input.dt_s = 0.0f;
    CHECK(!ap_airspeed_compute(&config.airspeed, &speed_input,
                               &speed_runtime, &speed_output));
    CHECK(fabsf(speed_runtime.integral_norm - 0.05f) < 1e-6f);
    CHECK(fabsf(speed_output.throttle_norm - 0.65f) < 1e-6f);
    return 0;
}

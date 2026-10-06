#include "autopilot/autopilot.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

int main(void)
{
    const ap_roll_attitude_config_t attitude_config = {2.0f, 0.3f};
    ap_roll_attitude_input_t attitude_input = {0.0f, 1.0f};
    ap_roll_attitude_output_t reference;
    CHECK(ap_roll_attitude_compute(&attitude_config, &attitude_input, &reference));
    CHECK(reference.limited && reference.body_rate_command_rad_s == 0.3f);
    attitude_input.target_bank_rad = -1.0f;
    CHECK(ap_roll_attitude_compute(&attitude_config, &attitude_input, &reference));
    CHECK(reference.limited && reference.body_rate_command_rad_s == -0.3f);
    attitude_input.target_bank_rad = 0.05f;
    CHECK(ap_roll_attitude_compute(&attitude_config, &attitude_input, &reference));
    CHECK(!reference.limited && reference.body_rate_command_rad_s == 0.1f);
    attitude_input.measured_bank_rad = NAN;
    CHECK(!ap_roll_attitude_compute(&attitude_config, &attitude_input, &reference));
    CHECK(reference.body_rate_command_rad_s == 0.1f);

    const ap_roll_rate_config_t roll_config = {0.5f, 0.25f, 0.5f};
    const ap_pitch_rate_config_t pitch_config = {0.5f, 0.25f, 0.5f};
    ap_roll_rate_runtime_t roll_runtime = {0};
    ap_pitch_rate_runtime_t pitch_runtime = {0};
    ap_roll_rate_input_t roll_input = {0.1f, 0.4f, 0.0f, 1.0f};
    ap_pitch_rate_input_t pitch_input = {0.1f, 0.4f, 0.0f, 1.0f};
    ap_roll_rate_output_t roll_output;
    ap_pitch_rate_output_t pitch_output;
    CHECK(ap_roll_rate_compute(&roll_config, &roll_input, &roll_runtime, &roll_output));
    CHECK(ap_pitch_rate_compute(&pitch_config, &pitch_input, &pitch_runtime, &pitch_output));
    CHECK(fabsf(roll_output.aileron_norm - 0.225f) < 1e-6f);
    CHECK(fabsf(pitch_output.elevator_norm + 0.225f) < 1e-6f);
    CHECK(fabsf(roll_runtime.integral_norm - 0.075f) < 1e-6f);
    CHECK(fabsf(pitch_runtime.integral_norm - 0.075f) < 1e-6f);

    /* Sustained saturation must not accumulate integral, with either servo sign. */
    const float retained = roll_runtime.integral_norm;
    for (int sign = -1; sign <= 1; sign += 2) {
        roll_input.target_body_rate_rad_s = 10.0f * sign;
        pitch_input.target_body_rate_rad_s = 10.0f * sign;
        for (int tick = 0; tick < 100; ++tick) {
            CHECK(ap_roll_rate_compute(&roll_config, &roll_input, &roll_runtime, &roll_output));
            CHECK(ap_pitch_rate_compute(&pitch_config, &pitch_input, &pitch_runtime, &pitch_output));
            CHECK(roll_output.saturated && pitch_output.saturated);
            CHECK(roll_runtime.integral_norm == retained && pitch_runtime.integral_norm == retained);
            CHECK(roll_output.aileron_norm == 0.5f * sign);
            CHECK(pitch_output.elevator_norm == -0.5f * sign);
        }
    }

    /* Integration back out of a limit is allowed, including reversed pitch sign. */
    roll_runtime.integral_norm = pitch_runtime.integral_norm = 0.3f;
    roll_input = (ap_roll_rate_input_t){0.0f, -0.2f, 0.4f, 1.0f};
    pitch_input = (ap_pitch_rate_input_t){0.0f, -0.2f, -0.4f, 1.0f};
    CHECK(ap_roll_rate_compute(&roll_config, &roll_input, &roll_runtime, &roll_output));
    CHECK(ap_pitch_rate_compute(&pitch_config, &pitch_input, &pitch_runtime, &pitch_output));
    CHECK(fabsf(roll_runtime.integral_norm - 0.25f) < 1e-6f);
    CHECK(fabsf(pitch_runtime.integral_norm - 0.25f) < 1e-6f);
    CHECK(roll_output.saturated && pitch_output.saturated);

    const float saved_integral = roll_runtime.integral_norm;
    const float saved_command = roll_output.aileron_norm;
    roll_input.dt_s = 0.0f;
    CHECK(!ap_roll_rate_compute(&roll_config, &roll_input, &roll_runtime, &roll_output));
    CHECK(roll_runtime.integral_norm == saved_integral && roll_output.aileron_norm == saved_command);
    roll_input.dt_s = 1.0f;
    roll_input.measured_body_rate_rad_s = NAN;
    CHECK(!ap_roll_rate_compute(&roll_config, &roll_input, &roll_runtime, &roll_output));
    CHECK(roll_runtime.integral_norm == saved_integral && roll_output.aileron_norm == saved_command);

    /* The desired-rate limiter changes the law before actuator saturation. */
    attitude_input = (ap_roll_attitude_input_t){0.0f, 1.0f};
    CHECK(ap_roll_attitude_compute(&attitude_config, &attitude_input, &reference));
    ap_roll_rate_config_t proportional = {0.5f, 0.0f, 0.5f};
    roll_runtime.integral_norm = 0.0f;
    roll_input = (ap_roll_rate_input_t){0.0f, reference.body_rate_command_rad_s, 0.0f, 0.01f};
    CHECK(ap_roll_rate_compute(&proportional, &roll_input, &roll_runtime, &roll_output));
    CHECK(fabsf(roll_output.aileron_norm - 0.15f) < 1e-6f && !roll_output.saturated);

    ap_config_t config = {
        .roll = {{2.0f, 0.3f}, {0.5f, 0.25f, 0.5f}},
        .pitch = {{2.0f, 0.2f}, {0.5f, 0.25f, 0.5f}},
        .airspeed = {0.1f, 0.05f, 0.0f, 1.0f},
        .altitude = {0.015f, 0.05f, 0.06f},
        .attitude_limits = {0.5f, 0.4f}
    };
    ap_controller_t controller;
    CHECK(ap_controller_init(&controller, &config));
    ap_input_t input = {0};
    input.mode = AP_MODE_ATTITUDE_HOLD;
    input.dt_s = 0.1f;
    input.bank_command_rad = 0.4f;
    input.pitch_command_rad = 0.3f;
    ap_output_t output = {0};
    CHECK(ap_controller_step(&controller, &input, &output));
    CHECK(output.roll_rate_limited && output.pitch_rate_limited);
    CHECK(output.roll_rate_command_rad_s == 0.3f && output.pitch_rate_command_rad_s == 0.2f);
    CHECK(controller.runtime.roll.integral_norm > 0.0f && controller.runtime.pitch.integral_norm > 0.0f);
    CHECK(output.controls.aileron > 0.0f && output.controls.elevator < 0.0f);

    /* A late-axis failure rolls back the earlier axis's candidate integration. */
    controller.config.pitch.rate.ki = NAN;
    unsigned char saved_controller[sizeof(controller)], saved_output[sizeof(output)];
    memcpy(saved_controller, &controller, sizeof(controller));
    memcpy(saved_output, &output, sizeof(output));
    CHECK(!ap_controller_step(&controller, &input, &output));
    CHECK(memcmp(saved_controller, &controller, sizeof(controller)) == 0);
    CHECK(memcmp(saved_output, &output, sizeof(output)) == 0);
    controller.config.pitch.rate.ki = 0.25f;
    input.mode = AP_MODE_ATTITUDE_AIRSPEED_HOLD;
    input.state.airspeed_m_s = 50.0f;
    input.airspeed_command_m_s = NAN;
    memcpy(saved_controller, &controller, sizeof(controller));
    CHECK(!ap_controller_step(&controller, &input, &output));
    CHECK(memcmp(saved_controller, &controller, sizeof(controller)) == 0);
    CHECK(memcmp(saved_output, &output, sizeof(output)) == 0);

    input.mode = AP_MODE_ROLL_HOLD;
    CHECK(ap_controller_step(&controller, &input, &output));
    CHECK(controller.runtime.pitch.integral_norm == 0.0f && controller.runtime.roll.integral_norm > 0.0f);
    CHECK(output.pitch_rate_command_rad_s == 0.0f && !output.pitch_rate_limited);
    input.mode = AP_MODE_PITCH_HOLD;
    CHECK(ap_controller_step(&controller, &input, &output));
    CHECK(controller.runtime.roll.integral_norm == 0.0f && controller.runtime.pitch.integral_norm > 0.0f);
    input.mode = AP_MODE_MANUAL;
    CHECK(ap_controller_step(&controller, &input, &output));
    CHECK(controller.runtime.roll.integral_norm == 0.0f && controller.runtime.pitch.integral_norm == 0.0f);
    CHECK(controller.runtime.airspeed.integral_norm == 0.0f);
    return 0;
}

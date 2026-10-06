#include "autopilot/autopilot.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "Line %d: %s\n", __LINE__, #c); return 1; } } while (0)

int main(void)
{
    ap_turn_coordination_config_t tc = {0.3f, 25.0f, 0.785f};
    ap_turn_coordination_input_t ti = {0.3f, 0.0f, 50.0f};
    ap_turn_coordination_output_t to;
    CHECK(ap_turn_coordination_compute(&tc, &ti, &to));
    const float expected = 9.80665f * sinf(0.3f) / 50.0f;
    CHECK(fabsf(to.yaw_rate_command_rad_s - expected) < 1e-6f);
    CHECK(fabsf(to.pitch_rate_feedforward_rad_s - expected * tanf(0.3f)) < 1e-6f);
    const float pitch_ff = to.pitch_rate_feedforward_rad_s;
    ti.measured_bank_rad = -0.3f;
    CHECK(ap_turn_coordination_compute(&tc, &ti, &to));
    CHECK(fabsf(to.yaw_rate_command_rad_s + expected) < 1e-6f);
    CHECK(fabsf(to.pitch_rate_feedforward_rad_s - pitch_ff) < 1e-6f);
    ti.true_airspeed_m_s = 0.0f;
    CHECK(ap_turn_coordination_compute(&tc, &ti, &to) && to.airspeed_guarded);
    CHECK(fabsf(to.yaw_rate_command_rad_s + 2.0f * expected) < 1e-6f);
    tc.max_rate_rad_s = 0.01f;
    ti.measured_bank_rad = 2.0f;
    CHECK(ap_turn_coordination_compute(&tc, &ti, &to));
    CHECK(to.bank_limited && to.rate_limited && to.yaw_rate_command_rad_s == 0.01f);
    const ap_turn_coordination_output_t saved_turn = to;
    ti.true_airspeed_m_s = -1.0f;
    CHECK(!ap_turn_coordination_compute(&tc, &ti, &to));
    CHECK(memcmp(&to, &saved_turn, sizeof(to)) == 0);
    ti.true_airspeed_m_s = NAN;
    CHECK(!ap_turn_coordination_compute(&tc, &ti, &to));

    for (int sign = -1; sign <= 1; sign += 2) {
        ap_yaw_rate_config_t yc = {0.5f, 0.25f, 0.5f, (float)sign};
        ap_yaw_rate_runtime_t yr = {0};
        ap_yaw_rate_input_t yi = {0.1f, 0.4f, 0.0f, 1.0f};
        ap_yaw_rate_output_t yo;
        CHECK(ap_yaw_rate_compute(&yc, &yi, &yr, &yo));
        CHECK(fabsf(yo.rudder_norm - 0.225f * sign) < 1e-6f);
        CHECK(fabsf(yr.integral_norm - 0.075f) < 1e-6f);
        const float retained = yr.integral_norm;
        for (int direction = -1; direction <= 1; direction += 2) {
            yi.target_body_rate_rad_s = 10.0f * direction;
            for (int tick = 0; tick < 100; ++tick) {
                CHECK(ap_yaw_rate_compute(&yc, &yi, &yr, &yo));
                CHECK(yo.saturated && yo.rudder_norm == 0.5f * sign * direction);
                CHECK(yr.integral_norm == retained);
            }
        }
        yr.integral_norm = 0.3f;
        yi = (ap_yaw_rate_input_t){0.0f, -0.2f, 0.4f * sign, 1.0f};
        CHECK(ap_yaw_rate_compute(&yc, &yi, &yr, &yo));
        CHECK(fabsf(yr.integral_norm - 0.25f) < 1e-6f && yo.saturated);
        const ap_yaw_rate_output_t saved = yo;
        yi.dt_s = 0.0f;
        CHECK(!ap_yaw_rate_compute(&yc, &yi, &yr, &yo));
        CHECK(memcmp(&saved, &yo, sizeof(yo)) == 0 && yr.integral_norm == 0.25f);
        yi.dt_s = 1.0f; yc.rudder_sign = 0.0f;
        CHECK(!ap_yaw_rate_compute(&yc, &yi, &yr, &yo));
    }

    ap_config_t config = {
        .roll = {{2.0f, 1.0f}, {0.5f, 0.2f, 1.0f}},
        .pitch = {{1.5f, 0.5f}, {0.5f, 0.2f, 0.5f}},
        .airspeed = {0.08f, 0.005f, 0.0f, 1.0f},
        .altitude = {0.015f, 0.05f, 0.052f},
        .attitude_limits = {0.35f, 0.17f},
        .yaw = {true, {0.5f, 0.25f, 0.5f, 1.0f}, {0.3f, 25.0f, 0.785f}}
    };
    ap_runtime_t runtime = {0};
    ap_input_t input = {0};
    input.state.roll_rad = 0.3f;
    input.state.airspeed_m_s = 50.0f;
    input.mode = AP_MODE_ATTITUDE_HOLD;
    input.dt_s = 0.1f;
    input.requested.rudder = 0.01f;
    ap_output_t output;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(output.yaw_control_active && output.controls.rudder > 0.01f);
    CHECK(fabsf(output.yaw_rate_command_rad_s - expected) < 1e-6f); /* measured bank, target is zero */
    CHECK(output.pitch_coordination_ff_rad_s > 0.0f && runtime.yaw.integral_norm > 0.0f);
    config.pitch.attitude.max_rate_rad_s = 0.0001f;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(output.pitch_rate_limited && output.pitch_rate_command_rad_s == 0.0001f);
    unsigned char saved_runtime[sizeof(runtime)], saved_output[sizeof(output)];
    memcpy(saved_runtime, &runtime, sizeof(runtime));
    memcpy(saved_output, &output, sizeof(output));
    config.yaw.rate.kp = NAN;
    CHECK(!ap_step(&config, &input, &runtime, &output));
    CHECK(memcmp(saved_runtime, &runtime, sizeof(runtime)) == 0);
    CHECK(memcmp(saved_output, &output, sizeof(output)) == 0);
    config.yaw.rate.kp = 0.5f;
    input.mode = AP_MODE_ATTITUDE_AIRSPEED_HOLD;
    input.airspeed_command_m_s = NAN;
    CHECK(!ap_step(&config, &input, &runtime, &output));
    CHECK(memcmp(saved_runtime, &runtime, sizeof(runtime)) == 0);
    CHECK(memcmp(saved_output, &output, sizeof(output)) == 0);
    input.mode = AP_MODE_ROLL_HOLD;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(!output.yaw_control_active && output.controls.rudder == 0.01f && runtime.yaw.integral_norm == 0.0f);
    input.mode = AP_MODE_ATTITUDE_HOLD;
    config.yaw.enabled = false;
    CHECK(ap_step(&config, &input, &runtime, &output));
    CHECK(!output.yaw_control_active && output.controls.rudder == 0.01f);
    runtime.yaw.integral_norm = 0.2f;
    input.mode = AP_MODE_MANUAL;
    CHECK(ap_step(NULL, &input, &runtime, &output) && runtime.yaw.integral_norm == 0.0f);
    input.requested.rudder = 2.0f;
    CHECK(ap_step(NULL, &input, &runtime, &output));
    CHECK(output.controls.rudder == 1.0f && output.rudder_saturated && !output.yaw_control_active);
    return 0;
}

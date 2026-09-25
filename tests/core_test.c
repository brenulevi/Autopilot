#include "autopilot/autopilot.h"

#include <math.h>
#include <float.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

int main(void)
{
    ap_input_t input = {0};
    ap_output_t output = {0};
    ap_config_t config = {2.0f, 0.3f, 0.35f, 0.5f, 3.0f, 0.5f, 0.2f, 0.5f};
    input.dt_s = 0.01f;
    input.requested = (ap_controls_t){2.0f, -2.0f, 0.25f, 1.2f};
    CHECK(ap_step(NULL, &input, &output));
    CHECK(output.controls.aileron == 1.0f && output.controls.elevator == -1.0f);
    CHECK(output.controls.rudder == 0.25f && output.controls.throttle == 1.0f);
    CHECK(output.aileron_saturated && !output.bank_command_limited);
    CHECK(output.elevator_saturated);

    input.requested.throttle = -1.0f;
    CHECK(ap_step(NULL, &input, &output));
    CHECK(output.controls.throttle == 0.0f);

    /* Invalid samples must be reported without partially changing outputs. */
    input.state.roll_rad = NAN;
    input.requested.aileron = 0.0f;
    CHECK(!ap_step(NULL, &input, &output));
    CHECK(output.controls.aileron == 1.0f);
    input.state.roll_rad = 0.0f;
    input.requested.elevator = INFINITY;
    CHECK(!ap_step(NULL, &input, &output));
    input.requested.elevator = 0.0f;
    input.dt_s = 0.0f;
    CHECK(!ap_step(NULL, &input, &output));
    CHECK(!ap_step(NULL, NULL, &output));
    CHECK(!ap_step(NULL, &input, NULL));

    input.dt_s = 0.01f;
    input.mode = AP_MODE_ROLL_HOLD;
    input.requested = (ap_controls_t){-0.07f, 0.1f, -0.02f, 0.8f};
    input.bank_command_rad = 0.0f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.controls.aileron == input.requested.aileron); /* Preserve trim. */
    CHECK(output.controls.elevator == 0.1f && output.controls.rudder == -0.02f);
    CHECK(output.controls.throttle == 0.8f);

    input.bank_command_rad = 0.1f;
    CHECK(ap_step(&config, &input, &output));
    const float without_damping = output.controls.aileron;
    CHECK(without_damping > input.requested.aileron);
    input.state.p_rad_s = 0.2f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.controls.aileron < without_damping); /* Resist positive roll rate. */
    input.state.p_rad_s = 0.0f;
    input.bank_command_rad = -0.1f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.controls.aileron < input.requested.aileron);

    /* Both command and actuator limits, in both directions. */
    input.bank_command_rad = 2.0f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.bank_command_limited && output.bank_command_rad == 0.35f);
    CHECK(output.aileron_saturated && output.controls.aileron == 0.5f);
    input.bank_command_rad = -2.0f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.bank_command_limited && output.bank_command_rad == -0.35f);
    CHECK(output.aileron_saturated && output.controls.aileron == -0.5f);

    CHECK(!ap_step(NULL, &input, &output));
    config.roll_angle_gain = -1.0f;
    CHECK(!ap_step(&config, &input, &output));
    config.roll_angle_gain = NAN;
    CHECK(!ap_step(&config, &input, &output));
    config.roll_angle_gain = FLT_MAX;
    input.state.roll_rad = -FLT_MAX;
    CHECK(!ap_step(&config, &input, &output));
    CHECK(output.controls.aileron == -0.5f && output.bank_command_limited);
    input.state.roll_rad = 0.0f;
    config.roll_angle_gain = 2.0f;
    config.max_aileron = 1.1f;
    CHECK(!ap_step(&config, &input, &output));
    config.max_aileron = 0.5f;
    input.bank_command_rad = NAN;
    CHECK(!ap_step(&config, &input, &output));

    /* Manual mode ignores roll configuration/setpoint and has no stale state. */
    input.mode = AP_MODE_MANUAL;
    CHECK(ap_step(NULL, &input, &output));
    CHECK(output.controls.aileron == input.requested.aileron);
    CHECK(!output.aileron_saturated && !output.bank_command_limited);
    CHECK(output.bank_command_rad == 0.0f);

    /* Pitch hold uses the measured C172X elevator sign, trim and rate damping. */
    input.mode = AP_MODE_PITCH_HOLD;
    input.pitch_command_rad = 0.1f;
    input.state.pitch_rad = 0.0f;
    input.state.q_rad_s = 0.0f;
    input.requested.elevator = 0.05f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.controls.elevator < input.requested.elevator);
    CHECK(output.controls.aileron == input.requested.aileron);
    const float without_pitch_damping = output.controls.elevator;
    input.state.q_rad_s = 0.1f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.controls.elevator > without_pitch_damping);
    input.state.q_rad_s = 0.0f;
    input.pitch_command_rad = 2.0f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.pitch_command_limited && output.pitch_command_rad == 0.2f);
    CHECK(output.elevator_saturated && output.controls.elevator == -0.5f);
    input.pitch_command_rad = -2.0f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.pitch_command_limited && output.pitch_command_rad == -0.2f);
    CHECK(output.elevator_saturated && output.controls.elevator == 0.5f);
    config.pitch_angle_gain = NAN;
    CHECK(!ap_step(&config, &input, &output));
    config.pitch_angle_gain = 3.0f;
    input.pitch_command_rad = NAN;
    CHECK(!ap_step(&config, &input, &output));
    input.pitch_command_rad = 0.0f;
    input.mode = AP_MODE_MANUAL;
    CHECK(ap_step(NULL, &input, &output));
    CHECK(!output.pitch_command_limited && !output.elevator_saturated);

    /* Combined mode computes both axes, preserves the others, and commits atomically. */
    input.mode = AP_MODE_ATTITUDE_HOLD;
    input.bank_command_rad = 0.1f;
    input.pitch_command_rad = 0.1f;
    CHECK(ap_step(&config, &input, &output));
    CHECK(output.controls.aileron > input.requested.aileron);
    CHECK(output.controls.elevator < input.requested.elevator);
    CHECK(output.controls.rudder == input.requested.rudder &&
          output.controls.throttle == input.requested.throttle);
    CHECK(output.bank_command_rad == 0.1f && output.pitch_command_rad == 0.1f);
    const ap_output_t previous = output;
    config.pitch_angle_gain = NAN;
    CHECK(!ap_step(&config, &input, &output));
    CHECK(output.controls.aileron == previous.controls.aileron &&
          output.controls.elevator == previous.controls.elevator);
    config.pitch_angle_gain = 3.0f;
    input.mode = (ap_mode_t)99;
    CHECK(!ap_step(&config, &input, &output));
    return 0;
}

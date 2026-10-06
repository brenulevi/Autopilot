#include "autopilot/autopilot.h"
#include "control/manual.h"
#include "guidance/bank_limit.h"
#include "guidance/pitch_limit.h"

#include <math.h>
#include <stddef.h>

static bool valid_sample(const ap_input_t *input)
{
    return isfinite(input->dt_s) && input->dt_s > 0.0f &&
           isfinite(input->state.roll_rad) && isfinite(input->state.pitch_rad) &&
           isfinite(input->state.yaw_rad) && isfinite(input->state.p_rad_s) &&
           isfinite(input->state.q_rad_s) && isfinite(input->state.r_rad_s) &&
           isfinite(input->state.airspeed_m_s) && isfinite(input->state.altitude_m) &&
           isfinite(input->state.climb_rate_m_s) &&
           isfinite(input->requested.aileron) && isfinite(input->requested.elevator) &&
           isfinite(input->requested.rudder) && isfinite(input->requested.throttle);
}

static bool ap_step_attitude(const ap_config_t *config, const ap_input_t *input,
                             ap_runtime_t *runtime, ap_output_t *output)
{
    if (input == NULL || output == NULL || !valid_sample(input)) return false;

    /* Work locally so errors never leave a partly updated output. */
    ap_output_t result = {0};
    ap_manual_bound(&input->requested, &result);
    ap_turn_coordination_output_t turn = {0};

    switch (input->mode) {
    case AP_MODE_MANUAL:
        runtime->roll.integral_norm = 0.0f;
        runtime->pitch.integral_norm = 0.0f;
        runtime->yaw.integral_norm = 0.0f;
        break;
    case AP_MODE_ROLL_HOLD:
    case AP_MODE_PITCH_HOLD:
    case AP_MODE_ATTITUDE_HOLD:
        if (config == NULL) return false;
        const bool yaw_active = input->mode == AP_MODE_ATTITUDE_HOLD && config->yaw.enabled;
        if (yaw_active) {
            const ap_turn_coordination_input_t turn_input = {
                input->state.roll_rad, input->state.pitch_rad, input->state.airspeed_m_s
            };
            if (!ap_turn_coordination_compute(&config->yaw.coordination, &turn_input, &turn)) return false;
        } else {
            runtime->yaw.integral_norm = 0.0f;
        }
        if (input->mode == AP_MODE_PITCH_HOLD) runtime->roll.integral_norm = 0.0f;
        if (input->mode == AP_MODE_ROLL_HOLD) runtime->pitch.integral_norm = 0.0f;
        if (input->mode != AP_MODE_PITCH_HOLD) {
            if (!ap_guidance_limit_bank(input->bank_command_rad,
                                        config->attitude_limits.max_bank_rad,
                                        &result.bank_command_rad,
                                        &result.bank_command_limited)) return false;
            const ap_roll_attitude_input_t attitude = {
                input->state.roll_rad, result.bank_command_rad
            };
            ap_roll_attitude_output_t reference;
            if (!ap_roll_attitude_compute(&config->roll.attitude, &attitude, &reference)) return false;
            const ap_roll_rate_input_t rate_input = {
                input->state.p_rad_s, reference.body_rate_command_rad_s,
                input->requested.aileron, input->dt_s
            };
            ap_roll_rate_output_t rate_output;
            if (!ap_roll_rate_compute(&config->roll.rate, &rate_input,
                                        &runtime->roll, &rate_output)) return false;
            result.controls.aileron = rate_output.aileron_norm;
            result.aileron_saturated = rate_output.saturated;
            result.roll_rate_command_rad_s = reference.body_rate_command_rad_s;
            result.roll_rate_error_rad_s = rate_output.error_rad_s;
            result.roll_rate_limited = reference.limited;
        }
        if (input->mode != AP_MODE_ROLL_HOLD) {
            if (!ap_guidance_limit_pitch(input->pitch_command_rad,
                                         config->attitude_limits.max_pitch_rad,
                                         &result.pitch_command_rad,
                                         &result.pitch_command_limited)) return false;
            const ap_pitch_attitude_input_t attitude = {
                input->state.pitch_rad, result.pitch_command_rad
            };
            ap_pitch_attitude_output_t reference;
            if (!ap_pitch_attitude_compute(&config->pitch.attitude, &attitude, &reference)) return false;
            const float raw_pitch_rate = reference.body_rate_command_rad_s + turn.pitch_rate_feedforward_rad_s;
            if (!isfinite(raw_pitch_rate)) return false;
            const float pitch_rate_limit = config->pitch.attitude.max_rate_rad_s;
            if (raw_pitch_rate > pitch_rate_limit) reference.body_rate_command_rad_s = pitch_rate_limit;
            else if (raw_pitch_rate < -pitch_rate_limit) reference.body_rate_command_rad_s = -pitch_rate_limit;
            else reference.body_rate_command_rad_s = raw_pitch_rate;
            reference.limited = reference.limited || reference.body_rate_command_rad_s != raw_pitch_rate;
            const ap_pitch_rate_input_t rate_input = {
                input->state.q_rad_s, reference.body_rate_command_rad_s,
                input->requested.elevator, input->dt_s
            };
            ap_pitch_rate_output_t rate_output;
            if (!ap_pitch_rate_compute(&config->pitch.rate, &rate_input,
                                        &runtime->pitch, &rate_output)) return false;
            result.controls.elevator = rate_output.elevator_norm;
            result.elevator_saturated = rate_output.saturated;
            result.pitch_rate_command_rad_s = reference.body_rate_command_rad_s;
            result.pitch_rate_error_rad_s = rate_output.error_rad_s;
            result.pitch_rate_limited = reference.limited;
        }
        if (yaw_active) {
            const ap_yaw_rate_input_t yaw_input = {
                input->state.r_rad_s, turn.yaw_rate_command_rad_s,
                input->requested.rudder, input->dt_s
            };
            ap_yaw_rate_output_t yaw_output;
            if (!ap_yaw_rate_compute(&config->yaw.rate, &yaw_input, &runtime->yaw, &yaw_output)) return false;
            result.controls.rudder = yaw_output.rudder_norm;
            result.rudder_saturated = yaw_output.saturated;
            result.yaw_control_active = true;
            result.yaw_rate_command_rad_s = turn.yaw_rate_command_rad_s;
            result.yaw_rate_error_rad_s = yaw_output.error_rad_s;
            result.pitch_coordination_ff_rad_s = turn.pitch_rate_feedforward_rad_s;
            result.yaw_rate_limited = turn.rate_limited;
            result.yaw_airspeed_guarded = turn.airspeed_guarded;
            result.yaw_bank_limited = turn.bank_limited;
        }
        break;
    default:
        return false;
    }

    *output = result;
    return true;
}

bool ap_step(const ap_config_t *config, const ap_input_t *input,
             ap_runtime_t *runtime, ap_output_t *output)
{
    if (input == NULL || runtime == NULL || output == NULL) return false;
    ap_output_t result;
    ap_runtime_t next_runtime = *runtime;

    if (input->mode != AP_MODE_ATTITUDE_AIRSPEED_HOLD &&
        input->mode != AP_MODE_ALTITUDE_AIRSPEED_HOLD) {
        if (!ap_step_attitude(config, input, &next_runtime, &result)) return false;
        next_runtime.airspeed.integral_norm = 0.0f;
        *output = result;
        *runtime = next_runtime;
        return true;
    }

    ap_input_t attitude_input = *input;
    attitude_input.mode = AP_MODE_ATTITUDE_HOLD;
    bool altitude_pitch_limited = false;
    if (input->mode == AP_MODE_ALTITUDE_AIRSPEED_HOLD) {
        if (config == NULL) return false;
        const ap_altitude_input_t altitude_input = {
            .measured_altitude_m = input->state.altitude_m,
            .measured_climb_rate_m_s = input->state.climb_rate_m_s,
            .target_altitude_m = input->altitude_command_m,
            .trim_pitch_rad = input->pitch_command_rad
        };
        ap_altitude_output_t altitude_output;
        if (!ap_guidance_altitude(&config->altitude, &altitude_input,
                                  &altitude_output)) return false;
        attitude_input.pitch_command_rad = altitude_output.pitch_command_rad;
        altitude_pitch_limited = altitude_output.limited;
    }
    if (!ap_step_attitude(config, &attitude_input, &next_runtime, &result)) return false;
    const ap_airspeed_input_t airspeed_input = {
        .measured_airspeed_m_s = input->state.airspeed_m_s,
        .target_airspeed_m_s = input->airspeed_command_m_s,
        .trim_throttle_norm = input->requested.throttle,
        .dt_s = input->dt_s
    };
    ap_airspeed_output_t airspeed_output;
    if (!ap_airspeed_compute(&config->airspeed, &airspeed_input,
                             &next_runtime.airspeed, &airspeed_output)) return false;
    result.controls.throttle = airspeed_output.throttle_norm;
    result.airspeed_command_m_s = input->airspeed_command_m_s;
    result.airspeed_error_m_s = airspeed_output.error_m_s;
    result.throttle_saturated = airspeed_output.saturated;

    if (input->mode == AP_MODE_ALTITUDE_AIRSPEED_HOLD) {
        result.altitude_command_m = input->altitude_command_m;
        result.altitude_error_m = input->altitude_command_m - input->state.altitude_m;
        result.altitude_pitch_limited = altitude_pitch_limited;
    }

    *output = result;
    *runtime = next_runtime;
    return true;
}

bool ap_controller_init(ap_controller_t *controller, const ap_config_t *config)
{
    if (controller == NULL || !ap_control_config_validate(config)) return false;
    const ap_controller_t next = {.config = *config, .runtime = {0}};
    *controller = next;
    return true;
}

bool ap_controller_step(ap_controller_t *controller, const ap_input_t *input,
                        ap_output_t *output)
{
    if (controller == NULL) return false;
    return ap_step(&controller->config, input, &controller->runtime, output);
}

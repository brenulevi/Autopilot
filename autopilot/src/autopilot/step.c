#include "autopilot/autopilot.h"
#include "control/manual.h"
#include "control/roll.h"
#include "control/pitch.h"
#include "control/airspeed.h"
#include "guidance/bank_limit.h"
#include "guidance/pitch_limit.h"
#include "guidance/altitude.h"

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
                             ap_output_t *output)
{
    if (input == NULL || output == NULL || !valid_sample(input)) return false;

    /* Work locally so errors never leave a partly updated output. */
    ap_output_t result = {0};
    ap_manual_bound(&input->requested, &result);

    switch (input->mode) {
    case AP_MODE_MANUAL:
        break;
    case AP_MODE_ROLL_HOLD:
    case AP_MODE_PITCH_HOLD:
    case AP_MODE_ATTITUDE_HOLD:
        if (config == NULL) return false;
        if (input->mode != AP_MODE_PITCH_HOLD) {
            if (!ap_guidance_limit_bank(input->bank_command_rad, config->max_bank_rad,
                                        &result.bank_command_rad, &result.bank_command_limited) ||
                !ap_roll_compute(config, &input->state, result.bank_command_rad,
                                 input->requested.aileron, &result.controls.aileron,
                                 &result.aileron_saturated)) return false;
        }
        if (input->mode != AP_MODE_ROLL_HOLD) {
            if (!ap_guidance_limit_pitch(input->pitch_command_rad, config->max_pitch_rad,
                                         &result.pitch_command_rad, &result.pitch_command_limited) ||
                !ap_pitch_compute(config, &input->state, result.pitch_command_rad,
                                  input->requested.elevator, &result.controls.elevator,
                                  &result.elevator_saturated)) return false;
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
        if (!ap_step_attitude(config, input, &result)) return false;
        next_runtime.airspeed_integral_norm = 0.0f;
        *output = result;
        *runtime = next_runtime;
        return true;
    }

    ap_input_t attitude_input = *input;
    attitude_input.mode = AP_MODE_ATTITUDE_HOLD;
    bool altitude_pitch_limited = false;
    if (input->mode == AP_MODE_ALTITUDE_AIRSPEED_HOLD &&
        !ap_guidance_altitude(config, input, &attitude_input.pitch_command_rad,
                              &altitude_pitch_limited)) return false;
    if (!ap_step_attitude(config, &attitude_input, &result) ||
        !ap_airspeed_compute(config, input, &next_runtime, &result)) return false;

    if (input->mode == AP_MODE_ALTITUDE_AIRSPEED_HOLD) {
        result.altitude_command_m = input->altitude_command_m;
        result.altitude_error_m = input->altitude_command_m - input->state.altitude_m;
        result.altitude_pitch_limited = altitude_pitch_limited;
    }

    *output = result;
    *runtime = next_runtime;
    return true;
}

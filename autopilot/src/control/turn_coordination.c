#include "autopilot/control/turn_coordination.h"
#include <math.h>
#include <stddef.h>

static float bound(float value, float limit)
{ return value < -limit ? -limit : (value > limit ? limit : value); }

bool ap_turn_coordination_compute(const ap_turn_coordination_config_t *config,
                                  const ap_turn_coordination_input_t *input,
                                  ap_turn_coordination_output_t *output)
{
    if (config == NULL || input == NULL || output == NULL ||
        !isfinite(config->max_rate_rad_s) || config->max_rate_rad_s <= 0.0f ||
        !isfinite(config->min_airspeed_m_s) || config->min_airspeed_m_s <= 0.0f ||
        !isfinite(config->max_bank_rad) || config->max_bank_rad <= 0.0f || config->max_bank_rad >= 1.570796327f ||
        !isfinite(input->measured_bank_rad) || !isfinite(input->measured_pitch_rad) ||
        !isfinite(input->true_airspeed_m_s) || input->true_airspeed_m_s < 0.0f)
        return false;
    const float bank = bound(input->measured_bank_rad, config->max_bank_rad);
    const bool guarded = input->true_airspeed_m_s < config->min_airspeed_m_s;
    const float speed = guarded ? config->min_airspeed_m_s : input->true_airspeed_m_s;
    const float raw = 9.80665f * sinf(bank) * cosf(input->measured_pitch_rad) / speed;
    if (!isfinite(raw)) return false;
    const float reference = bound(raw, config->max_rate_rad_s);
    const float pitch_ff = reference * tanf(bank);
    if (!isfinite(pitch_ff)) return false;
    const ap_turn_coordination_output_t result = {
        reference, pitch_ff, reference != raw, guarded, bank != input->measured_bank_rad
    };
    *output = result;
    return true;
}

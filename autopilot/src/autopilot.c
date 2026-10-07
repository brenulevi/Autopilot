#include "autopilot/autopilot.h"

#include <stddef.h>

bool autopilot_step(const autopilot_config_t *config,
                    const autopilot_input_t *input,
                    autopilot_state_t *state,
                    autopilot_output_t *output)
{
    if (config == NULL || input == NULL || state == NULL || output == NULL) return false;

    float aileron_command;
    float elevator_command;
    float throttle_command;
    autopilot_state_t next_state = *state;
    if (!control_roll_step(input->target_bank_rad,
                           input->measured_bank_rad,
                           input->measured_roll_rate_rad_s,
                           &config->roll, &config->aileron,
                           &aileron_command) ||
        !control_pitch_step(input->target_pitch_rad,
                            input->measured_pitch_rad,
                            input->measured_pitch_rate_rad_s,
                            &config->pitch, &config->elevator,
                            &elevator_command) ||
        !control_airspeed_step(input->target_airspeed_m_s,
                               input->measured_airspeed_m_s,
                               input->dt_s,
                               &config->airspeed, &config->throttle,
                               &next_state.airspeed,
                               &throttle_command)) return false;

    *state = next_state;
    output->aileron_command = aileron_command;
    output->elevator_command = elevator_command;
    output->throttle_command = throttle_command;
    return true;
}

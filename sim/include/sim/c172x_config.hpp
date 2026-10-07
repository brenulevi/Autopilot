#pragma once

#include "autopilot/autopilot.h"
#include "sim/jsbsim_adapter.hpp"

namespace sim {

/* Simulation-only C172X gains and normalized surface limits. JSBSim supplies
 * the trim commands for this initial flight condition. */
inline autopilot_config_t c172x_config(const Controls& trim)
{
    return {
        {4.0f, 0.5f},
        {10.0f, 3.0f},
        {0.08f, 0.01f},
        {trim.aileron, -0.5f, 0.5f},
        {trim.elevator, -0.5f, 0.5f},
        {trim.throttle, 0.0f, 1.0f}
    };
}

} // namespace sim

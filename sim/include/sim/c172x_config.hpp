#pragma once

#include "autopilot/autopilot.h"

namespace sim {
inline constexpr double radians_per_degree = 0.017453292519943295;

// C172X attitude/true-airspeed tuning at 3000 ft / 100 kt CAS. Checked at
// +/-5 deg bank, +/-2 deg pitch, +/-1 m/s true-speed steps, and +/-5 m
// altitude steps with a small banked segment.
// These values are not Skyward gains and do not define a validated envelope.
inline constexpr ap_config_t c172x_config{
    4.0f, 0.5f, static_cast<float>(20.0 * radians_per_degree), 1.0f,
    10.0f, 3.0f, static_cast<float>(10.0 * radians_per_degree), 0.5f,
    0.08f, 0.005f, 0.0f, 1.0f,
    0.015f, 0.05f, static_cast<float>(3.0 * radians_per_degree)
};
} // namespace sim

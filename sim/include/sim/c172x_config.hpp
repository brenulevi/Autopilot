#pragma once

#include "autopilot/autopilot.h"

namespace sim {
inline constexpr double radians_per_degree = 0.017453292519943295;

// C172X attitude/true-airspeed tuning at 3000 ft / 100 kt CAS. Checked at
// +/-5 deg bank, +/-2 deg pitch, +/-1 m/s true-speed steps, and +/-5 m
// altitude steps with a small banked segment.
// Inner loops are checked separately with +/-0.08 rad/s roll and
// +/-0.03 rad/s pitch steps before the attitude/guidance regressions.
// Yaw PI is checked with +/-0.02 rad/s steps and ramped +/-15 deg turns.
// These values are not Skyward gains and do not define a validated envelope.
inline constexpr ap_config_t c172x_config{
    {{2.0f, 1.0f}, {2.0f, 1.0f, 1.0f}}, // roll attitude, rate PI
    {{1.5f, 0.5f}, {4.0f, 4.0f, 0.5f}}, // pitch attitude, rate PI
    {0.08f, 0.005f, 0.0f, 1.0f}, // airspeed
    {0.015f, 0.05f, static_cast<float>(3.0 * radians_per_degree)}, // altitude
    {static_cast<float>(20.0 * radians_per_degree),
     static_cast<float>(10.0 * radians_per_degree)}, // attitude limits
    {true, {10.0f, 10.0f, 0.5f, -1.0f},
     {0.3f, 25.0f, static_cast<float>(45.0 * radians_per_degree)}} // yaw PI, turn guards
};
} // namespace sim

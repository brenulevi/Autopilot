#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void run_direction(const char* root, double offset_deg)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto config = sim::c172x_config;
    const float trim_pitch = aircraft.state().pitch_rad;
    double hold_error_deg = 0.0;
    double return_error_deg = 0.0;
    double max_elevator = 0.0;
    for (int k = 0; k < 2000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const float command = trim_pitch + (time >= 2.0 && time < 10.0 ?
            static_cast<float>(offset_deg * sim::radians_per_degree) : 0.0f);
        const ap_input_t input{state, trim, 0.01f, AP_MODE_PITCH_HOLD, 0.0f, command};
        ap_output_t result{};
        check(ap_step(&config, &input, &result), "Pitch hold rejected simulation state");
        check(!result.elevator_saturated, "Small pitch command saturated elevator");
        check(!result.pitch_command_limited, "Small pitch command hit pitch limit");
        check(result.controls.aileron == trim.aileron && result.controls.rudder == trim.rudder &&
              result.controls.throttle == trim.throttle, "Pitch hold changed another control axis");
        const double error_deg = std::abs(command - state.pitch_rad) / sim::radians_per_degree;
        if (time >= 6.0 && time < 10.0) hold_error_deg = std::max(hold_error_deg, error_deg);
        if (time >= 14.0) return_error_deg = std::max(return_error_deg, error_deg);
        max_elevator = std::max(max_elevator, std::abs(static_cast<double>(result.controls.elevator)));
        aircraft.step(result.controls);
    }
    std::cout << offset_deg << " deg: hold error <= " << hold_error_deg
              << " deg; return error <= " << return_error_deg
              << " deg; max |elevator| " << max_elevator << '\n';
    check(hold_error_deg < 0.5, "Pitch failed to stay within 0.5 deg after 4 s");
    check(return_error_deg < 0.5, "Pitch return failed to stay within 0.5 deg after 4 s");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        run_direction(argv[1], 2.0);
        run_direction(argv[1], -2.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

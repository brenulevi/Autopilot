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

void run_direction(const char* root, double bank_deg)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto initial_roll = aircraft.state().roll_rad;
    const auto config = sim::c172x_config;
    const double target = bank_deg * sim::radians_per_degree;
    double max_hold_error_deg = 0.0;
    double max_return_error_deg = 0.0;
    double max_overshoot_deg = 0.0;
    double max_aileron = 0.0;
    ap_runtime_t runtime{};
    for (int k = 0; k < 2000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const float command = time < 2.0 ? initial_roll :
            (time < 10.0 ? static_cast<float>(target) : 0.0f);
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ROLL_HOLD, command};
        ap_output_t result{};
        check(ap_step(&config, &input, &runtime, &result), "Roll hold rejected simulation state");
        check(!result.aileron_saturated, "Small bank command saturated the aileron");
        check(!result.bank_command_limited, "Small bank command hit the bank limit");
        check(result.controls.elevator == trim.elevator && result.controls.rudder == trim.rudder &&
              result.controls.throttle == trim.throttle, "Roll hold changed another control axis");
        max_aileron = std::max(max_aileron, std::abs(static_cast<double>(result.controls.aileron)));
        const double error_deg = std::abs(command - state.roll_rad) / sim::radians_per_degree;
        if (time >= 6.0 && time < 10.0) max_hold_error_deg = std::max(max_hold_error_deg, error_deg);
        if (time >= 14.0) max_return_error_deg = std::max(max_return_error_deg, error_deg);
        if (time >= 2.0 && time < 10.0) {
            const double directed_error = (bank_deg > 0 ? 1.0 : -1.0) *
                                          (state.roll_rad - target) / sim::radians_per_degree;
            max_overshoot_deg = std::max(max_overshoot_deg, directed_error);
        }
        aircraft.step(result.controls);
    }
    std::cout << bank_deg << " deg: hold error <= " << max_hold_error_deg
              << " deg; return error <= " << max_return_error_deg
              << " deg; overshoot " << max_overshoot_deg
              << " deg; max |aileron| " << max_aileron << '\n';
    // Deliberately loose regression bounds for this one model/operating point.
    check(max_hold_error_deg < 0.5, "Bank failed to stay within 0.5 deg after 4 s");
    check(max_return_error_deg < 0.5, "Wings-level return failed after 4 s");
    check(max_overshoot_deg < 1.0, "Bank overshoot exceeds 1 deg");
    check(std::abs(aircraft.state().roll_rad) / sim::radians_per_degree < 0.5,
          "Final bank error exceeds 0.5 deg");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        run_direction(argv[1], 5.0);
        run_direction(argv[1], -5.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

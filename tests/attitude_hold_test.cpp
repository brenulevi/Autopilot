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

void run_case(const char* root, double speed_kts, double bank_deg, double pitch_offset_deg)
{
    sim::JsbsimAdapter aircraft(root, 0.01, {}, speed_kts);
    const auto config = sim::c172x_config;
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    double max_bank_hold_error_deg = 0.0;
    double max_pitch_hold_error_deg = 0.0;
    double max_bank_return_error_deg = 0.0;
    double max_pitch_return_error_deg = 0.0;
    ap_runtime_t runtime{};
    for (int k = 0; k < 2000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const float bank_command = time < 2.0 ? initial.roll_rad :
            (time < 10.0 ? static_cast<float>(bank_deg * sim::radians_per_degree) : 0.0f);
        const float pitch_command = initial.pitch_rad + (time >= 2.0 && time < 10.0 ?
            static_cast<float>(pitch_offset_deg * sim::radians_per_degree) : 0.0f);
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ATTITUDE_HOLD,
                               bank_command, pitch_command};
        ap_output_t output{};
        check(ap_step(&config, &input, &runtime, &output), "Combined hold rejected simulation state");
        check(!output.aileron_saturated && !output.elevator_saturated,
              "Small combined command saturated an actuator");
        check(!output.bank_command_limited && !output.pitch_command_limited,
              "Small combined command hit an attitude limit");
        check(!output.rudder_saturated && output.yaw_control_active && output.controls.throttle == trim.throttle,
              "Combined hold saturated rudder, disabled yaw, or changed throttle");
        const double bank_error_deg = std::abs(bank_command - state.roll_rad) / sim::radians_per_degree;
        const double pitch_error_deg = std::abs(pitch_command - state.pitch_rad) / sim::radians_per_degree;
        if (time >= 6.0 && time < 10.0) {
            max_bank_hold_error_deg = std::max(max_bank_hold_error_deg, bank_error_deg);
            max_pitch_hold_error_deg = std::max(max_pitch_hold_error_deg, pitch_error_deg);
        }
        if (time >= 14.0) {
            max_bank_return_error_deg = std::max(max_bank_return_error_deg, bank_error_deg);
            max_pitch_return_error_deg = std::max(max_pitch_return_error_deg, pitch_error_deg);
        }
        aircraft.step(output.controls);
    }
    std::cout << speed_kts << " kt, bank " << bank_deg << " deg, pitch offset "
              << pitch_offset_deg << " deg: hold errors " << max_bank_hold_error_deg
              << "/" << max_pitch_hold_error_deg << " deg; return errors "
              << max_bank_return_error_deg << "/" << max_pitch_return_error_deg << " deg\n";
    check(max_bank_hold_error_deg < 0.5 && max_bank_return_error_deg < 0.5,
          "Combined hold bank error exceeded 0.5 deg in later intervals");
    check(max_pitch_hold_error_deg < 0.6 && max_pitch_return_error_deg < 0.6,
          "Combined hold pitch error exceeded 0.6 deg in later intervals");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        for (double speed_kts : {90.0, 100.0, 110.0}) {
            run_case(argv[1], speed_kts, 5.0, 2.0);
        }
        run_case(argv[1], 100.0, -5.0, -2.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

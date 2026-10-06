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

void run_case(const char* root, double step_m, double turn_bank_deg)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    const auto config = sim::c172x_config;
    ap_runtime_t runtime{};
    double hold_error = 0.0;
    double return_error = 0.0;
    double turn_error = 0.0;
    double speed_error = 0.0;
    double bank_error_deg = 0.0;
    int limited_samples = 0;
    int saturated_samples = 0;
    bool saw_positive_climb = false;
    bool saw_negative_climb = false;
    for (int k = 0; k < 9000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const float altitude_command = initial.altitude_m +
            (time >= 2.0 && time < 30.0 ? static_cast<float>(step_m) : 0.0f);
        const float bank_command = time >= 40.0 && time < 60.0 ?
            static_cast<float>(turn_bank_deg * sim::radians_per_degree) : 0.0f;
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ALTITUDE_AIRSPEED_HOLD,
                               bank_command, initial.pitch_rad, initial.airspeed_m_s,
                               altitude_command};
        ap_output_t output{};
        check(ap_step(&config, &input, &runtime, &output),
              "Altitude hold rejected simulation state");
        check(output.yaw_control_active && !output.rudder_saturated, "Altitude hold yaw control inactive or saturated");
        check(std::abs(output.altitude_error_m - (altitude_command - state.altitude_m)) < 1e-4f,
              "Altitude error has incorrect sign");
        check(output.controls.throttle >= 0.0f && output.controls.throttle <= 1.0f,
              "Throttle command out of range");
        check(std::abs(output.controls.elevator) <= config.pitch.rate.max_elevator_norm,
              "Elevator command out of range");
        if (time > 3.0 && time < 10.0 &&
            state.climb_rate_m_s * step_m > 0.25) saw_positive_climb = true;
        if (time > 31.0 && time < 40.0 &&
            state.climb_rate_m_s * step_m < -0.25) saw_negative_climb = true;
        if (output.altitude_pitch_limited) ++limited_samples;
        if (output.elevator_saturated || output.aileron_saturated || output.throttle_saturated)
            ++saturated_samples;
        const double error = std::abs(altitude_command - state.altitude_m);
        if (time >= 20.0 && time < 30.0) hold_error = std::max(hold_error, error);
        if (time >= 70.0) return_error = std::max(return_error, error);
        if (turn_bank_deg != 0.0 && time >= 45.0 && time < 60.0) {
            turn_error = std::max(turn_error, error);
            bank_error_deg = std::max(bank_error_deg,
                std::abs((bank_command - state.roll_rad) / sim::radians_per_degree));
        }
        if (time >= 20.0) speed_error = std::max(speed_error,
            static_cast<double>(std::abs(initial.airspeed_m_s - state.airspeed_m_s)));
        aircraft.step(output.controls);
    }
    std::cout << "Altitude step " << step_m << " m, bank " << turn_bank_deg
              << " deg: hold " << hold_error << " m, return " << return_error
              << " m, turn " << turn_error << " m, speed " << speed_error
              << " m/s, limited " << limited_samples << ", saturated " << saturated_samples << '\n';
    check(saw_positive_climb && saw_negative_climb, "Climb-rate sign or response incorrect");
    check(hold_error < 0.25, "Late altitude hold error exceeded 0.25 m");
    check(return_error < 0.25, "Late altitude return error exceeded 0.25 m");
    check(turn_error < 0.25, "Banked altitude error exceeded 0.25 m");
    check(bank_error_deg < 0.5, "Bank tracking error exceeded 0.5 deg");
    check(speed_error < 0.35, "Airspeed drift exceeded 0.35 m/s");
    check(limited_samples < 100, "Altitude pitch limiter remained active too long");
    check(saturated_samples < 20, "Control saturation remained active too long");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        run_case(argv[1], 5.0, 0.0);
        run_case(argv[1], -5.0, 0.0);
        run_case(argv[1], 5.0, 5.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

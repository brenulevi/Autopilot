#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void run(const char* root, const std::filesystem::path& logs, bool roll,
         float direction, const ap_config_t& config)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    ap_runtime_t other_axis{};
    ap_roll_rate_runtime_t roll_runtime{};
    ap_pitch_rate_runtime_t pitch_runtime{};
    const float target = direction * (roll ? 0.08f : 0.03f);
    const std::string name = std::string(roll ? "roll" : "pitch") + (direction > 0 ? "_positive" : "_negative");
    std::ofstream csv(logs / (name + ".csv"));
    check(static_cast<bool>(csv), "Cannot open rate-loop CSV");
    csv << "time_s,target_rad_s,measured_rad_s,error_rad_s,command_norm,integral_norm,saturated\n";
    csv << std::setprecision(10);
    double hold_error = 0.0, return_error = 0.0;
    for (int tick = 0; tick < 900; ++tick) {
        const double time = tick * 0.01;
        const auto state = aircraft.state();
        const float reference = time >= 1.0 && time < 5.0 ? target : 0.0f;
        /* Stabilize the other attitude axis; the tested axis receives a direct
         * body-rate target and bypasses its outer attitude controller entirely. */
        ap_input_t input{state, trim, 0.01f,
                         roll ? AP_MODE_PITCH_HOLD : AP_MODE_ROLL_HOLD,
                         initial.roll_rad, initial.pitch_rad};
        ap_output_t controls{};
        check(ap_step(&config, &input, &other_axis, &controls), "Other-axis attitude control failed");
        float command, integral, error, measured;
        bool saturated;
        if (roll) {
            const ap_roll_rate_input_t rate_input{state.p_rad_s, reference, trim.aileron, 0.01f};
            ap_roll_rate_output_t output{};
            check(ap_roll_rate_compute(&config.roll.rate, &rate_input, &roll_runtime, &output),
                  "Roll rate controller rejected input");
            controls.controls.aileron = command = output.aileron_norm;
            integral = roll_runtime.integral_norm;
            error = output.error_rad_s;
            measured = state.p_rad_s;
            saturated = output.saturated;
        } else {
            const ap_pitch_rate_input_t rate_input{state.q_rad_s, reference, trim.elevator, 0.01f};
            ap_pitch_rate_output_t output{};
            check(ap_pitch_rate_compute(&config.pitch.rate, &rate_input, &pitch_runtime, &output),
                  "Pitch rate controller rejected input");
            controls.controls.elevator = command = output.elevator_norm;
            integral = pitch_runtime.integral_norm;
            error = output.error_rad_s;
            measured = state.q_rad_s;
            saturated = output.saturated;
        }
        csv << time << ',' << reference << ',' << measured << ',' << error << ','
            << command << ',' << integral << ',' << saturated << '\n';
        check(!saturated, "Small rate step saturated the actuator");
        if (time >= 4.0 && time < 5.0) hold_error = std::max(hold_error, std::abs(static_cast<double>(error)));
        if (time >= 8.0) return_error = std::max(return_error, std::abs(static_cast<double>(error)));
        aircraft.step(controls.controls);
    }
    std::cout << name << ": target " << target << " rad/s; settled error <= "
              << hold_error << "; return error <= " << return_error << " rad/s\n";
    check(hold_error < (roll ? 0.015 : 0.008), "Body-rate tracking exceeds settled-error bound");
    check(return_error < (roll ? 0.015 : 0.008), "Body-rate return exceeds error bound");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 3 || argc == 7, "Expected data root, log directory, optionally roll Kp Ki and pitch Kp Ki");
        auto config = sim::c172x_config;
        if (argc == 7) {
            config.roll.rate.kp = std::stof(argv[3]); config.roll.rate.ki = std::stof(argv[4]);
            config.pitch.rate.kp = std::stof(argv[5]); config.pitch.rate.ki = std::stof(argv[6]);
        }
        check(ap_control_config_validate(&config), "Invalid tuning configuration");
        const std::filesystem::path logs = argv[2];
        std::filesystem::create_directories(logs);
        bool passed = true;
        for (const bool roll : {true, false}) {
            for (const float direction : {1.0f, -1.0f}) {
                try { run(argv[1], logs, roll, direction, config); }
                catch (const std::exception& error) {
                    std::cerr << error.what() << '\n';
                    passed = false;
                }
            }
        }
        check(passed, "At least one rate-loop case failed; inspect the CSVs");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

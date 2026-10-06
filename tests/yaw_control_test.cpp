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
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
std::ofstream log_file(const std::filesystem::path& logs, const std::string& name)
{
    std::ofstream file(logs / (name + ".csv"));
    check(static_cast<bool>(file), "Cannot open yaw CSV");
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << std::setprecision(10);
    return file;
}

void probe(const char* root, const std::filesystem::path& logs, float direction, ap_config_t config)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    config.yaw.enabled = false;
    ap_runtime_t runtime{};
    auto csv = log_file(logs, direction > 0 ? "rudder_positive" : "rudder_negative");
    csv << "time_s,rudder_norm,r_rad_s,sideslip_rad,lateral_specific_force_m_s2\n";
    double response = 0;
    for (int k = 0; k < 300; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ATTITUDE_HOLD, initial.roll_rad, initial.pitch_rad};
        ap_output_t output{};
        check(ap_step(&config, &input, &runtime, &output), "Probe attitude control failed");
        if (time >= 1.0 && time < 1.5) output.controls.rudder += direction * 0.03f;
        if (k == 150) response = state.r_rad_s - initial.r_rad_s;
        csv << time << ',' << output.controls.rudder << ',' << state.r_rad_s << ','
            << aircraft.sideslip_rad() << ',' << aircraft.lateral_specific_force_m_s2() << '\n';
        aircraft.step(output.controls);
    }
    std::cout << "rudder pulse " << direction * 0.03f << ": r response " << response << " rad/s\n";
    check(response * direction * config.yaw.rate.rudder_sign > 0.001, "C172X rudder sign does not match configured sign");
}

void rate_step(const char* root, const std::filesystem::path& logs, float direction, ap_config_t config)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    const auto yaw_config = config.yaw.rate;
    config.yaw.enabled = false; // Both attitude loops stabilize; yaw outer reference is bypassed.
    ap_runtime_t attitude_runtime{};
    ap_yaw_rate_runtime_t yaw_runtime{};
    auto csv = log_file(logs, direction > 0 ? "yaw_rate_positive" : "yaw_rate_negative");
    csv << "time_s,target_rad_s,measured_rad_s,error_rad_s,rudder_norm,integral_norm,sideslip_rad,lateral_specific_force_m_s2,saturated\n";
    double hold = 0, returning = 0;
    for (int k = 0; k < 900; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const float target = time >= 1.0 && time < 5.0 ? direction * 0.02f : 0.0f;
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ATTITUDE_HOLD, initial.roll_rad, initial.pitch_rad};
        ap_output_t output{};
        check(ap_step(&config, &input, &attitude_runtime, &output), "Rate test attitude control failed");
        const ap_yaw_rate_input_t yi{state.r_rad_s, target, trim.rudder, 0.01f};
        ap_yaw_rate_output_t yo{};
        check(ap_yaw_rate_compute(&yaw_config, &yi, &yaw_runtime, &yo), "Yaw rate input rejected");
        check(!yo.saturated, "Small yaw-rate step saturated rudder");
        output.controls.rudder = yo.rudder_norm;
        csv << time << ',' << target << ',' << state.r_rad_s << ',' << yo.error_rad_s << ','
            << yo.rudder_norm << ',' << yaw_runtime.integral_norm << ',' << aircraft.sideslip_rad()
            << ',' << aircraft.lateral_specific_force_m_s2() << ',' << yo.saturated << '\n';
        if (time >= 4.0 && time < 5.0) hold = std::max(hold, std::abs(static_cast<double>(yo.error_rad_s)));
        if (time >= 8.0) returning = std::max(returning, std::abs(static_cast<double>(yo.error_rad_s)));
        aircraft.step(output.controls);
    }
    std::cout << "yaw rate " << direction * 0.02 << ": tracking " << hold << "; return " << returning << " rad/s\n";
    check(hold < 0.006 && returning < 0.006, "Yaw rate tracking/return error exceeds 0.006 rad/s");
}

struct Metrics { double beta = 0, lateral = 0, rate = 0, bank = 0, pitch = 0; };
Metrics turn(const char* root, const std::filesystem::path& logs, float direction,
             bool enabled, ap_config_t config)
{
    sim::JsbsimAdapter aircraft(root, 0.01);
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    config.yaw.enabled = enabled;
    ap_runtime_t runtime{};
    auto csv = log_file(logs, std::string("turn_") + (direction > 0 ? "positive_" : "negative_") + (enabled ? "on" : "off"));
    csv << "time_s,bank_target_rad,roll_rad,pitch_rad,r_target_rad_s,r_rad_s,r_error_rad_s,sideslip_rad,lateral_specific_force_m_s2,rudder_norm,integral_norm,yaw_active,q_feedforward_rad_s,rudder_saturated\n";
    Metrics m;
    unsigned count = 0;
    for (int k = 0; k < 3000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const double fraction = time < 2.0 ? 0.0 : time < 4.0 ? (time - 2.0) / 2.0 :
                                time < 20.0 ? 1.0 : time < 22.0 ? (22.0 - time) / 2.0 : 0.0;
        const float bank = initial.roll_rad + static_cast<float>(fraction *
            (direction * 15.0 * sim::radians_per_degree - initial.roll_rad));
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ATTITUDE_HOLD, bank, initial.pitch_rad};
        ap_output_t output{};
        check(ap_step(&config, &input, &runtime, &output), "Turn control input rejected");
        check(!output.rudder_saturated && !output.aileron_saturated && !output.elevator_saturated, "15-degree turn saturated a surface");
        const ap_turn_coordination_input_t ti{state.roll_rad, state.pitch_rad, state.airspeed_m_s};
        ap_turn_coordination_output_t reference{};
        check(ap_turn_coordination_compute(&config.yaw.coordination, &ti, &reference), "Turn diagnostic rejected input");
        const double error = reference.yaw_rate_command_rad_s - state.r_rad_s;
        const double beta = aircraft.sideslip_rad(), lateral = aircraft.lateral_specific_force_m_s2();
        csv << time << ',' << bank << ',' << state.roll_rad << ',' << state.pitch_rad << ','
            << reference.yaw_rate_command_rad_s << ',' << state.r_rad_s << ',' << error << ','
            << beta << ',' << lateral << ',' << output.controls.rudder << ',' << runtime.yaw.integral_norm
            << ',' << output.yaw_control_active << ',' << output.pitch_coordination_ff_rad_s << ',' << output.rudder_saturated << '\n';
        if (time >= 12.0 && time < 20.0) {
            ++count; m.beta += beta * beta; m.lateral += lateral * lateral; m.rate += error * error;
            m.bank = std::max(m.bank, std::abs(static_cast<double>(bank - state.roll_rad)) / sim::radians_per_degree);
            m.pitch = std::max(m.pitch, std::abs(static_cast<double>(initial.pitch_rad - state.pitch_rad)) / sim::radians_per_degree);
        }
        aircraft.step(output.controls);
    }
    m.beta = std::sqrt(m.beta / count); m.lateral = std::sqrt(m.lateral / count); m.rate = std::sqrt(m.rate / count);
    std::cout << direction * 15 << " deg yaw " << (enabled ? "on" : "off") << ": beta RMS "
              << m.beta / sim::radians_per_degree << " deg; lateral RMS " << m.lateral
              << " m/s^2; rate error RMS " << m.rate << " rad/s; bank/pitch max " << m.bank << '/' << m.pitch << " deg\n";
    check(m.bank < 0.6 && m.pitch < 0.8, "Bank/pitch turn tracking exceeds bounds");
    return m;
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 3 || argc == 5, "Expected root, logs, optionally yaw Kp Ki");
        auto config = sim::c172x_config;
        if (argc == 5) { config.yaw.rate.kp = std::stof(argv[3]); config.yaw.rate.ki = std::stof(argv[4]); }
        check(ap_control_config_validate(&config), "Invalid yaw tuning configuration");
        const std::filesystem::path logs = argv[2];
        std::filesystem::create_directories(logs);
        probe(argv[1], logs, 1.0f, config); probe(argv[1], logs, -1.0f, config);
        bool passed = true;
        for (const float direction : {1.0f, -1.0f}) {
            try { rate_step(argv[1], logs, direction, config); }
            catch (const std::exception& e) { std::cerr << e.what() << '\n'; passed = false; }
            try {
                const auto baseline = turn(argv[1], logs, direction, false, config);
                const auto active = turn(argv[1], logs, direction, true, config);
                check(active.rate < 0.005, "Coordinated yaw rate error exceeds bound");
                check(active.beta < 0.8 * baseline.beta, "Coordination did not reduce sideslip RMS by 20%");
                check(active.lateral < 0.8 * baseline.lateral, "Coordination did not reduce lateral force RMS by 20%");
            } catch (const std::exception& e) { std::cerr << e.what() << '\n'; passed = false; }
        }
        check(passed, "At least one yaw case failed; inspect CSVs");
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

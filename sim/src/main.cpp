#include "autopilot/autopilot.h"
#include "sim/c172x_config.hpp"
#include "sim/jsbsim_adapter.hpp"
#include "sim_config.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr double dt_s = 0.01;
constexpr double radians_per_degree = 0.017453292519943295;
constexpr double metres_per_second_per_knot = 0.514444;

double parse_number(const char* text)
{
    const std::string value(text);
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(result))
        throw std::invalid_argument("Expected a finite number: " + value);
    return result;
}
}

int main(int argc, char** argv)
{
    try {
        std::string mode;
        double bank_deg = 5.0;
        double pitch_offset_deg = 2.0;
        double airspeed_offset_kts = 5.0;
        double duration_s = 20.0;
        std::filesystem::path output;
        bool bank_set = false;
        bool pitch_set = false;
        bool airspeed_set = false;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--help") {
                std::cout << "Usage: attitude_hold_sim --mode roll|pitch|airspeed "
                             "[--bank-deg DEGREES] [--pitch-offset-deg DEGREES] "
                             "[--airspeed-offset-kts KNOTS] [--duration SECONDS] [--output CSV]\n"
                             "Roll uses an absolute bank target; pitch and airspeed use offsets from trim. "
                             "The command starts at 2 s.\n";
                return 0;
            }
            if (option != "--mode" && option != "--bank-deg" &&
                option != "--pitch-offset-deg" && option != "--airspeed-offset-kts" &&
                option != "--duration" &&
                option != "--output")
                throw std::invalid_argument("Unknown option: " + option);
            if (i + 1 >= argc) throw std::invalid_argument("Missing value after " + option);
            const std::string value = argv[++i];
            if (option == "--mode") mode = value;
            else if (option == "--bank-deg") {
                bank_deg = parse_number(value.c_str());
                bank_set = true;
            }
            else if (option == "--pitch-offset-deg") {
                pitch_offset_deg = parse_number(value.c_str());
                pitch_set = true;
            }
            else if (option == "--airspeed-offset-kts") {
                airspeed_offset_kts = parse_number(value.c_str());
                airspeed_set = true;
            }
            else if (option == "--duration") duration_s = parse_number(value.c_str());
            else if (option == "--output") output = value;
        }
        if (mode != "roll" && mode != "pitch" && mode != "airspeed")
            throw std::invalid_argument("Choose --mode roll, pitch, or airspeed");
        if ((mode != "roll" && bank_set) || (mode != "pitch" && pitch_set) ||
            (mode != "airspeed" && airspeed_set))
            throw std::invalid_argument("Target option must match the selected mode");
        if (std::abs(bank_deg) > 20.0 || std::abs(pitch_offset_deg) > 5.0 ||
            std::abs(airspeed_offset_kts) > 15.0 ||
            duration_s < 2.01 || duration_s > 300.0)
            throw std::invalid_argument("Bank must be within +/-20 deg, pitch offset within +/-5 deg, "
                                        "airspeed offset within +/-15 kt, and duration in [2.01, 300] s");
        if (output.empty()) output = mode == "roll" ?
            "logs/c172x_roll_hold.csv" : mode == "pitch" ?
            "logs/c172x_pitch_hold.csv" : "logs/c172x_airspeed_hold.csv";

        sim::JsbsimAdapter aircraft(SIM_JSBSIM_DATA_ROOT, dt_s);
        const auto trim = aircraft.trim_controls();
        const auto initial_attitude = aircraft.attitude_state();
        const float initial_airspeed_m_s = aircraft.airspeed_m_s();
        const auto config = sim::c172x_config(trim);
        autopilot_state_t controller_state{};

        if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
        std::ofstream csv(output);
        if (!csv) throw std::runtime_error("Cannot open CSV: " + output.string());
        csv << "time_s,target_bank_deg,bank_deg,roll_rate_deg_s,aileron_command,aileron_trim,"
               "target_pitch_deg,pitch_deg,pitch_rate_deg_s,elevator_command,elevator_trim,"
               "elevator_requested_deg,elevator_actual_deg,"
               "target_airspeed_kts,airspeed_kts,throttle_command,throttle_trim\n";

        const auto steps = static_cast<unsigned long long>(duration_s / dt_s);
        for (unsigned long long k = 0; k < steps; ++k) {
            const auto state = aircraft.attitude_state();
            const auto elevator_actuator = aircraft.elevator_actuator_state();
            const double time = aircraft.time_s();
            const float target_bank = mode == "roll" && time >= 2.0 ?
                static_cast<float>(bank_deg * radians_per_degree) : initial_attitude.bank_rad;
            const float target_pitch = initial_attitude.pitch_rad +
                (mode == "pitch" && time >= 2.0 ?
                    static_cast<float>(pitch_offset_deg * radians_per_degree) : 0.0f);
            const float target_airspeed = initial_airspeed_m_s +
                (mode == "airspeed" && time >= 2.0 ?
                    static_cast<float>(airspeed_offset_kts * metres_per_second_per_knot) : 0.0f);
            autopilot_input_t input{};
            input.target_bank_rad = target_bank;
            input.target_pitch_rad = target_pitch;
            input.measured_bank_rad = state.bank_rad;
            input.measured_pitch_rad = state.pitch_rad;
            input.measured_roll_rate_rad_s = state.roll_rate_rad_s;
            input.measured_pitch_rate_rad_s = state.pitch_rate_rad_s;
            input.target_airspeed_m_s = target_airspeed;
            input.measured_airspeed_m_s = aircraft.airspeed_m_s();
            input.dt_s = static_cast<float>(dt_s);
            autopilot_output_t result{};
            if (!autopilot_step(&config, &input, &controller_state, &result))
                throw std::runtime_error("A controller rejected a simulation sample");

            auto controls = trim;
            controls.aileron = result.aileron_command;
            controls.elevator = result.elevator_command;
            controls.throttle = result.throttle_command;
            csv << time << ',' << target_bank / radians_per_degree << ','
                << state.bank_rad / radians_per_degree << ','
                << state.roll_rate_rad_s / radians_per_degree << ','
                << controls.aileron << ',' << trim.aileron << ','
                << target_pitch / radians_per_degree << ','
                << state.pitch_rad / radians_per_degree << ','
                << state.pitch_rate_rad_s / radians_per_degree << ','
                << controls.elevator << ',' << trim.elevator << ','
                << elevator_actuator.requested_rad / radians_per_degree << ','
                << elevator_actuator.actual_rad / radians_per_degree << ','
                << target_airspeed / metres_per_second_per_knot << ','
                << input.measured_airspeed_m_s / metres_per_second_per_knot << ','
                << controls.throttle << ',' << trim.throttle << '\n';
            aircraft.step(controls);
        }
        const auto final = aircraft.attitude_state();
        if (mode == "roll")
            std::cout << "C172X final bank: " << final.bank_rad / radians_per_degree
                      << " deg; target: " << bank_deg << " deg; ";
        else if (mode == "pitch")
            std::cout << "C172X final pitch: " << final.pitch_rad / radians_per_degree
                      << " deg; target: "
                      << initial_attitude.pitch_rad / radians_per_degree + pitch_offset_deg
                      << " deg; ";
        else
            std::cout << "C172X final airspeed: " << aircraft.airspeed_m_s() / metres_per_second_per_knot
                      << " kt; target: " << initial_airspeed_m_s / metres_per_second_per_knot +
                      airspeed_offset_kts << " kt; ";
        std::cout << "CSV: " << std::filesystem::absolute(output).string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "attitude_hold_sim: " << error.what() << '\n';
        return 1;
    }
}

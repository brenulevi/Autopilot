#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"
#include "sim_config.h"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
constexpr double dt_s = 0.01;
constexpr double pulse_start_s = 2.0;
constexpr double pulse_end_s = 2.5;

double number(const std::string& text)
{
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument("Invalid finite number: " + text);
    }
    return value;
}
}

int main(int argc, char** argv)
{
    try {
        std::string data_root = default_jsbsim_root;
        std::filesystem::path output;
        double duration_s = 0.0;
        bool duration_set = false;
        double pulse = 0.05;
        double elevator_pulse = 0.0;
        double throttle_pulse = 0.0;
        double bank_deg = 5.0;
        double pitch_deg = 2.0;
        double airspeed_kts = 100.0;
        double speed_step_m_s = 1.0;
        double altitude_step_m = 5.0;
        double turn_bank_deg = 0.0;
        std::string mode = "manual";
        auto config = sim::c172x_config;
        bool roll_options_set = false;
        bool pitch_options_set = false;
        bool speed_options_set = false;
        bool speed_step_set = false;
        bool altitude_options_set = false;
        bool aileron_pulse_set = false;
        bool elevator_pulse_set = false;
        bool throttle_pulse_set = false;
        bool flightgear = false;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--help") {
                std::cout << "Usage: autopilot_sim [--jsbsim-root PATH] [--output FILE]\n"
                             "                     [--duration SECONDS] [--aileron-pulse VALUE|--elevator-pulse VALUE|--throttle-pulse VALUE]\n"
                             "                     [--mode manual|roll-hold|pitch-hold|attitude-hold|attitude-airspeed-hold|altitude-hold]\n"
                             "                     [--bank-deg DEGREES] [--pitch-deg DEGREES]\n"
                             "                     [--airspeed-kts KNOTS] (80 to 120)\n"
                             "                     [--roll-kp GAIN] [--roll-kd GAIN]\n"
                             "                     [--pitch-kp GAIN] [--pitch-kd GAIN] [--flightgear]\n"
                             "                     [--speed-step-m-s VALUE] [--speed-kp GAIN] [--speed-ki GAIN]\n"
                             "                     [--altitude-step-m VALUE] [--turn-bank-deg VALUE]\n"
                             "                     [--altitude-kh GAIN] [--altitude-kv GAIN]\n"
                             "C172X, 100 Hz, 3000 ft MSL, 100 kt calibrated airspeed.\n"
                             "Manual (default): pulse +0.05 during [2, 2.5) s, duration 10 s.\n"
                             "Use --aileron-pulse 0 for a trim-only baseline.\n"
                             "Use --elevator-pulse for a manual elevator pulse instead.\n"
                             "Use --throttle-pulse in manual or attitude-hold mode\n"
                             "for a throttle step during [2, 7) s.\n"
                             "Roll hold: initial bank until 2 s, bank command until 10 s,\n"
                             "then wings level. Defaults: +5 degrees, duration 20 s.\n"
                             "Pitch hold: trim attitude until 2 s, trim +2 degrees until\n"
                             "10 s, then return to trim. Duration 20 s by default.\n"
                             "Attitude hold: bank and pitch commands together.\n"
                             "Attitude-airspeed hold: trim attitude; true speed +1 m/s\n"
                             "during [2, 30) s, then return. Duration 60 s.\n"
                             "Altitude hold: +/- altitude step during [2, 30) s;\n"
                             "optional banked segment [40, 60) s. Duration 90 s.\n"
                             "Bank commands are limited to +/-20 degrees; aileron to +/-0.5.\n"
                             "Gains use radians, not degrees. Existing CSV files are overwritten.\n"
                             "--flightgear sends native FDM to localhost UDP 5600 at 50 Hz\n"
                             "and paces the simulation in real time. Start FlightGear first.\n";
                return 0;
            }
            if (option == "--flightgear") {
                flightgear = true;
                continue;
            }
            if (i + 1 >= argc) {
                throw std::invalid_argument("Missing value for " + option);
            }
            const std::string value = argv[++i];
            if (option == "--jsbsim-root") data_root = value;
            else if (option == "--output") output = value;
            else if (option == "--duration") { duration_s = number(value); duration_set = true; }
            else if (option == "--airspeed-kts") airspeed_kts = number(value);
            else if (option == "--speed-step-m-s") { speed_step_m_s = number(value); speed_step_set = true; }
            else if (option == "--altitude-step-m") {
                altitude_step_m = number(value); altitude_options_set = true;
            }
            else if (option == "--turn-bank-deg") {
                turn_bank_deg = number(value); altitude_options_set = true;
            }
            else if (option == "--aileron-pulse") {
                pulse = number(value); aileron_pulse_set = true;
            }
            else if (option == "--elevator-pulse") {
                elevator_pulse = number(value);
                elevator_pulse_set = true; pulse = 0.0;
            }
            else if (option == "--throttle-pulse") {
                throttle_pulse = number(value);
                throttle_pulse_set = true; pulse = 0.0;
            }
            else if (option == "--mode") mode = value;
            else if (option == "--bank-deg") { bank_deg = number(value); roll_options_set = true; }
            else if (option == "--pitch-deg") { pitch_deg = number(value); pitch_options_set = true; }
            else if (option == "--roll-kp") {
                config.roll_angle_gain = static_cast<float>(number(value)); roll_options_set = true;
            }
            else if (option == "--roll-kd") {
                config.roll_rate_gain = static_cast<float>(number(value)); roll_options_set = true;
            }
            else if (option == "--pitch-kp") {
                config.pitch_angle_gain = static_cast<float>(number(value)); pitch_options_set = true;
            }
            else if (option == "--pitch-kd") {
                config.pitch_rate_gain = static_cast<float>(number(value)); pitch_options_set = true;
            }
            else if (option == "--speed-kp") {
                config.airspeed_kp = static_cast<float>(number(value)); speed_options_set = true;
            }
            else if (option == "--speed-ki") {
                config.airspeed_ki = static_cast<float>(number(value)); speed_options_set = true;
            }
            else if (option == "--altitude-kh") {
                config.altitude_gain = static_cast<float>(number(value)); altitude_options_set = true;
            }
            else if (option == "--altitude-kv") {
                config.climb_rate_gain = static_cast<float>(number(value)); altitude_options_set = true;
            }
            else throw std::invalid_argument("Unknown option: " + option);
        }
        if (mode != "manual" && mode != "roll-hold" && mode != "pitch-hold" &&
            mode != "attitude-hold" && mode != "attitude-airspeed-hold" && mode != "altitude-hold") {
            throw std::invalid_argument("Unknown control mode");
        }
        const bool roll_hold = mode == "roll-hold";
        const bool pitch_hold = mode == "pitch-hold";
        const bool attitude_hold = mode == "attitude-hold";
        const bool speed_hold = mode == "attitude-airspeed-hold";
        const bool altitude_hold = mode == "altitude-hold";
        const bool speed_active = speed_hold || altitude_hold;
        const bool roll_active = roll_hold || attitude_hold || speed_active;
        const bool pitch_active = pitch_hold || attitude_hold || speed_active;
        if ((mode != "manual" && (aileron_pulse_set || elevator_pulse_set)) ||
            (mode != "manual" && mode != "attitude-hold" && throttle_pulse_set) ||
            (mode != "roll-hold" && mode != "attitude-hold" && !altitude_hold && roll_options_set) ||
            (mode != "pitch-hold" && mode != "attitude-hold" && !altitude_hold && pitch_options_set) ||
            (!speed_active && speed_options_set) ||
            (!speed_hold && speed_step_set) ||
            (!altitude_hold && altitude_options_set) ||
            (static_cast<int>(aileron_pulse_set) + static_cast<int>(elevator_pulse_set) +
             static_cast<int>(throttle_pulse_set) > 1)) {
            throw std::invalid_argument("Pulse and attitude options must match their modes; choose one pulse axis");
        }
        if (!duration_set) duration_s = mode == "manual" ? 10.0 :
            (altitude_hold ? 90.0 : (speed_hold ? 60.0 : 20.0));
        if (output.empty()) output = altitude_hold ? "logs/c172x_altitude_hold.csv" :
            (speed_hold ? "logs/c172x_airspeed_hold.csv" :
            (attitude_hold ? "logs/c172x_attitude_hold.csv" :
            (roll_hold ? "logs/c172x_roll_hold.csv" :
            (pitch_hold ? "logs/c172x_pitch_hold.csv" : "logs/c172x_pulse.csv"))));
        if (duration_s < dt_s || duration_s > 3600.0 || std::abs(pulse) > 1.0 ||
            std::abs(elevator_pulse) > 1.0 || std::abs(throttle_pulse) > 1.0) {
            throw std::invalid_argument("Duration must be in [0.01, 3600] s and pulses in [-1, 1]");
        }
        if (std::abs(bank_deg) > 90.0 || !std::isfinite(config.roll_angle_gain) ||
            config.roll_angle_gain <= 0.0f || !std::isfinite(config.roll_rate_gain) ||
            config.roll_rate_gain < 0.0f) {
            throw std::invalid_argument("Bank input must be within +/-90 deg; finite Kp > 0 and Kd >= 0 required");
        }
        if (std::abs(pitch_deg) > 90.0 || !std::isfinite(config.pitch_angle_gain) ||
            config.pitch_angle_gain <= 0.0f || !std::isfinite(config.pitch_rate_gain) ||
            config.pitch_rate_gain < 0.0f) {
            throw std::invalid_argument("Pitch input must be within +/-90 deg; finite Kp > 0 and Kd >= 0 required");
        }
        if (airspeed_kts < 80.0 || airspeed_kts > 120.0) {
            throw std::invalid_argument("Initial calibrated airspeed must be in [80, 120] kt");
        }
        if (std::abs(speed_step_m_s) > 5.0 || !std::isfinite(config.airspeed_kp) ||
            config.airspeed_kp < 0.0f || !std::isfinite(config.airspeed_ki) ||
            config.airspeed_ki <= 0.0f) {
            throw std::invalid_argument("Speed step must be within +/-5 m/s; finite Kp >= 0 and Ki > 0 required");
        }
        if (std::abs(altitude_step_m) > 20.0 || std::abs(turn_bank_deg) > 15.0 ||
            !std::isfinite(config.altitude_gain) || config.altitude_gain <= 0.0f ||
            !std::isfinite(config.climb_rate_gain) || config.climb_rate_gain < 0.0f) {
            throw std::invalid_argument("Altitude step must be within +/-20 m, turn bank within +/-15 deg, and altitude gains valid");
        }

        sim::JsbsimAdapter aircraft(data_root, dt_s,
                                   flightgear ? flightgear_output_file : "", airspeed_kts);
        const auto trim = aircraft.trim_controls();
        const float initial_bank = aircraft.state().roll_rad;
        const float initial_pitch = aircraft.state().pitch_rad;
        const float initial_speed = aircraft.state().airspeed_m_s;
        const float initial_altitude = aircraft.state().altitude_m;
        ap_runtime_t runtime{};
        if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
        std::ofstream csv(output);
        if (!csv) throw std::runtime_error("Cannot open CSV: " + output.string());
        csv.exceptions(std::ios::badbit | std::ios::failbit);
        csv << "time_s,roll_rad,pitch_rad,yaw_rad,p_rad_s,q_rad_s,r_rad_s,airspeed_m_s,altitude_m,"
               "aileron_cmd_norm,elevator_cmd_norm,rudder_cmd_norm,throttle_cmd_norm,"
               "control_mode,roll_request_rad,roll_command_rad,roll_error_rad,"
               "aileron_saturated,bank_command_limited,roll_kp,roll_kd,aileron_limit_norm,"
               "elevator_pos_rad,upstream_elevator_cmd_norm,pitch_trim_cmd_norm,"
               "pitch_request_rad,pitch_command_rad,pitch_error_rad,"
               "elevator_saturated,pitch_command_limited,pitch_kp,pitch_kd,elevator_limit_norm,"
               "airspeed_request_m_s,airspeed_command_m_s,airspeed_error_m_s,"
               "throttle_saturated,speed_kp,speed_ki,speed_integral_norm,"
               "throttle_min_norm,throttle_max_norm,climb_rate_m_s,"
               "altitude_request_m,altitude_command_m,altitude_error_m,"
               "altitude_pitch_limited,altitude_kh,altitude_kv,max_pitch_offset_rad\n";
        csv << std::setprecision(10);

        const auto steps = static_cast<unsigned long long>(std::ceil(duration_s / dt_s));
        if (flightgear) {
            std::cout << "Streaming to FlightGear on localhost UDP 5600 at 50 Hz for "
                      << duration_s << " s.\n" << std::flush;
        }
        const auto wall_start = std::chrono::steady_clock::now();
        for (unsigned long long k = 0; k < steps; ++k) {
            const double time = static_cast<double>(k) * dt_s;
            const auto state = aircraft.state();
            auto requested = trim;
            if (mode == "manual" && time >= pulse_start_s && time < pulse_end_s) {
                requested.aileron += static_cast<float>(pulse);
                requested.elevator += static_cast<float>(elevator_pulse);
            }
            if (time >= 2.0 && time < 7.0) {
                requested.throttle += static_cast<float>(throttle_pulse);
            }
            const float bank_command = altitude_hold ?
                (time >= 40.0 && time < 60.0 ?
                 static_cast<float>(turn_bank_deg * sim::radians_per_degree) : 0.0f) :
                speed_hold ? initial_bank : time < 2.0 ? initial_bank :
                (time < 10.0 ? static_cast<float>(bank_deg * sim::radians_per_degree) : 0.0f);
            const float pitch_command = speed_active || time < 2.0 || time >= 10.0 ? initial_pitch :
                initial_pitch + static_cast<float>(pitch_deg * sim::radians_per_degree);
            const float speed_command = time < 2.0 || time >= 30.0 ? initial_speed :
                initial_speed + static_cast<float>(speed_step_m_s);
            const float altitude_command = time < 2.0 || time >= 30.0 ? initial_altitude :
                initial_altitude + static_cast<float>(altitude_step_m);
            const ap_input_t input{state, requested, static_cast<float>(dt_s),
                                  altitude_hold ? AP_MODE_ALTITUDE_AIRSPEED_HOLD :
                                  (speed_hold ? AP_MODE_ATTITUDE_AIRSPEED_HOLD :
                                  (attitude_hold ? AP_MODE_ATTITUDE_HOLD :
                                  (roll_hold ? AP_MODE_ROLL_HOLD :
                                  (pitch_hold ? AP_MODE_PITCH_HOLD : AP_MODE_MANUAL)))),
                                  bank_command, pitch_command, altitude_hold ? initial_speed : speed_command,
                                  altitude_command};
            ap_output_t result{};
            const bool accepted = speed_active ? ap_step_with_runtime(&config, &input, &runtime, &result) :
                ap_step(&config, &input, &result);
            if (!accepted) {
                throw std::runtime_error("C autopilot rejected invalid state, command, configuration, or timestep");
            }
            const auto& controls = result.controls;
            // Each row is state at t plus the command applied over [t, t + dt).
            csv << aircraft.time_s() << ',' << state.roll_rad << ',' << state.pitch_rad << ','
                << state.yaw_rad << ',' << state.p_rad_s << ',' << state.q_rad_s << ','
                << state.r_rad_s << ',' << state.airspeed_m_s << ',' << state.altitude_m << ','
                << controls.aileron << ',' << controls.elevator << ',' << controls.rudder << ','
                << controls.throttle << ',' << mode << ',';
            if (roll_active) {
                csv << bank_command << ',' << result.bank_command_rad << ','
                    << (result.bank_command_rad - state.roll_rad);
            } else {
                csv << ",,"; // No bank reference or tracking error in this mode.
            }
            csv << ',' << result.aileron_saturated << ',' << result.bank_command_limited
                << ',' << (roll_active ? config.roll_angle_gain : 0.0f)
                << ',' << (roll_active ? config.roll_rate_gain : 0.0f)
                << ',' << (roll_active ? config.max_aileron : 1.0f)
                << ',' << aircraft.elevator_position_rad()
                << ',' << aircraft.upstream_elevator_command()
                << ',' << aircraft.pitch_trim_command() << ',';
            if (pitch_active) {
                csv << (altitude_hold ? result.pitch_command_rad : pitch_command)
                    << ',' << result.pitch_command_rad << ','
                    << (result.pitch_command_rad - state.pitch_rad);
            } else {
                csv << ",,";
            }
            csv << ',' << result.elevator_saturated << ',' << result.pitch_command_limited
                << ',' << (pitch_active ? config.pitch_angle_gain : 0.0f)
                << ',' << (pitch_active ? config.pitch_rate_gain : 0.0f)
                << ',' << (pitch_active ? config.max_elevator : 1.0f) << ',';
            if (speed_active) {
                csv << (altitude_hold ? initial_speed : speed_command)
                    << ',' << result.airspeed_command_m_s << ','
                    << result.airspeed_error_m_s;
            } else {
                csv << ",,";
            }
            csv << ',' << result.throttle_saturated
                << ',' << (speed_active ? config.airspeed_kp : 0.0f)
                << ',' << (speed_active ? config.airspeed_ki : 0.0f)
                << ',' << (speed_active ? runtime.airspeed_integral_norm : 0.0f)
                << ',' << (speed_active ? config.min_throttle : 0.0f)
                << ',' << (speed_active ? config.max_throttle : 1.0f)
                << ',' << state.climb_rate_m_s << ',';
            if (altitude_hold) {
                csv << altitude_command << ',' << result.altitude_command_m << ','
                    << result.altitude_error_m;
            } else {
                csv << ",,";
            }
            csv << ',' << result.altitude_pitch_limited
                << ',' << (altitude_hold ? config.altitude_gain : 0.0f)
                << ',' << (altitude_hold ? config.climb_rate_gain : 0.0f)
                << ',' << (altitude_hold ? config.max_pitch_offset_rad : 0.0f) << '\n';
            aircraft.step(controls);
            if (flightgear) {
                const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(static_cast<double>(k + 1) * dt_s));
                std::this_thread::sleep_until(target);
            }
        }
        csv.close();
        std::cout << "Simulated " << aircraft.time_s() << " s. CSV: "
                  << std::filesystem::absolute(output).string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "autopilot_sim: " << error.what() << '\n';
        return 1;
    }
}

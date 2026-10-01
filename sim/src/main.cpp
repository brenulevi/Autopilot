#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"
#include "sim/config_file.hpp"
#include "sim_config.h"
#include "autopilot/mission.h"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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
        std::filesystem::path mission_path;
        std::filesystem::path config_path;
        bool config_path_set = false;
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
        double l1_period_s = 4.0;
        double wind_north_m_s = 0.0;
        double wind_east_m_s = 0.0;
        sim::InitialPosition start;
        std::string mode = "manual";
        auto config = sim::c172x_config;
        std::vector<std::pair<float ap_config_t::*, float>> config_overrides;
        bool roll_options_set = false;
        bool pitch_options_set = false;
        bool speed_options_set = false;
        bool speed_step_set = false;
        bool altitude_options_set = false;
        bool l1_period_set = false;
        bool aileron_pulse_set = false;
        bool elevator_pulse_set = false;
        bool throttle_pulse_set = false;
        bool flightgear = false;
        bool realtime = false;
        bool telemetry_stdout = false;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--help") {
                std::cout << "Usage: autopilot_sim [--jsbsim-root PATH] [--output FILE]\n"
                             "                     [--config FILE.apcf]\n"
                             "                     [--duration SECONDS] [--aileron-pulse VALUE|--elevator-pulse VALUE|--throttle-pulse VALUE]\n"
                             "                     [--mode manual|roll-hold|pitch-hold|attitude-hold|attitude-airspeed-hold|altitude-hold|mission]\n"
                             "                     [--mission FILE] [--l1-period-s SECONDS]\n"
                             "                     [--wind-north-m-s VALUE] [--wind-east-m-s VALUE]\n"
                             "                     [--start-lat-deg VALUE] [--start-lon-deg VALUE] [--start-heading-deg VALUE]\n"
                             "                     [--bank-deg DEGREES] [--pitch-deg DEGREES]\n"
                             "                     [--bank-limit-deg DEGREES] (0 < limit < 90; default 20)\n"
                             "                     [--pitch-limit-deg DEGREES] (absolute attitude; 0 < limit < 90; default 10)\n"
                             "                     [--airspeed-kts KNOTS] (positive; default 100)\n"
                             "                     [--roll-kp GAIN] [--roll-kd GAIN]\n"
                             "                     [--pitch-kp GAIN] [--pitch-kd GAIN] [--flightgear]\n"
                             "                     [--realtime] [--telemetry-stdout]\n"
                             "                     [--speed-step-m-s VALUE] [--speed-kp GAIN] [--speed-ki GAIN]\n"
                             "                     [--altitude-step-m VALUE] [--turn-bank-deg VALUE]\n"
                             "                     [--altitude-kh GAIN] [--altitude-kv GAIN]\n"
                             "C172X, 100 Hz, 3000 ft MSL; default 100 kt calibrated airspeed.\n"
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
                             "Mission: airborne; APM3 fly-by/fly-over waypoints (APM2 also accepted).\n"
                             "Mission entry captures the nearest route leg; earlier waypoints may be skipped.\n"
                             "Start defaults to 30 N, 0 E, heading 0 deg; heading range [0,360).\n"
                             "Wind components are positive toward north/east; total steady wind <=20 m/s.\n"
                             "Default bank limit is +/-20 degrees; default aileron limit is +/-1.0.\n"
                             "--config loads a validated binary APCF profile; explicit gain/L1 options override it.\n"
                             "Gains use radians, not degrees. Existing CSV files are overwritten.\n"
                             "--flightgear sends native FDM to localhost UDP 5600 at 50 Hz\n"
                             "and paces the simulation in real time. Start FlightGear first.\n"
                             "--realtime paces without FlightGear; --telemetry-stdout emits JSON at 10 Hz.\n";
                return 0;
            }
            if (option == "--flightgear") {
                flightgear = true;
                continue;
            }
            if (option == "--realtime") { realtime = true; continue; }
            if (option == "--telemetry-stdout") { telemetry_stdout = true; continue; }
            if (i + 1 >= argc) {
                throw std::invalid_argument("Missing value for " + option);
            }
            const std::string value = argv[++i];
            if (option == "--jsbsim-root") data_root = value;
            else if (option == "--output") output = value;
            else if (option == "--mission") mission_path = value;
            else if (option == "--config") {
                if (config_path_set || value.empty())
                    throw std::invalid_argument("Specify one nonempty --config path");
                config_path = value; config_path_set = true;
            }
            else if (option == "--l1-period-s") { l1_period_s = number(value); l1_period_set = true; }
            else if (option == "--wind-north-m-s") wind_north_m_s = number(value);
            else if (option == "--wind-east-m-s") wind_east_m_s = number(value);
            else if (option == "--start-lat-deg") start.latitude_deg = number(value);
            else if (option == "--start-lon-deg") start.longitude_deg = number(value);
            else if (option == "--start-heading-deg") start.heading_deg = number(value);
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
            else if (option == "--bank-limit-deg") {
                const double limit = number(value);
                if (limit <= 0.0 || limit >= 90.0)
                    throw std::invalid_argument("Bank limit must be strictly between 0 and 90 degrees");
                config_overrides.emplace_back(&ap_config_t::max_bank_rad,
                                               static_cast<float>(limit * sim::radians_per_degree));
                roll_options_set = true;
            }
            else if (option == "--pitch-deg") { pitch_deg = number(value); pitch_options_set = true; }
            else if (option == "--pitch-limit-deg") {
                const double limit = number(value);
                if (limit <= 0.0 || limit >= 90.0)
                    throw std::invalid_argument("Pitch limit must be strictly between 0 and 90 degrees");
                config_overrides.emplace_back(&ap_config_t::max_pitch_rad,
                                               static_cast<float>(limit * sim::radians_per_degree));
                pitch_options_set = true;
            }
            else if (option == "--roll-kp") {
                config_overrides.emplace_back(&ap_config_t::roll_angle_gain, static_cast<float>(number(value))); roll_options_set = true;
            }
            else if (option == "--roll-kd") {
                config_overrides.emplace_back(&ap_config_t::roll_rate_gain, static_cast<float>(number(value))); roll_options_set = true;
            }
            else if (option == "--pitch-kp") {
                config_overrides.emplace_back(&ap_config_t::pitch_angle_gain, static_cast<float>(number(value))); pitch_options_set = true;
            }
            else if (option == "--pitch-kd") {
                config_overrides.emplace_back(&ap_config_t::pitch_rate_gain, static_cast<float>(number(value))); pitch_options_set = true;
            }
            else if (option == "--speed-kp") {
                config_overrides.emplace_back(&ap_config_t::airspeed_kp, static_cast<float>(number(value))); speed_options_set = true;
            }
            else if (option == "--speed-ki") {
                config_overrides.emplace_back(&ap_config_t::airspeed_ki, static_cast<float>(number(value))); speed_options_set = true;
            }
            else if (option == "--altitude-kh") {
                config_overrides.emplace_back(&ap_config_t::altitude_gain, static_cast<float>(number(value))); altitude_options_set = true;
            }
            else if (option == "--altitude-kv") {
                config_overrides.emplace_back(&ap_config_t::climb_rate_gain, static_cast<float>(number(value))); altitude_options_set = true;
            }
            else throw std::invalid_argument("Unknown option: " + option);
        }
        uint32_t config_sequence = 0;
        if (config_path_set) {
            const auto loaded = sim::load_config(config_path);
            config = loaded.control;
            config_sequence = loaded.sequence;
            if (!l1_period_set) l1_period_s = loaded.l1_period_s;
        }
        for (const auto& override_value : config_overrides)
            config.*(override_value.first) = override_value.second;
        const ap_aircraft_config_t effective_config{config, static_cast<float>(l1_period_s), config_sequence};
        if (!ap_config_validate(&effective_config))
            throw std::invalid_argument("Invalid effective aircraft configuration");
        if (config_path_set)
            std::cout << "Loaded configuration: " << config_path.string() << " (sequence "
                      << config_sequence << "); explicit gain/L1 options override file values.\n";

        if (mode != "manual" && mode != "roll-hold" && mode != "pitch-hold" &&
            mode != "attitude-hold" && mode != "attitude-airspeed-hold" &&
            mode != "altitude-hold" && mode != "mission") {
            throw std::invalid_argument("Unknown control mode");
        }
        const bool roll_hold = mode == "roll-hold";
        const bool pitch_hold = mode == "pitch-hold";
        const bool attitude_hold = mode == "attitude-hold";
        const bool speed_hold = mode == "attitude-airspeed-hold";
        const bool altitude_hold = mode == "altitude-hold";
        const bool mission_mode = mode == "mission";
        const bool altitude_active = altitude_hold || mission_mode;
        const bool speed_active = speed_hold || altitude_active;
        const bool roll_active = roll_hold || attitude_hold || speed_active;
        const bool pitch_active = pitch_hold || attitude_hold || speed_active;
        if ((mode != "manual" && (aileron_pulse_set || elevator_pulse_set)) ||
            (mode != "manual" && mode != "attitude-hold" && throttle_pulse_set) ||
            (mode != "roll-hold" && mode != "attitude-hold" && !altitude_active && roll_options_set) ||
            (mode != "pitch-hold" && mode != "attitude-hold" && !altitude_active && pitch_options_set) ||
            (!speed_active && speed_options_set) ||
            (!speed_hold && speed_step_set) ||
            (!altitude_hold && altitude_options_set) ||
            (mission_mode != !mission_path.empty()) ||
            (static_cast<int>(aileron_pulse_set) + static_cast<int>(elevator_pulse_set) +
             static_cast<int>(throttle_pulse_set) > 1)) {
            throw std::invalid_argument("Pulse and attitude options must match their modes; choose one pulse axis");
        }
        if (!duration_set) duration_s = mode == "manual" ? 10.0 :
            (altitude_active ? 90.0 : (speed_hold ? 60.0 : 20.0));
        if (output.empty()) output = mission_mode ? "logs/c172x_mission.csv" :
            (altitude_hold ? "logs/c172x_altitude_hold.csv" :
            (speed_hold ? "logs/c172x_airspeed_hold.csv" :
            (attitude_hold ? "logs/c172x_attitude_hold.csv" :
            (roll_hold ? "logs/c172x_roll_hold.csv" :
            (pitch_hold ? "logs/c172x_pitch_hold.csv" : "logs/c172x_pulse.csv")))));
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
        if (airspeed_kts <= 0.0) {
            throw std::invalid_argument("Initial calibrated airspeed must be finite and positive");
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
        if (l1_period_s < 1.0 || l1_period_s > 30.0 || (!mission_mode && l1_period_set)) {
            throw std::invalid_argument("L1 period must be in [1, 30] s and requires mission mode");
        }
        if (std::hypot(wind_north_m_s, wind_east_m_s) > 20.0) {
            throw std::invalid_argument("Steady wind magnitude must be at most 20 m/s");
        }
        if (config_path_set && std::filesystem::exists(output) &&
            std::filesystem::equivalent(output, config_path))
            throw std::invalid_argument("CSV output must not overwrite the input configuration");

        ap_mission_t mission{};
        ap_mission_runtime_t mission_runtime{};
        if (mission_mode) {
            std::ifstream file(mission_path, std::ios::binary | std::ios::ate);
            if (!file) throw std::runtime_error("Cannot open compiled mission: " + mission_path.string());
            const auto length = file.tellg();
            if (length < 0 || length > 16 + 16 * static_cast<std::streamoff>(AP_MISSION_MAX_WAYPOINTS))
                throw std::runtime_error("Invalid compiled mission size");
            std::vector<uint8_t> bytes(static_cast<std::size_t>(length));
            file.seekg(0);
            file.read(reinterpret_cast<char*>(bytes.data()), length);
            if (!file || !ap_mission_decode(bytes.data(), bytes.size(), &mission))
                throw std::runtime_error("Invalid compiled mission format, CRC, or waypoint values");
        }

        sim::JsbsimAdapter aircraft(data_root, dt_s,
                                   flightgear ? flightgear_output_file : "", airspeed_kts,
                                   wind_north_m_s, wind_east_m_s, start);
        const auto steady_wind = aircraft.steady_wind_m_s();
        const auto trim = aircraft.trim_controls();
        const float initial_bank = aircraft.state().roll_rad;
        const float initial_pitch = aircraft.state().pitch_rad;
        const float initial_speed = aircraft.state().airspeed_m_s;
        const float initial_altitude = aircraft.state().altitude_m;
        if (mission_mode) {
            const auto nav = aircraft.navigation_state();
            float north_m = 0.0f, east_m = 0.0f;
            if (!nav.valid || !ap_mission_project(&mission, nav.lat_e7, nav.lon_e7,
                                                  &north_m, &east_m))
                throw std::runtime_error("Aircraft start is outside the mission's local projection bounds");
        }
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
               "altitude_pitch_limited,altitude_kh,altitude_kv,max_pitch_offset_rad,"
               "lat_deg,lon_deg,north_m,east_m,ground_north_m_s,ground_east_m_s,"
               "cross_track_m,mission_leg,wind_north_m_s,wind_east_m_s,mission_phase,mission_target_distance_m,waypoint_type\n";
        csv << std::setprecision(10);

        const auto steps = static_cast<unsigned long long>(std::ceil(duration_s / dt_s));
        if (flightgear) {
            std::cout << "Streaming to FlightGear on localhost UDP 5600 at 50 Hz for "
                      << duration_s << " s.\n" << std::flush;
        }
        const auto wall_start = std::chrono::steady_clock::now();
        bool mission_completed = false;
        for (unsigned long long k = 0; k < steps; ++k) {
            const double time = static_cast<double>(k) * dt_s;
            const auto state = aircraft.state();
            ap_nav_state_t nav{};
            ap_mission_output_t mission_output{};
            if (mission_mode) {
                nav = aircraft.navigation_state();
                if (!ap_mission_step(&mission, &nav, static_cast<float>(l1_period_s),
                                     config.max_bank_rad, &mission_runtime, &mission_output))
                    throw std::runtime_error("Mission guidance rejected navigation, configuration, or infeasible turn geometry");
                if (mission_output.completed) { mission_completed = true; break; }
            }
            auto requested = trim;
            if (mode == "manual" && time >= pulse_start_s && time < pulse_end_s) {
                requested.aileron += static_cast<float>(pulse);
                requested.elevator += static_cast<float>(elevator_pulse);
            }
            if (time >= 2.0 && time < 7.0) {
                requested.throttle += static_cast<float>(throttle_pulse);
            }
            const float bank_command = mission_mode ? mission_output.bank_command_rad : altitude_hold ?
                (time >= 40.0 && time < 60.0 ?
                 static_cast<float>(turn_bank_deg * sim::radians_per_degree) : 0.0f) :
                speed_hold ? initial_bank : time < 2.0 ? initial_bank :
                (time < 10.0 ? static_cast<float>(bank_deg * sim::radians_per_degree) : 0.0f);
            const float pitch_command = speed_active || time < 2.0 || time >= 10.0 ? initial_pitch :
                initial_pitch + static_cast<float>(pitch_deg * sim::radians_per_degree);
            const float speed_command = time < 2.0 || time >= 30.0 ? initial_speed :
                initial_speed + static_cast<float>(speed_step_m_s);
            const float altitude_command = mission_mode ? mission_output.altitude_command_m :
                time < 2.0 || time >= 30.0 ? initial_altitude :
                initial_altitude + static_cast<float>(altitude_step_m);
            const ap_input_t input{state, requested, static_cast<float>(dt_s),
                                  altitude_active ? AP_MODE_ALTITUDE_AIRSPEED_HOLD :
                                  (speed_hold ? AP_MODE_ATTITUDE_AIRSPEED_HOLD :
                                  (attitude_hold ? AP_MODE_ATTITUDE_HOLD :
                                  (roll_hold ? AP_MODE_ROLL_HOLD :
                                  (pitch_hold ? AP_MODE_PITCH_HOLD : AP_MODE_MANUAL)))),
                                  bank_command, pitch_command, mission_mode ? mission_output.airspeed_command_m_s :
                                  (altitude_hold ? initial_speed : speed_command),
                                  altitude_command};
            ap_output_t result{};
            const bool accepted = ap_step(&config, &input, &runtime, &result);
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
                csv << (altitude_active ? result.pitch_command_rad : pitch_command)
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
                csv << (mission_mode ? mission_output.airspeed_command_m_s :
                        (altitude_hold ? initial_speed : speed_command))
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
            if (altitude_active) {
                csv << altitude_command << ',' << result.altitude_command_m << ','
                    << result.altitude_error_m;
            } else {
                csv << ",,";
            }
            csv << ',' << result.altitude_pitch_limited
                << ',' << (altitude_active ? config.altitude_gain : 0.0f)
                << ',' << (altitude_active ? config.climb_rate_gain : 0.0f)
                << ',' << (altitude_active ? config.max_pitch_offset_rad : 0.0f) << ',';
            if (mission_mode) {
                csv << nav.lat_e7 * 1e-7 << ',' << nav.lon_e7 * 1e-7 << ','
                    << mission_output.north_m << ',' << mission_output.east_m << ',' << nav.ground_north_m_s
                    << ',' << nav.ground_east_m_s << ',' << mission_output.cross_track_m
                    << ',' << mission_output.leg_index;
            } else {
                csv << ",,,,,,,";
            }
            csv << ',' << steady_wind.first << ',' << steady_wind.second << ',';
            if (mission_mode) csv << static_cast<int>(mission_output.phase) << ',' << mission_output.target_distance_m
                                  << ',' << static_cast<int>(mission_output.waypoint_type);
            else csv << ",,";
            csv << '\n';
            if (telemetry_stdout && k % 10 == 0) {
                const auto telemetry_nav = mission_mode ? nav : aircraft.navigation_state();
                if (telemetry_nav.valid) {
                    const double heading = std::fmod(state.yaw_rad * 180.0 / 3.141592653589793 + 360.0, 360.0);
                    std::cout << std::setprecision(10)
                        << "{\"time_s\":" << time
                        << ",\"lat_deg\":" << telemetry_nav.lat_e7 * 1e-7
                        << ",\"lon_deg\":" << telemetry_nav.lon_e7 * 1e-7
                        << ",\"altitude_m\":" << state.altitude_m
                        << ",\"heading_deg\":" << heading
                        << ",\"airspeed_m_s\":" << state.airspeed_m_s
                        << ",\"ground_speed_m_s\":" << std::hypot(telemetry_nav.ground_north_m_s, telemetry_nav.ground_east_m_s);
                    if (mission_mode) std::cout << ",\"mission_leg\":" << mission_output.leg_index
                        << ",\"cross_track_m\":" << mission_output.cross_track_m
                        << ",\"phase\":\"" << static_cast<int>(mission_output.phase) << "\"";
                    std::cout << "}\n" << std::flush;
                }
            }
            aircraft.step(controls);
            if (flightgear || realtime) {
                const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(static_cast<double>(k + 1) * dt_s));
                std::this_thread::sleep_until(target);
            }
        }
        csv.close();
        std::cout << "Simulated " << aircraft.time_s() << " s."
                  << (mission_completed ? " Mission complete." :
                      (mission_mode ? " Mission incomplete (duration reached)." : "")) << " CSV: "
                  << std::filesystem::absolute(output).string() << '\n';
        return mission_mode && !mission_completed ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "autopilot_sim: " << error.what() << '\n';
        return 1;
    }
}

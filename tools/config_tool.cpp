#include "sim/config_file.hpp"
#include "sim/c172x_config.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
std::vector<std::pair<const char*, float*>> fields(ap_aircraft_config_t& record)
{
    auto& c = record.control;
    return {{"roll_attitude_gain", &c.roll.attitude.gain}, {"roll_rate_kp", &c.roll.rate.kp},
        {"max_bank_rad", &c.attitude_limits.max_bank_rad}, {"max_aileron", &c.roll.rate.max_aileron_norm},
        {"pitch_attitude_gain", &c.pitch.attitude.gain}, {"pitch_rate_kp", &c.pitch.rate.kp},
        {"max_pitch_rad", &c.attitude_limits.max_pitch_rad}, {"max_elevator", &c.pitch.rate.max_elevator_norm},
        {"airspeed_kp", &c.airspeed.kp}, {"airspeed_ki", &c.airspeed.ki},
        {"min_throttle", &c.airspeed.min_throttle_norm}, {"max_throttle", &c.airspeed.max_throttle_norm},
        {"altitude_gain", &c.altitude.altitude_gain}, {"climb_rate_gain", &c.altitude.climb_rate_gain},
        {"max_pitch_offset_rad", &c.altitude.max_pitch_offset_rad}, {"l1_period_s", &record.l1_period_s},
        {"roll_rate_ki", &c.roll.rate.ki}, {"max_roll_rate_rad_s", &c.roll.attitude.max_rate_rad_s},
        {"pitch_rate_ki", &c.pitch.rate.ki}, {"max_pitch_rate_rad_s", &c.pitch.attitude.max_rate_rad_s},
        {"yaw_rate_kp", &c.yaw.rate.kp}, {"yaw_rate_ki", &c.yaw.rate.ki},
        {"max_rudder", &c.yaw.rate.max_rudder_norm}, {"rudder_sign", &c.yaw.rate.rudder_sign},
        {"max_yaw_rate_rad_s", &c.yaw.coordination.max_rate_rad_s},
        {"yaw_min_airspeed_m_s", &c.yaw.coordination.min_airspeed_m_s},
        {"yaw_max_bank_rad", &c.yaw.coordination.max_bank_rad}};
}

void set_field(ap_aircraft_config_t& record, const std::string& assignment)
{
    const auto equal = assignment.find('=');
    if (equal == std::string::npos) throw std::invalid_argument("Expected NAME=VALUE: " + assignment);
    const auto name = assignment.substr(0, equal);
    const auto value = assignment.substr(equal + 1);
    if (name == "yaw_enabled") {
        if (value != "0" && value != "1") throw std::invalid_argument("yaw_enabled must be 0 or 1");
        record.control.yaw.enabled = value == "1";
        return;
    }
    for (auto field : fields(record)) {
        if (name != field.first) continue;
        std::size_t consumed = 0;
        const float parsed = std::stof(value, &consumed);
        if (consumed != value.size() || !std::isfinite(parsed))
            throw std::invalid_argument("Expected finite number: " + assignment);
        *field.second = parsed;
        return;
    }
    throw std::invalid_argument("Unknown configuration field: " + name);
}
}

int main(int argc, char** argv)
{
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "Usage:\n"
                "  autopilot_config create OUTPUT.apcf [NAME=VALUE ...]\n"
                "  autopilot_config show INPUT.apcf\n"
                "  autopilot_config set INPUT.apcf OUTPUT.apcf NAME=VALUE [NAME=VALUE ...]\n"
                "Create starts with C172X simulation defaults (sequence 0), not Skyward settings.\n"
                "Set increments the sequence; outputs must not already exist.\n"
                "Show lists field names. Angles/rates use radians; other values use SI/normalized units.\n";
            return 0;
        }
        if (argc < 3) throw std::invalid_argument("Use --help for configuration commands");
        const std::string command = argv[1];
        if (command == "show" && argc == 3) {
            auto config = sim::load_config(argv[2]);
            std::cout << "format=APCF encoder_version=3 encoder_bytes=128 sequence=" << config.sequence << '\n';
            std::cout << std::setprecision(std::numeric_limits<float>::max_digits10);
            std::cout << "yaw_enabled=" << config.control.yaw.enabled << '\n';
            for (const auto field : fields(config)) std::cout << field.first << '=' << *field.second << '\n';
            return 0;
        }
        ap_aircraft_config_t config{sim::c172x_config, 4.0f, 0};
        std::filesystem::path output;
        int first_assignment;
        if (command == "create") {
            output = argv[2];
            first_assignment = 3;
        } else if (command == "set" && argc >= 5) {
            config = sim::load_config(argv[2]);
            if (config.sequence == UINT32_MAX)
                throw std::invalid_argument("Configuration sequence exhausted");
            ++config.sequence;
            output = argv[3];
            first_assignment = 4;
        } else {
            throw std::invalid_argument("Invalid configuration command; use --help");
        }
        for (int i = first_assignment; i < argc; ++i) set_field(config, argv[i]);
        sim::save_config(output, config);
        std::cout << "Saved 128-byte APCF v3 configuration, sequence " << config.sequence
                  << ": " << std::filesystem::absolute(output).string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "autopilot_config: " << error.what() << '\n';
        return 1;
    }
}

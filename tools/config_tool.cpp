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
    return {{"roll_angle_gain", &c.roll_angle_gain}, {"roll_rate_gain", &c.roll_rate_gain},
        {"max_bank_rad", &c.max_bank_rad}, {"max_aileron", &c.max_aileron},
        {"pitch_angle_gain", &c.pitch_angle_gain}, {"pitch_rate_gain", &c.pitch_rate_gain},
        {"max_pitch_rad", &c.max_pitch_rad}, {"max_elevator", &c.max_elevator},
        {"airspeed_kp", &c.airspeed_kp}, {"airspeed_ki", &c.airspeed_ki},
        {"min_throttle", &c.min_throttle}, {"max_throttle", &c.max_throttle},
        {"altitude_gain", &c.altitude_gain}, {"climb_rate_gain", &c.climb_rate_gain},
        {"max_pitch_offset_rad", &c.max_pitch_offset_rad}, {"l1_period_s", &record.l1_period_s}};
}

void set_field(ap_aircraft_config_t& record, const std::string& assignment)
{
    const auto equal = assignment.find('=');
    if (equal == std::string::npos) throw std::invalid_argument("Expected NAME=VALUE: " + assignment);
    const auto name = assignment.substr(0, equal);
    const auto value = assignment.substr(equal + 1);
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
            std::cout << "format=APCF version=1 bytes=80 sequence=" << config.sequence << '\n';
            std::cout << std::setprecision(std::numeric_limits<float>::max_digits10);
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
        std::cout << "Saved 80-byte APCF configuration, sequence " << config.sequence
                  << ": " << std::filesystem::absolute(output).string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "autopilot_config: " << error.what() << '\n';
        return 1;
    }
}

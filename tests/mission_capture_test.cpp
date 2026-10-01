#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
struct Case { const char* name; double north, east, heading, wind; unsigned leg; };

void run(const char* root, const std::filesystem::path& directory, const Case& test)
{
    sim::InitialPosition start{30.0 + test.north / 111194.927,
                               test.east / 96306.57, test.heading};
    sim::JsbsimAdapter aircraft(root, .01, "", 100, 0, test.wind, start);
    const auto initial = aircraft.state(); const auto trim = aircraft.trim_controls();
    ap_mission_t mission{}; mission.count = 3;
    mission.waypoints[0] = {300000000, 0, initial.altitude_m, initial.airspeed_m_s, 0, 0};
    mission.waypoints[1] = {300179864, 0, initial.altitude_m, initial.airspeed_m_s, 0, 0};
    mission.waypoints[2] = {300359729, 41538, initial.altitude_m, initial.airspeed_m_s, 0, 0};
    check(ap_mission_prepare(&mission), "Invalid capture test mission");
    ap_mission_runtime_t runtime{}; ap_runtime_t control{};
    const auto config = sim::c172x_config;
    std::ofstream csv(directory / (std::string(test.name) + ".csv"));
    csv.exceptions(std::ios::badbit | std::ios::failbit);
    csv << "time_s,north_m,east_m,mission_leg,mission_phase,cross_track_m,target_distance_m,bank_command_rad,roll_rad,altitude_error_m,airspeed_error_m_s\n";
    bool tracked = false, completed = false;
    double max_altitude = 0, max_speed = 0, max_roll = 0, tracking_time = 0;
    unsigned previous_leg = test.leg;
    std::cout << test.name << ": " << std::flush;
    for (int k = 0; k < 60000; ++k) {
        const auto state = aircraft.state(); const auto nav = aircraft.navigation_state();
        ap_mission_output_t guidance{};
        check(ap_mission_step(&mission, &nav, 4, config.max_bank_rad, &runtime, &guidance),
              "Capture guidance rejected state");
        if (k == 0) {
            check(guidance.leg_index == test.leg, "Incorrect entry segment");
            std::ofstream plan(directory / (std::string(test.name) + "_plan.csv"));
            plan.exceptions(std::ios::badbit | std::ios::failbit);
            plan << "north_m,east_m\n";
            if (runtime.capture_active) for (uint8_t segment=0;segment<3;++segment) {
                const float length=runtime.capture_path.segments[segment].length_m;
                const int steps=std::max(1,static_cast<int>(std::ceil(length/10)));
                for (int j=0;j<=steps;++j) {
                    float n,e,course;
                    check(ap_path_sample(&runtime.capture_path,segment,length*j/steps,&n,&e,&course),
                          "Could not sample planned entry");
                    plan << n << ',' << e << '\n';
                }
            }
        }
        check(guidance.leg_index >= previous_leg, "Entry segment was reselected during flight");
        previous_leg = guidance.leg_index;
        if (guidance.phase == AP_MISSION_TRACK_LEG && !tracked) { tracked = true; tracking_time = k * .01; }
        if (k % 10 == 0 || guidance.completed)
            csv << k * .01 << ',' << guidance.north_m << ',' << guidance.east_m << ','
                << guidance.leg_index << ',' << guidance.phase << ',' << guidance.cross_track_m << ','
                << guidance.target_distance_m << ',' << guidance.bank_command_rad << ',' << state.roll_rad << ','
                << state.altitude_m - initial.altitude_m << ',' << state.airspeed_m_s - initial.airspeed_m_s << '\n';
        if (guidance.completed) { completed = true; break; }
        ap_input_t input{state, trim, .01f, AP_MODE_ALTITUDE_AIRSPEED_HOLD,
                         guidance.bank_command_rad, initial.pitch_rad,
                         guidance.airspeed_command_m_s, guidance.altitude_command_m};
        ap_output_t output{};
        check(ap_step(&config, &input, &control, &output), "Controller rejected capture command");
        check(std::abs(guidance.bank_command_rad) <= config.max_bank_rad, "Capture command exceeded bank limit");
        max_altitude = std::max(max_altitude, std::abs(static_cast<double>(state.altitude_m - initial.altitude_m)));
        max_speed = std::max(max_speed, std::abs(static_cast<double>(state.airspeed_m_s - initial.airspeed_m_s)));
        max_roll = std::max(max_roll, std::abs(static_cast<double>(state.roll_rad)));
        aircraft.step(output.controls);
    }
    std::cout << "tracked=" << tracked << " at " << tracking_time << " s, completed=" << completed
              << " at " << aircraft.time_s() << " s, peak altitude/speed=" << max_altitude << '/' << max_speed << '\n';
    check(tracked && completed, "Capture did not join and complete the route within 600 s");
    check(max_altitude < 10 && max_speed < 3 && max_roll < .55, "Capture exceeded test flight envelope");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 3, "Expected JSBSim root and capture log directory");
        std::filesystem::create_directories(argv[2]);
        const Case cases[] = {
            {"west", 500,-1000,0,0,0}, {"east",500,1000,0,0,0},
            {"behind_away",-1000,0,180,0,0}, {"opposite_course",500,0,180,0,0},
            {"later_leg",3000,900,270,0,1}, {"past_finish",5000,600,0,0,1},
            {"crosswind_east",500,-1000,0,5,0}, {"crosswind_west",500,1000,0,-5,0}
        };
        for (const auto& test : cases) run(argv[1], argv[2], test);
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

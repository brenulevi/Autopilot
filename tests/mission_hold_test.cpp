#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"
#include "autopilot/mission.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
}

void run_case(const char* data_root, double wind_east_m_s)
{
    sim::JsbsimAdapter aircraft(data_root, 0.01, "", 100.0, 0.0, wind_east_m_s);
    const auto steady_wind = aircraft.steady_wind_m_s();
    check(std::abs(steady_wind.first) < 0.01 &&
          std::abs(steady_wind.second - wind_east_m_s) < 0.01,
          "JSBSim did not accept the requested steady wind");
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    const auto config = sim::c172x_config;
    ap_mission_t mission{};
    mission.count = 3;
    mission.waypoints[0] = {300000000, 0, initial.altitude_m, initial.airspeed_m_s, 0.0f, 0.0f};
    mission.waypoints[1] = {300179864, 0, initial.altitude_m, initial.airspeed_m_s, 0.0f, 0.0f};
    mission.waypoints[2] = {300359729, 41538, initial.altitude_m, initial.airspeed_m_s, 0.0f, 0.0f};
    check(ap_mission_prepare(&mission), "Invalid geodetic test mission");
    ap_mission_runtime_t mission_runtime{};
    ap_runtime_t control_runtime{};
    double settled_cross_track = 0.0;
    double settled_altitude_error = 0.0;
    double settled_speed_error = 0.0;
    double largest_bank = 0.0;
    bool saw_second_leg = false;
    bool completed = false;
    for (int k = 0; k < 9000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const auto nav = aircraft.navigation_state();
        ap_mission_output_t guidance{};
        check(ap_mission_step(&mission, &nav, 4.0f, config.max_bank_rad,
                              &mission_runtime, &guidance), "Mission guidance rejected JSBSim state");
        if (guidance.completed) { completed = true; break; }
        if (guidance.leg_index == 1) saw_second_leg = true;
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ALTITUDE_AIRSPEED_HOLD,
                               guidance.bank_command_rad, initial.pitch_rad,
                               guidance.airspeed_command_m_s, guidance.altitude_command_m};
        ap_output_t output{};
        check(ap_step_with_runtime(&config, &input, &control_runtime, &output),
              "Control rejected mission command");
        largest_bank = std::max(largest_bank, std::abs(static_cast<double>(guidance.bank_command_rad)));
        if (time >= 50.0) {
            settled_cross_track = std::max(settled_cross_track,
                std::abs(static_cast<double>(guidance.cross_track_m)));
            settled_altitude_error = std::max(settled_altitude_error,
                std::abs(static_cast<double>(state.altitude_m - guidance.altitude_command_m)));
            settled_speed_error = std::max(settled_speed_error,
                std::abs(static_cast<double>(state.airspeed_m_s - guidance.airspeed_command_m_s)));
        }
        aircraft.step(output.controls);
    }
    std::cout << "East wind " << wind_east_m_s << " m/s: mission complete "
              << completed << " at " << aircraft.time_s()
              << " s, late cross-track " << settled_cross_track << " m, altitude "
              << settled_altitude_error << " m, speed " << settled_speed_error << " m/s\n";
    check(saw_second_leg && completed, "Mission failed to advance and finish");
    check(aircraft.time_s() > 65.0 && aircraft.time_s() < 85.0, "Unexpected mission duration");
    check(settled_cross_track < 5.0, "Late cross-track exceeded 5 m");
    check(settled_altitude_error < 0.5, "Late altitude error exceeded 0.5 m");
    check(settled_speed_error < 0.5, "Late speed error exceeded 0.5 m/s");
    check(largest_bank <= config.max_bank_rad, "Bank command exceeded limit");
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        run_case(argv[1], 0.0);
        run_case(argv[1], 5.0);
        run_case(argv[1], -5.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

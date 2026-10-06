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

void run_case(const char* root, double initial_kts, double step_m_s)
{
    sim::JsbsimAdapter aircraft(root, 0.01, {}, initial_kts);
    const auto trim = aircraft.trim_controls();
    const auto initial = aircraft.state();
    const auto config = sim::c172x_config;
    ap_runtime_t runtime{};
    double hold_error = 0.0;
    double return_error = 0.0;
    for (int k = 0; k < 6000; ++k) {
        const double time = k * 0.01;
        const auto state = aircraft.state();
        const float command = initial.airspeed_m_s + (time >= 2.0 && time < 30.0 ?
            static_cast<float>(step_m_s) : 0.0f);
        const ap_input_t input{state, trim, 0.01f, AP_MODE_ATTITUDE_AIRSPEED_HOLD,
                               initial.roll_rad, initial.pitch_rad, command};
        ap_output_t output{};
        check(ap_step(&config, &input, &runtime, &output),
              "Airspeed hold rejected simulation state");
        check(!output.throttle_saturated, "Small speed step saturated throttle");
        check(output.controls.throttle >= 0.0f && output.controls.throttle <= 1.0f,
              "Throttle command out of range");
        check(output.yaw_control_active && !output.rudder_saturated, "Airspeed hold yaw control failed");
        const double error = std::abs(command - state.airspeed_m_s);
        if (time >= 20.0 && time < 30.0) hold_error = std::max(hold_error, error);
        if (time >= 50.0) return_error = std::max(return_error, error);
        aircraft.step(output.controls);
    }
    std::cout << initial_kts << " kt initial, step " << step_m_s
              << " m/s: max later hold error " << hold_error
              << " m/s, return error " << return_error << " m/s\n";
    check(hold_error < 0.3, "Airspeed hold error exceeded 0.3 m/s");
    check(return_error < 0.3, "Airspeed return error exceeded 0.3 m/s");
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        run_case(argv[1], 90.0, 1.0);
        run_case(argv[1], 100.0, 1.0);
        run_case(argv[1], 110.0, 1.0);
        run_case(argv[1], 100.0, -1.0);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

#include "sim/jsbsim_adapter.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Expected JSBSim data root");
        sim::JsbsimAdapter aircraft(argv[1], 0.01);
        const auto initial = aircraft.state();
        const auto trim = aircraft.trim_controls();
        check(std::abs(initial.altitude_m - 914.4f) < 0.5f, "Altitude unit conversion failed");
        check(initial.airspeed_m_s > 45.0f && initial.airspeed_m_s < 65.0f, "Airspeed outside expected cruise range");
        for (int k = 0; k < 100; ++k) {
            const ap_input_t input{aircraft.state(), trim, 0.01f, AP_MODE_MANUAL, 0.0f};
            ap_output_t result{};
            check(ap_step(nullptr, &input, &result), "C core rejected trimmed state");
            aircraft.step(result.controls);
        }
        const auto steady = aircraft.state();
        check(std::abs(steady.altitude_m - initial.altitude_m) < 1.0f, "Trim altitude drift exceeds 1 m in 1 s");
        check(std::abs(steady.airspeed_m_s - initial.airspeed_m_s) < 0.5f, "Trim speed drift exceeds 0.5 m/s in 1 s");
        check(std::abs(steady.roll_rad - initial.roll_rad) < 0.02f, "Unexpected trimmed roll drift");

        auto demand = trim;
        demand.aileron += 0.05f;
        for (int k = 0; k < 50; ++k) {
            const ap_input_t input{aircraft.state(), demand, 0.01f, AP_MODE_MANUAL, 0.0f};
            ap_output_t result{};
            check(ap_step(nullptr, &input, &result), "C core rejected pulse state");
            aircraft.step(result.controls);
        }
        const auto response = aircraft.state();
        const ap_input_t final_input{response, trim, 0.01f, AP_MODE_MANUAL, 0.0f};
        ap_output_t final_result{};
        check(ap_step(nullptr, &final_input, &final_result), "Pulse produced non-finite state");
        check(response.roll_rad > steady.roll_rad + 0.001f, "Positive aileron did not produce positive roll");
        check(response.p_rad_s > 0.001f, "Positive aileron did not produce positive roll rate");
        check(std::abs(aircraft.time_s() - 1.5) < 1e-8, "Simulation time did not advance correctly");
        std::cout << "Trim held; C/C++ interface and positive aileron response verified.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

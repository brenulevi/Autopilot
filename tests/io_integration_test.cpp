#include "autopilot/autopilot.h"
#include "flight_io/supervisor.h"
#include "flight_io/actuators.h"
#include "sim/c172x_config.hpp"
#include "sim/jsbsim_adapter.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

static_assert(std::is_same_v<ap_controls_t, flight_controls_t>, "Both libraries must share the command type");

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
        sim::JsbsimAdapter aircraft(argv[1], .01);
        const auto trim = aircraft.trim_controls();
        const auto config = sim::c172x_config;
        const fio_supervisor_config_t io_config{100, 50, {0,0,0,0}, trim};
        fio_supervisor_runtime_t runtime{};
        ap_runtime_t control_runtime{};
        flight_control_sample_t ap{};
        bool observed_timeout = false, observed_reengagement = false;
        for (uint32_t tick = 0; tick < 600; ++tick) {
            const uint32_t now = tick * 10;
            ap_input_t input{};
            input.state = aircraft.state();
            input.requested = trim;
            input.dt_s = .01f;
            input.mode = AP_MODE_ROLL_HOLD;
            input.bank_command_rad = static_cast<float>(5 * sim::radians_per_degree);
            ap_output_t result{};
            // One estimator failure while MANUAL is requested.
            if (tick == 100) input.state.roll_rad = std::numeric_limits<float>::quiet_NaN();
            bool accepted = ap_step(&config, &input, &control_runtime, &result);
            check(accepted == (tick != 100), "Unexpected autopilot validation result");
            // Simulate a stalled H723 link during [3,4) seconds: no timestamp refresh.
            if (tick < 300 || tick >= 400)
                ap = flight_control_sample_t{result.controls, now, accepted};
            flight_rc_sample_t rc{trim, now, true, true};
            rc.request_auto = !(tick < 10 || (tick >= 100 && tick < 120) || tick == 450);
            if (tick >= 100 && tick < 120) rc.controls.aileron += .03f;
            fio_supervisor_output_t selected{};
            check(fio_supervisor_step(&io_config, now, true, &rc, &ap, &runtime, &selected), "Supervisor rejected inputs");
            if (!rc.request_auto) {
                check(selected.status.authority == FLIGHT_AUTHORITY_MANUAL, "Pilot takeover failed");
                check(selected.controls.aileron == rc.controls.aileron, "Manual demand was changed");
            } else if (tick >= 305 && tick < 400) {
                check(selected.status.reason == FLIGHT_REASON_AUTOPILOT_UNAVAILABLE, "Stale H723 command was used");
                observed_timeout = true;
            } else if (tick >= 400 && tick < 450) {
                check(selected.status.reason == FLIGHT_REASON_REENGAGEMENT_REQUIRED, "Restart silently re-engaged AUTO");
                observed_reengagement = true;
            } else {
                check(selected.status.authority == FLIGHT_AUTHORITY_AUTOPILOT, "AUTO did not engage when permitted");
                check(selected.controls.aileron == ap.controls.aileron, "Autopilot demand was changed");
            }
            aircraft.step(selected.controls);
        }
        check(observed_timeout && observed_reengagement, "Failure/recovery scenario was not exercised");
        check(std::isfinite(aircraft.state().roll_rad), "Nonfinite simulator state");
        std::cout << "Shared controls: manual override, invalid estimator, H723 timeout, and deliberate re-engagement passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

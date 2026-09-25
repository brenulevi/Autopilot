#include "sim/jsbsim_adapter.hpp"

#include <FGFDMExec.h>
#include <initialization/FGTrim.h>
#include <simgear/misc/sg_path.hxx>

#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace sim {
namespace {
constexpr double meters_per_foot = 0.3048;
}

JsbsimAdapter::JsbsimAdapter(const std::string& data_root, double dt_s,
                             const std::string& flightgear_output_file, double airspeed_kts)
{
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("Simulation timestep must be finite and positive");
    }
    if (!std::isfinite(airspeed_kts) || airspeed_kts < 80.0 || airspeed_kts > 120.0) {
        throw std::invalid_argument("Initial calibrated airspeed must be in [80, 120] kt");
    }
    const auto root = std::filesystem::absolute(data_root);
    if (!std::filesystem::is_regular_file(root / "aircraft/c172x/c172x.xml")) {
        throw std::runtime_error("Missing aircraft/c172x/c172x.xml under JSBSim root: " + root.string());
    }

    fdm_ = std::make_unique<JSBSim::FGFDMExec>();
    fdm_->SetDebugLevel(0);
    fdm_->SetRootDir(SGPath(root.generic_string()));
    fdm_->SetAircraftPath(SGPath("aircraft"));
    fdm_->SetEnginePath(SGPath("engine"));
    fdm_->SetSystemsPath(SGPath("systems"));
    fdm_->Setdt(dt_s);
    if (!flightgear_output_file.empty() &&
        !fdm_->SetOutputDirectives(SGPath(std::filesystem::absolute(flightgear_output_file).generic_string()))) {
        throw std::runtime_error("Could not load FlightGear output directives: " + flightgear_output_file);
    }
    if (!fdm_->LoadModel("c172x")) {
        throw std::runtime_error("JSBSim could not load c172x");
    }
    if (flightgear_output_file.empty()) fdm_->DisableOutput();
    // The runner owns CSV logging. With FlightGear enabled, its stream is
    // output #0 and the aircraft's built-in CSV moves to output #1.
    // RunIC still opens the model's built-in CSV and writes its header even
    // when output is disabled. Send that stream to the platform's null device.
#ifdef _WIN32
    fdm_->SetOutputFileName(flightgear_output_file.empty() ? 0 : 1, "NUL");
#else
    fdm_->SetOutputFileName(flightgear_output_file.empty() ? 0 : 1, "/dev/null");
#endif

    // Both upstream autopilot systems must remain disengaged.
    for (const char* property : {"ap/attitude_hold", "ap/altitude_hold",
                                "ap/heading_hold", "ap/airspeed_hold",
                                "ap/autopilot-roll-on"}) {
        fdm_->SetPropertyValue(property, 0.0);
    }

    // A C172X learning condition, not a Skyward flight condition.
    fdm_->SetPropertyValue("ic/h-sl-ft", 3000.0);
    fdm_->SetPropertyValue("ic/terrain-elevation-ft", 0.0);
    fdm_->SetPropertyValue("ic/vc-kts", airspeed_kts);
    fdm_->SetPropertyValue("ic/lat-geod-deg", 30.0);
    fdm_->SetPropertyValue("ic/long-gc-deg", 0.0);
    fdm_->SetPropertyValue("ic/psi-true-deg", 0.0);
    fdm_->SetPropertyValue("ic/gamma-deg", 0.0);
    if (!fdm_->RunIC()) {
        throw std::runtime_error("JSBSim initial conditions failed");
    }
    fdm_->SetPropertyValue("propulsion/set-running", -1.0);
    fdm_->DoTrim(JSBSim::tFull); // Throws on failure; never continue untrimmed.

    trim_.aileron = static_cast<float>(fdm_->GetPropertyValue("fcs/aileron-cmd-norm"));
    trim_.elevator = static_cast<float>(fdm_->GetPropertyValue("fcs/elevator-cmd-norm"));
    trim_.rudder = static_cast<float>(fdm_->GetPropertyValue("fcs/rudder-cmd-norm"));
    trim_.throttle = static_cast<float>(fdm_->GetPropertyValue("fcs/throttle-cmd-norm"));
}

JsbsimAdapter::~JsbsimAdapter() = default;

ap_state_t JsbsimAdapter::state() const
{
    return {
        static_cast<float>(fdm_->GetPropertyValue("attitude/phi-rad")),
        static_cast<float>(fdm_->GetPropertyValue("attitude/theta-rad")),
        static_cast<float>(fdm_->GetPropertyValue("attitude/psi-rad")),
        static_cast<float>(fdm_->GetPropertyValue("velocities/p-rad_sec")),
        static_cast<float>(fdm_->GetPropertyValue("velocities/q-rad_sec")),
        static_cast<float>(fdm_->GetPropertyValue("velocities/r-rad_sec")),
        static_cast<float>(fdm_->GetPropertyValue("velocities/vtrue-fps") * meters_per_foot),
        static_cast<float>(fdm_->GetPropertyValue("position/h-sl-ft") * meters_per_foot),
        static_cast<float>(fdm_->GetPropertyValue("velocities/h-dot-fps") * meters_per_foot)
    };
}

ap_controls_t JsbsimAdapter::trim_controls() const { return trim_; }
double JsbsimAdapter::time_s() const { return fdm_->GetSimTime(); }
double JsbsimAdapter::elevator_position_rad() const
{ return fdm_->GetPropertyValue("fcs/elevator-pos-rad"); }
double JsbsimAdapter::upstream_elevator_command() const
{ return fdm_->GetPropertyValue("ap/elevator_cmd"); }
double JsbsimAdapter::pitch_trim_command() const
{ return fdm_->GetPropertyValue("fcs/pitch-trim-cmd-norm"); }

void JsbsimAdapter::step(const ap_controls_t& controls)
{
    fdm_->SetPropertyValue("fcs/aileron-cmd-norm", controls.aileron);
    fdm_->SetPropertyValue("fcs/elevator-cmd-norm", controls.elevator);
    fdm_->SetPropertyValue("fcs/rudder-cmd-norm", controls.rudder);
    fdm_->SetPropertyValue("fcs/throttle-cmd-norm", controls.throttle);
    if (!fdm_->Run()) {
        throw std::runtime_error("JSBSim stopped unexpectedly");
    }
}

} // namespace sim

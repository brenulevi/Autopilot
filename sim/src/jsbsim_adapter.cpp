#include "sim/jsbsim_adapter.hpp"

#include <FGFDMExec.h>
#include <initialization/FGTrim.h>
#include <simgear/misc/sg_path.hxx>

#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace sim {

JsbsimAdapter::JsbsimAdapter(const std::string& data_root, double dt_s)
{
    if (!std::isfinite(dt_s) || dt_s <= 0.0)
        throw std::invalid_argument("Simulation timestep must be finite and positive");

    const auto root = std::filesystem::absolute(data_root);
    if (!std::filesystem::is_regular_file(root / "aircraft/c172x/c172x.xml"))
        throw std::runtime_error("Missing JSBSim C172X data under " + root.string());

    fdm_ = std::make_unique<JSBSim::FGFDMExec>();
    fdm_->SetDebugLevel(0);
    fdm_->SetRootDir(SGPath(root.generic_string()));
    fdm_->SetAircraftPath(SGPath("aircraft"));
    fdm_->SetEnginePath(SGPath("engine"));
    fdm_->SetSystemsPath(SGPath("systems"));
    fdm_->Setdt(dt_s);
    if (!fdm_->LoadModel("c172x"))
        throw std::runtime_error("JSBSim could not load the C172X");
    fdm_->DisableOutput();

    /* The model's own output opens during RunIC even when disabled. */
#ifdef _WIN32
    fdm_->SetOutputFileName(0, "NUL");
#else
    fdm_->SetOutputFileName(0, "/dev/null");
#endif

    /* Do not let the C172X model's built-in autopilot compete with ours. */
    for (const char* property : {"ap/attitude_hold", "ap/altitude_hold",
                                 "ap/heading_hold", "ap/airspeed_hold",
                                 "ap/autopilot-roll-on"})
        fdm_->SetPropertyValue(property, 0.0);

    fdm_->SetPropertyValue("ic/h-sl-ft", 3000.0);
    fdm_->SetPropertyValue("ic/terrain-elevation-ft", 0.0);
    fdm_->SetPropertyValue("ic/vc-kts", 100.0);
    fdm_->SetPropertyValue("ic/lat-geod-deg", 30.0);
    fdm_->SetPropertyValue("ic/long-gc-deg", 0.0);
    fdm_->SetPropertyValue("ic/psi-true-deg", 0.0);
    fdm_->SetPropertyValue("ic/gamma-deg", 0.0);
    if (!fdm_->RunIC())
        throw std::runtime_error("JSBSim initial conditions failed");
    fdm_->SetPropertyValue("propulsion/set-running", -1.0);
    fdm_->DoTrim(JSBSim::tFull);

    trim_.aileron = static_cast<float>(fdm_->GetPropertyValue("fcs/aileron-cmd-norm"));
    trim_.elevator = static_cast<float>(fdm_->GetPropertyValue("fcs/elevator-cmd-norm"));
    trim_.rudder = static_cast<float>(fdm_->GetPropertyValue("fcs/rudder-cmd-norm"));
    trim_.throttle = static_cast<float>(fdm_->GetPropertyValue("fcs/throttle-cmd-norm"));
    if (!std::isfinite(trim_.aileron) || !std::isfinite(trim_.elevator) ||
        !std::isfinite(trim_.rudder) || !std::isfinite(trim_.throttle))
        throw std::runtime_error("JSBSim produced non-finite trim controls");
}

JsbsimAdapter::~JsbsimAdapter() = default;

AttitudeState JsbsimAdapter::attitude_state() const
{
    return {
        static_cast<float>(fdm_->GetPropertyValue("attitude/phi-rad")),
        static_cast<float>(fdm_->GetPropertyValue("attitude/theta-rad")),
        static_cast<float>(fdm_->GetPropertyValue("velocities/p-rad_sec")),
        static_cast<float>(fdm_->GetPropertyValue("velocities/q-rad_sec"))
    };
}

ElevatorActuatorState JsbsimAdapter::elevator_actuator_state() const
{
    return {
        static_cast<float>(fdm_->GetPropertyValue("fcs/elevator-control")),
        static_cast<float>(fdm_->GetPropertyValue("fcs/elevator-pos-rad"))
    };
}

float JsbsimAdapter::airspeed_m_s() const
{
    constexpr float metres_per_second_per_knot = 0.514444f;
    return static_cast<float>(fdm_->GetPropertyValue("velocities/vc-kts")) *
           metres_per_second_per_knot;
}

Controls JsbsimAdapter::trim_controls() const { return trim_; }
double JsbsimAdapter::time_s() const { return fdm_->GetSimTime(); }

void JsbsimAdapter::step(const Controls& controls)
{
    fdm_->SetPropertyValue("fcs/aileron-cmd-norm", controls.aileron);
    fdm_->SetPropertyValue("fcs/elevator-cmd-norm", controls.elevator);
    fdm_->SetPropertyValue("fcs/rudder-cmd-norm", controls.rudder);
    fdm_->SetPropertyValue("fcs/throttle-cmd-norm", controls.throttle);
    if (!fdm_->Run())
        throw std::runtime_error("JSBSim stopped unexpectedly");
}

} // namespace sim

#pragma once

#include <memory>
#include <string>

namespace JSBSim { class FGFDMExec; }

namespace sim {

struct AttitudeState {
    float bank_rad;
    float pitch_rad;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
};

/* JSBSim's physical elevator request before actuator lag and hysteresis,
 * and the resulting surface position. Both values are radians. */
struct ElevatorActuatorState {
    float requested_rad;
    float actual_rad;
};

struct Controls {
    float aileron;
    float elevator;
    float rudder;
    float throttle;
};

/* JSBSim stays behind this boundary; the C autopilot sees plain numbers. */
class JsbsimAdapter {
public:
    JsbsimAdapter(const std::string& data_root, double dt_s);
    ~JsbsimAdapter();
    JsbsimAdapter(const JsbsimAdapter&) = delete;
    JsbsimAdapter& operator=(const JsbsimAdapter&) = delete;

    AttitudeState attitude_state() const;
    ElevatorActuatorState elevator_actuator_state() const;
    float airspeed_m_s() const;
    Controls trim_controls() const;
    double time_s() const;
    void step(const Controls& controls);

private:
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    Controls trim_{};
};

} // namespace sim

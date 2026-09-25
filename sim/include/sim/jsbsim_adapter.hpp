#pragma once

#include "autopilot/autopilot.h"

#include <memory>
#include <string>

namespace JSBSim { class FGFDMExec; }

namespace sim {

// Native simulation boundary. JSBSim types stay out of the C library.
class JsbsimAdapter {
public:
    JsbsimAdapter(const std::string& data_root, double dt_s,
                  const std::string& flightgear_output_file = {}, double airspeed_kts = 100.0);
    ~JsbsimAdapter();
    JsbsimAdapter(const JsbsimAdapter&) = delete;
    JsbsimAdapter& operator=(const JsbsimAdapter&) = delete;

    ap_state_t state() const;
    ap_controls_t trim_controls() const;
    double time_s() const;
    double elevator_position_rad() const;
    double upstream_elevator_command() const;
    double pitch_trim_command() const;
    void step(const ap_controls_t& controls);

private:
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    ap_controls_t trim_{};
};

} // namespace sim

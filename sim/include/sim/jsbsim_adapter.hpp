#pragma once

#include "autopilot/autopilot.h"
#include "autopilot/mission.h"

#include <memory>
#include <string>
#include <utility>

namespace JSBSim { class FGFDMExec; }

namespace sim {

struct InitialPosition {
    double latitude_deg = 30.0;
    double longitude_deg = 0.0;
    double heading_deg = 0.0; // True heading, [0,360); wind can change ground course.
};

// Native simulation boundary. JSBSim types stay out of the C library.
class JsbsimAdapter {
public:
    JsbsimAdapter(const std::string& data_root, double dt_s,
                  const std::string& flightgear_output_file = {}, double airspeed_kts = 100.0,
                  double wind_north_m_s = 0.0, double wind_east_m_s = 0.0,
                  const InitialPosition& start = {});
    ~JsbsimAdapter();
    JsbsimAdapter(const JsbsimAdapter&) = delete;
    JsbsimAdapter& operator=(const JsbsimAdapter&) = delete;

    ap_state_t state() const;
    ap_nav_state_t navigation_state() const;
    std::pair<double, double> steady_wind_m_s() const;
    ap_controls_t trim_controls() const;
    double time_s() const;
    double elevator_position_rad() const;
    double upstream_elevator_command() const;
    double pitch_trim_command() const;
    double sideslip_rad() const;
    double lateral_specific_force_m_s2() const;
    void step(const ap_controls_t& controls);

private:
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    ap_controls_t trim_{};
};

} // namespace sim

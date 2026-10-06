#include "sim/jsbsim_adapter.hpp"
#include "sim/c172x_config.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message)
{ if (!condition) throw std::runtime_error(message); }
struct Case { const char *name; ap_waypoint_type_t type; int direction; double wind; bool mixed; };

void run(const char* root, const std::filesystem::path& directory, const Case& test)
{
    sim::JsbsimAdapter aircraft(root,.01,"",100,0,test.wind);
    const auto initial=aircraft.state(); const auto trim=aircraft.trim_controls();
    const auto config=sim::c172x_config;
    ap_mission_t mission{}; mission.count=test.mixed ? 4 : 3;
    const double north[]={0,5000,5000,10000};
    const double east[]={0,0,5000.0*test.direction,5000.0*test.direction};
    for (unsigned i=0;i<mission.count;++i) {
        mission.waypoints[i]={static_cast<int32_t>(std::lround(300000000+north[i]/.0111194927)),
            static_cast<int32_t>(std::lround(east[i]/.009630657)),initial.altitude_m,initial.airspeed_m_s,0,0,
            i==1 ? test.type : AP_WAYPOINT_FLY_OVER};
    }
    check(ap_mission_prepare(&mission),"Invalid turn test mission");
    std::ofstream route(directory/(std::string(test.name)+"_route.csv"));
    route.exceptions(std::ios::badbit|std::ios::failbit);
    route << "north_m,east_m,type\n";
    for(unsigned i=0;i<mission.count;++i)
        route << mission.waypoints[i].north_m << ',' << mission.waypoints[i].east_m << ',' << mission.waypoints[i].type << '\n';
    std::ofstream csv(directory/(std::string(test.name)+".csv"));
    csv.exceptions(std::ios::badbit|std::ios::failbit);
    csv << "time_s,north_m,east_m,mission_leg,phase,type,bank_command_rad,altitude_error_m,airspeed_error_m_s\n";
    ap_mission_runtime_t runtime{}; ap_runtime_t controller{};
    double closest[2]={1e9,1e9}; bool saw_turn[2]={false,false};
    double max_altitude=0,max_speed=0,max_roll=0;
    bool complete=false; unsigned previous=0;
    std::cout << test.name << ": " << std::flush;
    for(int k=0;k<70000;++k) {
        const auto state=aircraft.state(); const auto nav=aircraft.navigation_state();
        ap_mission_output_t guidance{};
        check(ap_mission_step(&mission,&nav,4,config.attitude_limits.max_bank_rad,&runtime,&guidance),"Turn guidance rejected mission/state");
        check(guidance.leg_index>=previous,"Mission leg went backwards"); previous=guidance.leg_index;
        check(std::abs(guidance.bank_command_rad)<=config.attitude_limits.max_bank_rad,"Turn exceeded bank command limit");
        for(unsigned w=1;w<mission.count-1;++w) {
            const auto& b=mission.waypoints[w]; const auto& a=mission.waypoints[w-1];
            const double distance=std::hypot(guidance.north_m-b.north_m,guidance.east_m-b.east_m);
            closest[w-1]=std::min(closest[w-1],distance);
            const bool turning=guidance.leg_index==w-1 &&
                (guidance.phase==AP_MISSION_FLY_BY_TURN || guidance.phase==AP_MISSION_FLY_OVER_TURN);
            if(turning && !saw_turn[w-1]) {
                const double length=std::hypot(b.north_m-a.north_m,b.east_m-a.east_m);
                const double beyond=((guidance.north_m-b.north_m)*(b.north_m-a.north_m)+
                                     (guidance.east_m-b.east_m)*(b.east_m-a.east_m))/length;
                if(b.type==AP_WAYPOINT_FLY_OVER)
                    check(guidance.phase==AP_MISSION_FLY_OVER_TURN && beyond>=-.06 && distance<=50.1,
                          "Fly-over turn started before/away from waypoint passage");
                else check(guidance.phase==AP_MISSION_FLY_BY_TURN && beyond<-100,
                           "Fly-by failed to anticipate the corner");
                saw_turn[w-1]=true;
            }
        }
        if(k%10==0 || guidance.completed)
            csv << k*.01 << ',' << guidance.north_m << ',' << guidance.east_m << ',' << guidance.leg_index << ','
                << guidance.phase << ',' << guidance.waypoint_type << ',' << guidance.bank_command_rad << ','
                << state.altitude_m-initial.altitude_m << ',' << state.airspeed_m_s-initial.airspeed_m_s << '\n';
        if(guidance.completed) {complete=true;break;}
        const ap_input_t input{state,trim,.01f,AP_MODE_ALTITUDE_AIRSPEED_HOLD,guidance.bank_command_rad,
                              initial.pitch_rad,guidance.airspeed_command_m_s,guidance.altitude_command_m};
        ap_output_t output{};
        check(ap_step(&config,&input,&controller,&output),"Control rejected turn command");
        max_altitude=std::max(max_altitude,std::abs(static_cast<double>(state.altitude_m-initial.altitude_m)));
        max_speed=std::max(max_speed,std::abs(static_cast<double>(state.airspeed_m_s-initial.airspeed_m_s)));
        max_roll=std::max(max_roll,std::abs(static_cast<double>(state.roll_rad)));
        aircraft.step(output.controls);
    }
    std::cout << "complete=" << complete << " at " << aircraft.time_s() << " s; closest corner="
              << closest[0] << " m; peak altitude/speed=" << max_altitude << '/' << max_speed << '\n';
    check(complete,"Turn mission did not finish within 700 simulated seconds");
    for(unsigned w=1;w<mission.count-1;++w) {
        check(saw_turn[w-1],"No turn phase observed for an intermediate waypoint");
        if(mission.waypoints[w].type==AP_WAYPOINT_FLY_OVER) check(closest[w-1]<50,"Fly-over missed passage tolerance");
        else check(closest[w-1]>100,"Fly-by did not cut the corner");
    }
    check(max_altitude<10 && max_speed<3 && max_roll<.55,"Turn exceeded test envelope");
}
}
int main(int argc,char** argv)
{
    try {
        check(argc==3,"Expected JSBSim root and turn log directory");
        std::filesystem::create_directories(argv[2]);
        const Case cases[]={
            {"right_fly_by",AP_WAYPOINT_FLY_BY,1,0,false},
            {"right_fly_over",AP_WAYPOINT_FLY_OVER,1,0,false},
            {"left_fly_by",AP_WAYPOINT_FLY_BY,-1,0,false},
            {"left_fly_over",AP_WAYPOINT_FLY_OVER,-1,0,false},
            {"wind_fly_by",AP_WAYPOINT_FLY_BY,1,5,false},
            {"wind_fly_over",AP_WAYPOINT_FLY_OVER,1,5,false},
            {"mixed",AP_WAYPOINT_FLY_BY,1,0,true}
        };
        for(const auto& test:cases) run(argv[1],argv[2],test);
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}

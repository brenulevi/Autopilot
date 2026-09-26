#include "autopilot/mission.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); return 1; } } while (0)
static void position(const ap_mission_t *m, ap_nav_state_t *nav, float n, float e)
{
    nav->lat_e7=m->origin_lat_e7+(int32_t)lroundf(n/m->north_m_per_e7);
    nav->lon_e7=m->origin_lon_e7+(int32_t)lroundf(e/m->east_m_per_e7);
}
static bool step(const ap_mission_t *m, ap_nav_state_t *nav, ap_mission_runtime_t *r, ap_mission_output_t *o)
{ return ap_mission_step(m,nav,4,.35f,r,o); }

int main(void)
{
    ap_mission_t m={0}; m.count=3;
    m.waypoints[0]=(ap_waypoint_t){300000000,0,900,20,0,0,AP_WAYPOINT_FLY_BY};
    m.waypoints[1]=(ap_waypoint_t){300449661,0,900,20,0,0,AP_WAYPOINT_FLY_BY};
    m.waypoints[2]=(ap_waypoint_t){300449661,519207,900,20,0,0,AP_WAYPOINT_FLY_OVER};
    CHECK(ap_mission_prepare(&m));
    const float corner=m.waypoints[1].north_m;
    ap_nav_state_t nav={0,0,20,0,true}; ap_mission_runtime_t r={0}; ap_mission_output_t o={0};
    position(&m,&nav,4000,0);
    CHECK(step(&m,&nav,&r,&o));
    CHECK(o.phase==AP_MISSION_TRACK_LEG && fabsf(o.bank_command_rad)<.001f);
    position(&m,&nav,corner-100,0);
    CHECK(step(&m,&nav,&r,&o));
    CHECK(o.phase==AP_MISSION_FLY_BY_TURN && o.bank_command_rad>0 && r.leg_index==0);
    float exit_n,exit_e,exit_course;
    CHECK(ap_path_sample(&r.turn_path,1,r.turn_path.segments[1].length_m,&exit_n,&exit_e,&exit_course));
    position(&m,&nav,exit_n,exit_e+1); nav.ground_north_m_s=0; nav.ground_east_m_s=20;
    CHECK(step(&m,&nav,&r,&o) && r.leg_index==1);

    m.waypoints[1].type=AP_WAYPOINT_FLY_OVER;
    r=(ap_mission_runtime_t){0}; nav.ground_north_m_s=20; nav.ground_east_m_s=0;
    position(&m,&nav,4000,0);
    CHECK(step(&m,&nav,&r,&o));
    position(&m,&nav,corner-10,0); /* Inside the acceptance circle but BEFORE passage. */
    CHECK(step(&m,&nav,&r,&o));
    CHECK(!r.turn_active && o.phase==AP_MISSION_TRACK_LEG && fabsf(o.bank_command_rad)<.001f);
    nav.lat_e7=m.waypoints[1].lat_e7; nav.lon_e7=m.waypoints[1].lon_e7;
    CHECK(step(&m,&nav,&r,&o));
    CHECK(r.turn_active && o.phase==AP_MISSION_FLY_OVER_TURN && r.leg_index==0);

    /* Crossing the plane far to the side is a miss, and must trigger recovery. */
    r=(ap_mission_runtime_t){0}; position(&m,&nav,4000,0);
    CHECK(step(&m,&nav,&r,&o));
    position(&m,&nav,corner+100,500);
    CHECK(step(&m,&nav,&r,&o));
    CHECK(r.leg_index==0 && !r.turn_active && r.capture_active && o.phase==AP_MISSION_CAPTURE_LEG);

    /* No shortening a radius to squeeze an infeasible fly-by into short legs. */
    m.waypoints[1].lat_e7=300004497; m.waypoints[1].type=AP_WAYPOINT_FLY_BY;
    m.waypoints[2].lat_e7=300004497; m.waypoints[2].lon_e7=5192;
    CHECK(ap_mission_prepare(&m)); r=(ap_mission_runtime_t){0};
    position(&m,&nav,0,0);
    unsigned char old_output[sizeof(o)]; memcpy(old_output,&o,sizeof(o));
    CHECK(!step(&m,&nav,&r,&o) && !r.initialized);
    CHECK(memcmp(old_output,&o,sizeof(o))==0);
    m.waypoints[1].type=AP_WAYPOINT_FLY_OVER;
    CHECK(step(&m,&nav,&r,&o));
    m.waypoints[1].type=(ap_waypoint_type_t)2;
    CHECK(!ap_mission_prepare(&m));
    return 0;
}

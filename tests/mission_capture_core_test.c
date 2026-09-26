#include "autopilot/mission.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

static void position(const ap_mission_t *mission, ap_nav_state_t *nav, float north, float east)
{
    nav->lat_e7 = mission->origin_lat_e7 + (int32_t)lroundf(north / mission->north_m_per_e7);
    nav->lon_e7 = mission->origin_lon_e7 + (int32_t)lroundf(east / mission->east_m_per_e7);
}
static bool step(const ap_mission_t *mission, ap_nav_state_t *nav,
                 ap_mission_runtime_t *runtime, ap_mission_output_t *output)
{
    return ap_mission_step(mission, nav, 4.0f, 0.35f, runtime, output);
}

int main(void)
{
    ap_mission_t mission = {0}; mission.count = 4;
    mission.waypoints[0] = (ap_waypoint_t){300000000, 0, 900, 50, 0, 0};
    mission.waypoints[1] = (ap_waypoint_t){300179864, 0, 910, 51, 0, 0};
    mission.waypoints[2] = (ap_waypoint_t){300179864, 207683, 920, 52, 0, 0};
    mission.waypoints[3] = (ap_waypoint_t){300359729, 207683, 930, 53, 0, 0};
    for (unsigned i=0;i<mission.count;++i) mission.waypoints[i].type=AP_WAYPOINT_FLY_OVER;
    CHECK(ap_mission_prepare(&mission));
    ap_nav_state_t nav = {0,0,50,0,true};
    ap_mission_runtime_t runtime = {0}; ap_mission_output_t output = {0};
    position(&mission, &nav, 500, -1000);
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(runtime.initialized && output.leg_index == 0 && output.phase == AP_MISSION_CAPTURE_LEG);
    CHECK(output.bank_command_rad > 0 && output.altitude_command_m == 910);

    /* Selection is persistent even when a different leg becomes nearer. */
    position(&mission, &nav, 3000, 2400);
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.leg_index == 0 && !output.completed);
    runtime = (ap_mission_runtime_t){0};
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.leg_index == 2 && output.altitude_command_m == 930 && output.airspeed_command_m_s == 53);

    /* Finite segments: the extension of leg 0 is not treated as distance zero. */
    position(&mission, &nav, 5000, 0); runtime = (ap_mission_runtime_t){0};
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.leg_index == 2 && !output.completed);

    /* Exactly opposite course must steer; tiny sign changes retain turn side. */
    position(&mission, &nav, 500, 0); nav.ground_north_m_s = -50;
    runtime = (ap_mission_runtime_t){0};
    runtime.initialized = true; /* Exercise course recovery on an existing leg. */
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.phase == AP_MISSION_CAPTURE_LEG && output.bank_command_rad > 0.3f);
    nav.ground_east_m_s = 0.01f;
    CHECK(step(&mission, &nav, &runtime, &output) && output.bank_command_rad > 0);
    nav.ground_east_m_s = -0.01f;
    CHECK(step(&mission, &nav, &runtime, &output) && output.bank_command_rad > 0);
    nav.ground_north_m_s = 50; nav.ground_east_m_s = 0;
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.phase == AP_MISSION_TRACK_LEG && runtime.turn_direction == 0);
    CHECK(fabsf(output.bank_command_rad) < 0.0001f);

    /* At a shared endpoint, ties prefer forward course before index. */
    nav.lat_e7 = mission.waypoints[1].lat_e7; nav.lon_e7 = mission.waypoints[1].lon_e7;
    nav.ground_north_m_s = 0; nav.ground_east_m_s = 50;
    runtime = (ap_mission_runtime_t){0};
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.leg_index == 1);

    /* Do not sequence when pointing backwards near a leg's end. */
    position(&mission, &nav, 1950, 0); nav.ground_north_m_s = -50; nav.ground_east_m_s = 0;
    runtime = (ap_mission_runtime_t){0};
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.leg_index == 0 && output.phase == AP_MISSION_CAPTURE_LEG);

    /* A missed final endpoint is recaptured instead of declaring success. */
    position(&mission, &nav, 5000, 2000); nav.ground_north_m_s = 50;
    runtime = (ap_mission_runtime_t){0};
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.leg_index == 2 && !output.completed && runtime.capture_active);
    nav.lat_e7 = mission.waypoints[3].lat_e7; nav.lon_e7 = mission.waypoints[3].lon_e7;
    runtime.capture_active = false; /* Entry finished; now test terminal acceptance. */
    CHECK(step(&mission, &nav, &runtime, &output));
    CHECK(output.completed && output.phase == AP_MISSION_COMPLETE);

    /* Failed first call must not commit the entry selection or touch output. */
    runtime = (ap_mission_runtime_t){0};
    unsigned char before[sizeof(output)]; memcpy(before, &output, sizeof(output));
    nav.valid = false;
    CHECK(!step(&mission, &nav, &runtime, &output));
    CHECK(!runtime.initialized && memcmp(before, &output, sizeof(output)) == 0);
    nav.valid = true; nav.ground_north_m_s = NAN;
    CHECK(!step(&mission, &nav, &runtime, &output) && !runtime.initialized);
    nav.ground_north_m_s = 50;
    position(&mission, &nav, 60000, 0);
    CHECK(!step(&mission, &nav, &runtime, &output) && !runtime.initialized);
    return 0;
}

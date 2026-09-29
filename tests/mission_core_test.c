#include "flight_common/crc32.h"
#include "autopilot/mission.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

static void put_u16(uint8_t *p, uint16_t value)
{ p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); }

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16); p[3] = (uint8_t)(value >> 24);
}


static void set_nav_position(ap_nav_state_t *nav, float north_m, float east_m)
{
    nav->lat_e7 = 300000000 + (int32_t)lroundf(north_m / 0.0111194927f);
    nav->lon_e7 = (int32_t)lroundf(east_m / 0.009630657f);
}

int main(void)
{
    uint8_t bytes[64] = {0};
    memcpy(bytes, "APM2", 4);
    put_u16(bytes + 4, 2);
    put_u16(bytes + 6, 3);
    const int32_t lat_e7[] = {300000000, 300089932, 300179864};
    const int32_t lon_e7[] = {0, 0, 20769};
    for (int i = 0; i < 3; ++i) {
        uint8_t *p = bytes + 12 + i * 16;
        put_u32(p, (uint32_t)lat_e7[i]);
        put_u32(p + 4, (uint32_t)lon_e7[i]);
        put_u32(p + 8, 91440u);
        put_u16(p + 12, 5620u);
    }
    put_u32(bytes + 60, flight_crc32(bytes, 60));
    ap_mission_t mission = {0};
    CHECK(ap_mission_decode(bytes, sizeof bytes, &mission));
    CHECK(mission.count == 3 && mission.origin_lat_e7 == 300000000);
    CHECK(mission.waypoints[2].lat_e7 == lat_e7[2]);
    CHECK(fabsf(mission.waypoints[2].north_m - 2000.0f) < 0.1f);
    CHECK(fabsf(mission.waypoints[2].east_m - 200.0f) < 0.1f);

    bytes[23] ^= 1u;
    CHECK(!ap_mission_decode(bytes, sizeof bytes, &mission));
    CHECK(mission.count == 3);
    bytes[23] ^= 1u;
    CHECK(!ap_mission_decode(bytes, sizeof bytes - 1, &mission));
    bytes[3] = '1';
    CHECK(!ap_mission_decode(bytes, sizeof bytes, &mission));
    bytes[3] = '2';

    /* APM3 uses the last waypoint word for the type; APM2 still reserves it. */
    bytes[3] = '3'; put_u16(bytes+4,3); put_u16(bytes+42,1);
    put_u32(bytes+60,flight_crc32(bytes,60));
    CHECK(ap_mission_decode(bytes,sizeof(bytes),&mission));
    CHECK(mission.waypoints[1].type==AP_WAYPOINT_FLY_OVER);
    put_u16(bytes+42,2); put_u32(bytes+60,flight_crc32(bytes,60));
    CHECK(!ap_mission_decode(bytes,sizeof(bytes),&mission));
    bytes[3]='2'; put_u16(bytes+4,2); put_u16(bytes+42,1); put_u32(bytes+60,flight_crc32(bytes,60));
    CHECK(!ap_mission_decode(bytes,sizeof(bytes),&mission));
    put_u16(bytes+42,0); put_u32(bytes+60,flight_crc32(bytes,60));
    CHECK(ap_mission_decode(bytes,sizeof(bytes),&mission));

    ap_nav_state_t nav = {0, 0, 56.0f, 0.0f, true};
    set_nav_position(&nav, 200.0f, 50.0f);
    ap_mission_runtime_t runtime = {0};
    runtime.initialized = true; /* This section tests a previously selected leg. */
    ap_mission_output_t output = {0};
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(output.leg_index == 0 && fabsf(output.cross_track_m - 50.0f) < 0.2f);
    CHECK(output.bank_command_rad < 0.0f); /* Right of path: turn left. */
    set_nav_position(&nav, 200.0f, -50.0f);
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(output.bank_command_rad > 0.0f);
    set_nav_position(&nav, 900.0f, 500.0f);
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(runtime.leg_index == 0); /* Do not skip a waypoint from far off track. */
    set_nav_position(&nav, 900.0f, 0.0f);
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(output.phase==AP_MISSION_FLY_BY_TURN && runtime.leg_index==0);
    float turn_n = 0.0f, turn_e = 0.0f, turn_course = 0.0f;
    CHECK(ap_path_sample(&runtime.turn_path, 1, runtime.turn_path.segments[1].length_m,
                         &turn_n, &turn_e, &turn_course));
    set_nav_position(&nav, turn_n, turn_e);
    nav.ground_north_m_s = 0.0f; nav.ground_east_m_s = 56.0f;
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(runtime.leg_index == 1 && output.leg_index == 1);

    /* Crossing the terminal plane far from the endpoint must not complete. */
    set_nav_position(&nav, 2200.0f, 1000.0f);
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(!output.completed);
    set_nav_position(&nav, 3000.0f, 400.0f);
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(!output.completed); /* Passing far beyond the endpoint is insufficient. */
    /* The miss starts a recovery connector; for this unit test, resume the
     * endpoint-acceptance check after returning near the terminal waypoint. */
    runtime.capture_active = false;
    set_nav_position(&nav, 2005.0f, 201.0f);
    CHECK(ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(output.completed && runtime.completed);

    const ap_mission_runtime_t previous_runtime = runtime;
    const ap_mission_output_t previous_output = output;
    nav.valid = false;
    CHECK(!ap_mission_step(&mission, &nav, 4.0f, 0.35f, &runtime, &output));
    CHECK(runtime.completed == previous_runtime.completed);
    CHECK(output.completed == previous_output.completed);

    ap_mission_t dateline = {0};
    dateline.count = 2;
    dateline.waypoints[0] = (ap_waypoint_t){0, 1799999900, 100.0f, 20.0f, 0.0f, 0.0f};
    dateline.waypoints[1] = (ap_waypoint_t){0, -1799999900, 100.0f, 20.0f, 0.0f, 0.0f};
    CHECK(ap_mission_prepare(&dateline));
    CHECK(dateline.waypoints[1].east_m > 2.0f && dateline.waypoints[1].east_m < 3.0f);
    float north = -1.0f, east = -1.0f;
    CHECK(ap_mission_project(&dateline, 0, -1799999900, &north, &east));
    CHECK(fabsf(north) < 0.01f && fabsf(east - dateline.waypoints[1].east_m) < 0.01f);
    return 0;
}

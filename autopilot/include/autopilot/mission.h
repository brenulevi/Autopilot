#ifndef AUTOPILOT_MISSION_H
#define AUTOPILOT_MISSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "autopilot/path.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AP_MISSION_MAX_WAYPOINTS 32u

typedef enum { AP_WAYPOINT_FLY_BY = 0, AP_WAYPOINT_FLY_OVER = 1 } ap_waypoint_type_t;

/* APM3 (and legacy APM2): header (12 bytes), 16-byte waypoint records, CRC32
 * trailer (4 bytes). Waypoints store geodetic degrees times 1e7; local
 * north/east coordinates are derived on load. No raw C structs are stored. */
typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    float altitude_m; /* MSL */
    float airspeed_m_s; /* True airspeed */
    float north_m; /* Derived local position; do not set directly. */
    float east_m;
    ap_waypoint_type_t type; /* Behavior at this waypoint; waypoint[0] is only the route origin. */
} ap_waypoint_t;

typedef struct {
    uint16_t count;
    ap_waypoint_t waypoints[AP_MISSION_MAX_WAYPOINTS];
    int32_t origin_lat_e7; /* First waypoint; populated by prepare/decode. */
    int32_t origin_lon_e7;
    float north_m_per_e7;
    float east_m_per_e7;
} ap_mission_t;

typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    float ground_north_m_s;
    float ground_east_m_s;
    bool valid;
} ap_nav_state_t;

typedef struct {
    uint16_t leg_index; /* Segment from waypoint[index] to waypoint[index+1]. */
    bool completed;
    bool initialized; /* Zero-init: select nearest finite route segment once. */
    int8_t turn_direction; /* Internal recovery latch: -1 left, 0 unset, +1 right. */
    bool capture_active;
    uint8_t capture_segment;
    float capture_progress_m;
    ap_dubins_path_t capture_path;
    float turn_radius_m; /* Fixed on engagement from maximum mission speed and entry groundspeed. */
    bool turn_prepared;
    bool turn_active;
    uint8_t turn_segment;
    float turn_progress_m;
    ap_dubins_path_t turn_path;
} ap_mission_runtime_t;

typedef enum {
    AP_MISSION_CAPTURE_LEG = 0,
    AP_MISSION_TRACK_LEG = 1,
    AP_MISSION_COMPLETE = 2,
    AP_MISSION_FLY_BY_TURN = 3,
    AP_MISSION_FLY_OVER_TURN = 4
} ap_mission_phase_t;

typedef struct {
    float bank_command_rad;
    float altitude_command_m;
    float airspeed_command_m_s;
    float cross_track_m; /* Positive right of the directed path. */
    float north_m; /* Aircraft position in the mission's local frame. */
    float east_m;
    uint16_t leg_index;
    bool completed;
    ap_mission_phase_t phase;
    float target_distance_m; /* Horizontal distance to the active leg's endpoint. */
    ap_waypoint_type_t waypoint_type;
} ap_mission_output_t;

/* Validate geodetic waypoints and derive a short-range local frame anchored at
 * waypoint 0. This is also called by the binary decoder. Failure leaves the
 * mission unchanged. */
bool ap_mission_prepare(ap_mission_t *mission);

/* Decoder accepts CRC-valid APM3 or APM2 with 2..32 waypoints. APM2 maps to fly-by.
 * Failure leaves mission unchanged. The caller owns the input bytes. */
bool ap_mission_decode(const uint8_t *data, size_t length, ap_mission_t *mission);

/* Project a geodetic point into the prepared mission frame. Limited to a
 * short route near waypoint 0; failure leaves the outputs unchanged. */
bool ap_mission_project(const ap_mission_t *mission, int32_t lat_e7,
                        int32_t lon_e7, float *north_m, float *east_m);

/* Geometry shared by runtime guidance and host route previews. Requires a
 * prepared mission. A leg includes its inbound straight and optional fly-by arc.
 * Fly-over starts at the supplied passage position/course and joins leg+1.
 * These functions preserve output on failure. */
bool ap_mission_leg_path(const ap_mission_t *mission, uint16_t leg,
                          float radius_m, ap_dubins_path_t *path);
bool ap_mission_fly_over_path(const ap_mission_t *mission, uint16_t leg,
                               float radius_m, float lookahead_m,
                               float north_m, float east_m, float course_rad,
                               ap_dubins_path_t *path);

/* Dubins entry, fly-by fillets, and fly-over passage for an aircraft already flying.
 * Zero-initialize runtime on engagement: select the nearest finite route segment,
 * allowing earlier waypoints to be skipped. Equal-distance ties prefer the leg
 * most aligned with ground course, then the lower index. Selection happens once.
 * Preserve runtime to resume that leg; reset it to reselect from current position.
 * Radius uses 1.6 * max(entry groundspeed, mission airspeeds)^2 / (g*tan(max_bank)).
 * Keep bank configuration and mission fixed until runtime is reset. Fly-by
 * tangent distances must fit 45% of each adjacent leg; infeasible turns fail.
 * Fly-over requires crossing the inbound plane within the acceptance radius;
 * only then does a Dubins connector begin. Missed fly-overs are recaptured.
 * Fresh calls must be frequent enough to resolve arc progress (less than half a
 * turn between positions). Reset/reselect after a discontinuous position jump.
 * Capture targets the selected leg end's altitude and speed; horizontal capture
 * does not verify them. Positions must stay inside the
 * prepared local projection bounds; "arbitrary start" is not global navigation.
 * Call with fresh geodetic position and ground velocity; no scheduling or I/O is hidden.
 * period_s sets lookahead approximately to period_s * groundspeed.
 * Positive bank means right roll. On completion the caller must switch to a
 * separate safe mode; this function does not generate a loiter or landing.
 * Failure leaves runtime and output unchanged. */
bool ap_mission_step(const ap_mission_t *mission, const ap_nav_state_t *nav,
                     float period_s, float max_bank_rad,
                     ap_mission_runtime_t *runtime, ap_mission_output_t *output);

#ifdef __cplusplus
}
#endif
#endif

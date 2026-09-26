#ifndef AUTOPILOT_PATH_H
#define AUTOPILOT_PATH_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Horizontal N/E path, course clockwise from north. Turn +1 is right, -1 left.
 * This is a geometric curvature constraint, not a wind/roll-dynamics model. */
typedef struct {
    float north_m, east_m, course_rad, length_m;
    int8_t turn; /* -1 left arc, 0 straight, +1 right arc */
} ap_path_segment_t;
typedef struct {
    ap_path_segment_t segments[3];
    float radius_m, length_m;
} ap_dubins_path_t;

/* Shortest of all six planar Dubins families. Allocation-free; false preserves output. */
bool ap_dubins_plan(float north_m, float east_m, float course_rad,
                    float goal_north_m, float goal_east_m, float goal_course_rad,
                    float radius_m, ap_dubins_path_t *path);
bool ap_path_sample(const ap_dubins_path_t *path, uint8_t segment, float along_m,
                    float *north_m, float *east_m, float *course_rad);
#ifdef __cplusplus
}
#endif
#endif

#include "autopilot/mission.h"

#include <math.h>
#include <string.h>

static const double radians_per_degree = 0.017453292519943295;
static const double earth_radius_m = 6371000.0;

static uint16_t read_u16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static uint32_t read_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static int32_t read_i32(const uint8_t *p)
{
    const uint32_t bits = read_u32(p);
    if (bits <= INT32_MAX) return (int32_t)bits;
    return (int32_t)((int64_t)bits - 4294967296LL);
}

static uint32_t crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}

static bool valid_lat_lon(int32_t lat_e7, int32_t lon_e7)
{
    return lat_e7 >= -900000000 && lat_e7 <= 900000000 &&
           lon_e7 >= -1800000000 && lon_e7 <= 1800000000;
}

bool ap_mission_project(const ap_mission_t *mission, int32_t lat_e7,
                        int32_t lon_e7, float *north_m, float *east_m)
{
    if (mission == NULL || north_m == NULL || east_m == NULL ||
        !valid_lat_lon(lat_e7, lon_e7) ||
        !valid_lat_lon(mission->origin_lat_e7, mission->origin_lon_e7) ||
        !isfinite(mission->north_m_per_e7) || mission->north_m_per_e7 <= 0.0f ||
        !isfinite(mission->east_m_per_e7) || mission->east_m_per_e7 < 0.0f)
        return false;
    int64_t lon_delta_e7 = (int64_t)lon_e7 - mission->origin_lon_e7;
    if (lon_delta_e7 > 1800000000LL) lon_delta_e7 -= 3600000000LL;
    if (lon_delta_e7 < -1800000000LL) lon_delta_e7 += 3600000000LL;
    const float north = (float)((int64_t)lat_e7 - mission->origin_lat_e7) *
                        mission->north_m_per_e7;
    const float east = (float)lon_delta_e7 * mission->east_m_per_e7;
    if (!isfinite(north) || !isfinite(east) ||
        fabsf(north) > 50000.0f || fabsf(east) > 50000.0f) return false;
    *north_m = north;
    *east_m = east;
    return true;
}

bool ap_mission_prepare(ap_mission_t *mission)
{
    if (mission == NULL || mission->count < 2 ||
        mission->count > AP_MISSION_MAX_WAYPOINTS) return false;
    ap_mission_t next = *mission;
    next.origin_lat_e7 = next.waypoints[0].lat_e7;
    next.origin_lon_e7 = next.waypoints[0].lon_e7;
    if (!valid_lat_lon(next.origin_lat_e7, next.origin_lon_e7)) return false;
    next.north_m_per_e7 = (float)(earth_radius_m * radians_per_degree * 1e-7);
    next.east_m_per_e7 = (float)(earth_radius_m * radians_per_degree * 1e-7 *
                                 cos((double)next.origin_lat_e7 * 1e-7 * radians_per_degree));
    for (uint16_t i = 0; i < next.count; ++i) {
        ap_waypoint_t *waypoint = &next.waypoints[i];
        if ((waypoint->type != AP_WAYPOINT_FLY_BY && waypoint->type != AP_WAYPOINT_FLY_OVER) ||
            !isfinite(waypoint->altitude_m) ||
            waypoint->altitude_m < -500.0f || waypoint->altitude_m > 10000.0f ||
            !isfinite(waypoint->airspeed_m_s) ||
            waypoint->airspeed_m_s < 5.0f || waypoint->airspeed_m_s > 100.0f ||
            !ap_mission_project(&next, waypoint->lat_e7, waypoint->lon_e7,
                                &waypoint->north_m, &waypoint->east_m) ||
            fabsf(waypoint->north_m) > 20000.0f || fabsf(waypoint->east_m) > 20000.0f)
            return false;
        if (i > 0) {
            const float dn = waypoint->north_m - next.waypoints[i-1].north_m;
            const float de = waypoint->east_m - next.waypoints[i-1].east_m;
            if (hypotf(dn, de) < 1.0f) return false;
        }
    }
    *mission = next;
    return true;
}

bool ap_mission_decode(const uint8_t *data, size_t length, ap_mission_t *mission)
{
    if (data == NULL || mission == NULL || length < 16) return false;
    const uint16_t version = read_u16(data + 4);
    if (!((version == 2 && memcmp(data, "APM2", 4) == 0) ||
          (version == 3 && memcmp(data, "APM3", 4) == 0))) return false;
    const uint16_t count = read_u16(data + 6);
    if (count < 2 || count > AP_MISSION_MAX_WAYPOINTS ||
        length != 16u + 16u * (size_t)count || read_u32(data + 8) != 0 ||
        crc32(data, length - 4) != read_u32(data + length - 4)) return false;

    ap_mission_t next = {0};
    next.count = count;
    for (uint16_t i = 0; i < count; ++i) {
        const uint8_t *p = data + 12u + 16u * (size_t)i;
        const uint16_t type = read_u16(p + 14);
        if (type > 1 || (version == 2 && type != 0)) return false;
        const int32_t lat_e7 = read_i32(p);
        const int32_t lon_e7 = read_i32(p + 4);
        const int32_t alt_cm = read_i32(p + 8);
        const uint16_t speed_cms = read_u16(p + 12);
        if (!valid_lat_lon(lat_e7, lon_e7) ||
            alt_cm < -50000 || alt_cm > 1000000 ||
            speed_cms < 500 || speed_cms > 10000) return false;
        next.waypoints[i] = (ap_waypoint_t){lat_e7, lon_e7,
                                            alt_cm * 0.01f, speed_cms * 0.01f, 0.0f, 0.0f,
                                            (ap_waypoint_type_t)type};
    }
    if (!ap_mission_prepare(&next)) return false;
    *mission = next;
    return true;
}

/* Preserve L1 behavior for targets ahead, but use a full course-error angle
 * for targets behind. sin(pi) alone would command zero bank when flying away.
 * Retain the chosen direction while the target is behind to avoid left/right
 * chatter around 180 degrees. Exact ties choose right; no obstacle avoidance. */
static bool capture_bank(float target_n, float target_e, const ap_nav_state_t *nav,
                         float speed, float lookahead, float max_bank_rad,
                         int8_t *turn_direction, float *bank_command)
{
    const float distance = hypotf(target_n, target_e);
    if (!isfinite(distance) || distance < 0.01f) return false;
    const float vn = nav->ground_north_m_s / speed;
    const float ve = nav->ground_east_m_s / speed;
    const float tn = target_n / distance, te = target_e / distance;
    const float sine = vn * te - ve * tn;
    const float cosine = vn * tn + ve * te;
    float steering;
    if (cosine < 0.0f) {
        if (*turn_direction == 0) *turn_direction = sine < -0.00001f ? -1 : 1;
        steering = (float)*turn_direction;
    } else {
        *turn_direction = 0;
        steering = sine;
    }
    const float lateral_accel = 2.0f * speed * speed / lookahead * steering;
    float bank = atanf(lateral_accel / 9.80665f);
    if (!isfinite(lateral_accel) || !isfinite(bank)) return false;
    if (bank > max_bank_rad) bank = max_bank_rad;
    if (bank < -max_bank_rad) bank = -max_bank_rad;
    *bank_command = bank;
    return true;
}

static bool nearest_leg(const ap_mission_t *mission, const ap_nav_state_t *nav,
                        float north, float east, float speed, uint16_t *index)
{
    float best_distance = INFINITY, best_alignment = -2.0f;
    uint16_t best = 0;
    for (uint16_t i = 0; i < mission->count - 1; ++i) {
        const ap_waypoint_t *a = &mission->waypoints[i], *b = &mission->waypoints[i + 1];
        const float dn = b->north_m - a->north_m, de = b->east_m - a->east_m;
        const float length = hypotf(dn, de);
        if (!isfinite(length) || length < 1.0f) return false;
        const float un = dn / length, ue = de / length;
        const float along = (north - a->north_m) * un + (east - a->east_m) * ue;
        const float clamped = fmaxf(0.0f, fminf(length, along));
        const float distance = hypotf(north - a->north_m - clamped * un,
                                      east - a->east_m - clamped * ue);
        const float alignment = (nav->ground_north_m_s / speed) * un +
                                 (nav->ground_east_m_s / speed) * ue;
        if (!isfinite(distance) || !isfinite(along)) return false;
        /* One-centimeter tolerance avoids unstable ties from geodetic rounding. */
        if (distance < best_distance - 0.01f ||
            (fabsf(distance - best_distance) <= 0.01f && alignment > best_alignment)) {
            best_distance = distance; best_alignment = alignment; best = i;
        }
    }
    *index = best;
    return true;
}

/* A tangent fillet uses d = R*tan(change_of_course/2) on both legs. Reserving
 * at most 45% per corner leaves a straight section between neighboring turns. */
static bool fly_by_path(const ap_mission_t *mission, uint16_t leg, float radius,
                        ap_dubins_path_t *path)
{
    const ap_waypoint_t *a = &mission->waypoints[leg], *b = a + 1;
    const float length = hypotf(b->north_m-a->north_m, b->east_m-a->east_m);
    if (!isfinite(length) || length < 1) return false;
    const float course = atan2f(b->east_m-a->east_m, b->north_m-a->north_m);
    ap_dubins_path_t result = {0}; result.radius_m = radius;
    result.segments[0] = (ap_path_segment_t){a->north_m,a->east_m,course,length,0};
    result.segments[1] = (ap_path_segment_t){b->north_m,b->east_m,course,0,0};
    result.segments[2] = result.segments[1]; result.length_m = length;
    if (leg < mission->count-2 && b->type == AP_WAYPOINT_FLY_BY) {
        const ap_waypoint_t *c = b + 1;
        const float outgoing = hypotf(c->north_m-b->north_m,c->east_m-b->east_m);
        const float out_course = atan2f(c->east_m-b->east_m,c->north_m-b->north_m);
        const float angle = remainderf(out_course-course,6.28318530718f);
        if (!isfinite(outgoing) || outgoing < 1 || fabsf(angle) > 3.14059f) return false;
        if (fabsf(angle) > 0.001f) {
            const float tangent = radius*tanf(0.5f*fabsf(angle));
            if (!isfinite(tangent) || tangent > 0.45f*fminf(length,outgoing)) return false;
            result.segments[0].length_m = length-tangent;
            result.segments[1] = (ap_path_segment_t){b->north_m-tangent*cosf(course),
                b->east_m-tangent*sinf(course),course,radius*fabsf(angle),angle>0 ? 1 : -1};
            float n,e,heading;
            if (!ap_path_sample(&result,1,result.segments[1].length_m,&n,&e,&heading)) return false;
            result.segments[2] = (ap_path_segment_t){n,e,heading,0,0};
            result.length_m = result.segments[0].length_m+result.segments[1].length_m;
        }
    }
    *path = result; return true;
}

static bool plan_entry(const ap_mission_t *mission, const ap_nav_state_t *nav,
                       float north, float east, float speed, float lookahead,
                       float bank_limit, ap_mission_runtime_t *runtime)
{
    const ap_waypoint_t *a=&mission->waypoints[runtime->leg_index], *b=a+1;
    const float length=hypotf(b->north_m-a->north_m,b->east_m-a->east_m);
    const float un=(b->north_m-a->north_m)/length, ue=(b->east_m-a->east_m)/length;
    const float along=(north-a->north_m)*un+(east-a->east_m)*ue;
    const float cross=un*(east-a->east_m)-ue*(north-a->north_m);
    const float alignment=(nav->ground_north_m_s*un+nav->ground_east_m_s*ue)/speed;
    ap_dubins_path_t route;
    if (!fly_by_path(mission,runtime->leg_index,runtime->turn_radius_m,&route)) return false;
    const float straight = route.segments[0].length_m;
    const float corridor=fmaxf(20.0f,fminf(0.5f*lookahead,100.0f));
    if (fabsf(cross)<=corridor && along>=0 && along<=straight && alignment>=0.8660254f) return true;
    /* Margin for roll response and modest wind. This radius is a fixed geometry
     * proxy from entry groundspeed; it is not a wind-optimal flight trajectory. */
    const float radius=runtime->turn_radius_m;
    (void)bank_limit;
    if (!isfinite(radius) || radius<1 || !isfinite(length)) return false;
    const float goal_along=fmaxf(0,fminf(straight-fminf(lookahead,0.25f*straight),along+2*radius));
    if (!ap_dubins_plan(north,east,atan2f(nav->ground_east_m_s,nav->ground_north_m_s),
                        a->north_m+goal_along*un,a->east_m+goal_along*ue,atan2f(ue,un),radius,
                        &runtime->capture_path)) return false;
    runtime->capture_active=true; runtime->capture_segment=0; runtime->capture_progress_m=0;
    return true;
}

static bool follow_path(float north, float east, const ap_nav_state_t *nav,
                         float speed, float lookahead, float bank_limit,
                         const ap_dubins_path_t *path, bool *active, uint8_t *segment,
                         float *progress_m, int8_t *turn_direction, float *bank)
{
    const float radius=path->radius_m;
    if (!isfinite(radius) || radius<1 || !isfinite(*progress_m)) return false;
    while (*segment<3) {
        const uint8_t index=*segment;
        const ap_path_segment_t *s=&path->segments[index];
        float end_n,end_e,end_course;
        if (!ap_path_sample(path,index,s->length_m,&end_n,&end_e,&end_course)) return false;
        float progress;
        if (s->turn==0) {
            progress=(north-s->north_m)*cosf(s->course_rad)+(east-s->east_m)*sinf(s->course_rad);
        } else {
            const float cn=s->north_m-s->turn*radius*sinf(s->course_rad);
            const float ce=s->east_m+s->turn*radius*cosf(s->course_rad);
            const float angle=atan2f(east-ce,north-cn);
            const float expected=s->course_rad-s->turn*1.57079632679f+
                                  s->turn*(*progress_m)/radius;
            progress=*progress_m+
                      s->turn*radius*remainderf(angle-expected,6.28318530718f);
        }
        if (!isfinite(progress)) return false;
        const float distance=hypotf(north-end_n,east-end_e);
        if (s->length_m<0.01f ||
            (progress>=s->length_m && distance<=fmaxf(100,0.2f*radius)) ||
            (distance<=20 && progress>=s->length_m-lookahead)) {
            ++*segment; *progress_m=0; *turn_direction=0;
            continue;
        }
        *progress_m=fmaxf(0,progress);
        uint8_t target_segment=index;
        float target_along=*progress_m+lookahead;
        while (target_segment<2 && target_along>path->segments[target_segment].length_m) {
            target_along-=path->segments[target_segment].length_m; ++target_segment;
        }
        float tn,te,tc;
        if (!ap_path_sample(path,target_segment,target_along,&tn,&te,&tc)) return false;
        if (target_segment==2 && target_along>path->segments[2].length_m) {
            const float extension=target_along-path->segments[2].length_m;
            tn+=extension*cosf(tc); te+=extension*sinf(tc);
        }
        return capture_bank(tn-north,te-east,nav,speed,lookahead,bank_limit,turn_direction,bank);
    }
    *active=false; *turn_direction=0;
    return true;
}

bool ap_mission_step(const ap_mission_t *mission, const ap_nav_state_t *nav,
                     float period_s, float max_bank_rad,
                     ap_mission_runtime_t *runtime, ap_mission_output_t *output)
{
    if (mission == NULL || nav == NULL || runtime == NULL || output == NULL ||
        !nav->valid || mission->count < 2 || mission->count > AP_MISSION_MAX_WAYPOINTS ||
        runtime->leg_index >= mission->count - 1 ||
        runtime->turn_direction < -1 || runtime->turn_direction > 1 ||
        runtime->capture_segment > 3 || runtime->turn_segment > 3 ||
        (!runtime->initialized && (runtime->leg_index != 0 || runtime->completed)) ||
        !isfinite(period_s) || period_s <= 0.0f ||
        !isfinite(max_bank_rad) || max_bank_rad <= 0.0f || max_bank_rad >= 1.5707963f ||
        !isfinite(nav->ground_north_m_s) || !isfinite(nav->ground_east_m_s)) return false;
    float north_m = 0.0f, east_m = 0.0f;
    if (!ap_mission_project(mission, nav->lat_e7, nav->lon_e7,
                            &north_m, &east_m)) return false;
    const float speed = hypotf(nav->ground_north_m_s, nav->ground_east_m_s);
    if (!isfinite(speed) || speed < 5.0f) return false;
    const float lookahead = fmaxf(30.0f, period_s * speed);
    if (!isfinite(lookahead)) return false;

    ap_mission_runtime_t next = *runtime;
    ap_mission_output_t result = {0};
    result.north_m = north_m;
    result.east_m = east_m;
    const bool engaging = !next.initialized;
    if (engaging) {
        if (!nearest_leg(mission, nav, north_m, east_m, speed, &next.leg_index)) return false;
        next.initialized = true;
        next.turn_direction = 0;
    }
    if (next.turn_radius_m == 0) {
        float planning_speed = speed;
        for (uint16_t i=0;i<mission->count;++i) {
            const ap_waypoint_t *w=&mission->waypoints[i];
            if (!isfinite(w->airspeed_m_s) || w->airspeed_m_s<5 || w->airspeed_m_s>100 ||
                (w->type!=AP_WAYPOINT_FLY_BY && w->type!=AP_WAYPOINT_FLY_OVER)) return false;
            planning_speed=fmaxf(planning_speed,w->airspeed_m_s);
        }
        next.turn_radius_m=1.6f*planning_speed*planning_speed/(9.80665f*tanf(max_bank_rad));
        if (!isfinite(next.turn_radius_m) || next.turn_radius_m<1) return false;
        for (uint16_t i=next.leg_index;i<mission->count-1;++i) {
            ap_dubins_path_t check;
            if (!fly_by_path(mission,i,next.turn_radius_m,&check)) return false;
        }
    }
    if (!isfinite(next.turn_radius_m) || next.turn_radius_m<1) return false;
    if (engaging && !plan_entry(mission,nav,north_m,east_m,speed,lookahead,max_bank_rad,&next)) return false;
    if (next.completed) {
        result.completed = true;
        result.phase = AP_MISSION_COMPLETE;
        result.leg_index = next.leg_index;
        result.altitude_command_m = mission->waypoints[mission->count - 1].altitude_m;
        result.airspeed_command_m_s = mission->waypoints[mission->count - 1].airspeed_m_s;
        result.waypoint_type = mission->waypoints[mission->count - 1].type;
        *output = result;
        return true;
    }

    const float acceptance = fmaxf(20.0f, fminf(0.25f * lookahead, 50.0f));
    if (next.capture_active) {
        if (!follow_path(north_m,east_m,nav,speed,lookahead,max_bank_rad,&next.capture_path,
                         &next.capture_active,&next.capture_segment,&next.capture_progress_m,
                         &next.turn_direction,&result.bank_command_rad)) return false;
        if (next.capture_active) {
            const ap_waypoint_t *a=&mission->waypoints[next.leg_index], *b=a+1;
            const float length=hypotf(b->north_m-a->north_m,b->east_m-a->east_m);
            result.phase=AP_MISSION_CAPTURE_LEG; result.leg_index=next.leg_index;
            result.target_distance_m=hypotf(b->north_m-north_m,b->east_m-east_m);
            result.cross_track_m=((b->north_m-a->north_m)*(east_m-a->east_m)-
                                  (b->east_m-a->east_m)*(north_m-a->north_m))/length;
            result.altitude_command_m=b->altitude_m; result.airspeed_command_m_s=b->airspeed_m_s;
            result.waypoint_type=b->type;
            if (!isfinite(result.cross_track_m) || !isfinite(b->altitude_m) ||
                !isfinite(b->airspeed_m_s) || b->airspeed_m_s<5 || b->airspeed_m_s>100) return false;
            *runtime=next; *output=result; return true;
        }
    }
    for (uint16_t attempt = 0; attempt < mission->count - 1; ++attempt) {
        const ap_waypoint_t *a = &mission->waypoints[next.leg_index];
        const ap_waypoint_t *b = &mission->waypoints[next.leg_index + 1];
        const float dn = b->north_m - a->north_m;
        const float de = b->east_m - a->east_m;
        const float length = hypotf(dn, de);
        if (!isfinite(length) || length < 1.0f) return false;
        const float un = dn / length, ue = de / length;
        const float pn = north_m - a->north_m;
        const float pe = east_m - a->east_m;
        const float along = pn * un + pe * ue;
        const float cross_track = un * pe - ue * pn;
        if (!isfinite(along) || !isfinite(cross_track) ||
            !isfinite(b->altitude_m) || !isfinite(b->airspeed_m_s) ||
            b->airspeed_m_s < 5.0f || b->airspeed_m_s > 100.0f) return false;
        const bool last = next.leg_index == mission->count - 2;
        const float switch_corridor = fmaxf(20.0f, fminf(0.5f * lookahead, 100.0f));
        const float alignment = (nav->ground_north_m_s / speed) * un +
                                 (nav->ground_east_m_s / speed) * ue;
        const bool tracking = fabsf(cross_track) <= switch_corridor && alignment >= 0.5f &&
                               along >= -acceptance && along <= length + acceptance;
        result.phase = tracking ? AP_MISSION_TRACK_LEG : AP_MISSION_CAPTURE_LEG;
        const float endpoint_distance = hypotf(b->north_m - north_m, b->east_m - east_m);
        if (!isfinite(endpoint_distance)) return false;
        /* Small plane tolerance only absorbs local-projection float roundoff.
         * The acceptance circle alone must never initiate a fly-over turn. */
        const bool passed = endpoint_distance<=acceptance && along>=length-0.05f;
        if (last && endpoint_distance <= acceptance && (b->type==AP_WAYPOINT_FLY_BY || passed)) {
            next.completed = true;
            result.completed = true;
            result.phase = AP_MISSION_COMPLETE;
            result.target_distance_m = endpoint_distance;
            result.leg_index = next.leg_index;
            result.altitude_command_m = b->altitude_m;
            result.airspeed_command_m_s = b->airspeed_m_s;
            result.waypoint_type = b->type;
            *runtime = next;
            *output = result;
            return true;
        }
        if (!last && !next.turn_prepared && b->type==AP_WAYPOINT_FLY_BY) {
            if (!fly_by_path(mission,next.leg_index,next.turn_radius_m,&next.turn_path)) return false;
            next.turn_prepared=true; next.turn_active=true; next.turn_segment=0; next.turn_progress_m=0;
        }
        if (!last && !next.turn_active && b->type==AP_WAYPOINT_FLY_OVER && passed) {
            ap_dubins_path_t outgoing;
            if (!fly_by_path(mission,next.leg_index+1,next.turn_radius_m,&outgoing)) return false;
            const float straight=outgoing.segments[0].length_m;
            const float goal=fminf(2*next.turn_radius_m,straight-fminf(lookahead,0.25f*straight));
            const float course=outgoing.segments[0].course_rad;
            if (!ap_dubins_plan(north_m,east_m,atan2f(nav->ground_east_m_s,nav->ground_north_m_s),
                                b->north_m+goal*cosf(course),b->east_m+goal*sinf(course),course,
                                next.turn_radius_m,&next.turn_path)) return false;
            next.turn_prepared=true; next.turn_active=true; next.turn_segment=0; next.turn_progress_m=0;
            next.turn_direction=0;
        }
        if (next.turn_active) {
            if (!follow_path(north_m,east_m,nav,speed,lookahead,max_bank_rad,&next.turn_path,
                             &next.turn_active,&next.turn_segment,&next.turn_progress_m,
                             &next.turn_direction,&result.bank_command_rad)) return false;
            if (!next.turn_active) {
                ++next.leg_index; next.turn_prepared=false; next.turn_segment=0;
                next.turn_progress_m=0; next.turn_direction=0;
                continue;
            }
            if (b->type==AP_WAYPOINT_FLY_OVER) result.phase=AP_MISSION_FLY_OVER_TURN;
            else if (next.turn_segment>0 || next.turn_progress_m+lookahead>=next.turn_path.segments[0].length_m)
                result.phase=AP_MISSION_FLY_BY_TURN;
        } else if (along>length+acceptance || (along>length && fabsf(cross_track)>acceptance)) {
            /* A missed passage must be approached again, not accepted merely
             * because the aircraft is beyond the waypoint's infinite plane. */
            if (!ap_dubins_plan(north_m,east_m,atan2f(nav->ground_east_m_s,nav->ground_north_m_s),
                                b->north_m,b->east_m,atan2f(ue,un),next.turn_radius_m,&next.capture_path)) return false;
            next.capture_active=true; next.capture_segment=0; next.capture_progress_m=0; next.turn_direction=0;
            if (!follow_path(north_m,east_m,nav,speed,lookahead,max_bank_rad,&next.capture_path,
                             &next.capture_active,&next.capture_segment,&next.capture_progress_m,
                             &next.turn_direction,&result.bank_command_rad)) return false;
            result.phase=AP_MISSION_CAPTURE_LEG;
        } else {
            const float ahead = sqrtf(fmaxf(0.0f, lookahead * lookahead - cross_track * cross_track));
            const float target_along = fminf(length, fmaxf(0.0f, along + ahead));
            const float target_n = a->north_m + un * target_along - north_m;
            const float target_e = a->east_m + ue * target_along - east_m;
            if (!capture_bank(target_n, target_e, nav, speed, lookahead, max_bank_rad,
                              &next.turn_direction, &result.bank_command_rad)) return false;
        }
        result.target_distance_m = endpoint_distance;
        result.altitude_command_m = b->altitude_m;
        result.airspeed_command_m_s = b->airspeed_m_s;
        result.cross_track_m = cross_track;
        result.leg_index = next.leg_index;
        result.waypoint_type = b->type;
        *runtime = next;
        *output = result;
        return true;
    }
    return false;
}

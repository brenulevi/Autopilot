/* Small stdio bridge: APM on stdin, preview JSON on stdout. No duplicate planner. */
#include "autopilot/mission.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

static const double radians = 0.017453292519943295;
typedef struct { ap_dubins_path_t path; const char *kind; unsigned leg; float from; } piece_t;
static piece_t pieces[100];
static unsigned count;
static int fail(const char *message) { fprintf(stderr, "%s\n", message); return 1; }
static bool number(const char *s, double *value) {
    char *end; *value = strtod(s, &end);
    return end != s && *end == '\0' && isfinite(*value);
}
static void add(ap_dubins_path_t path, const char *kind, unsigned leg, float from) {
    pieces[count++] = (piece_t){path,kind,leg,from};
}
static bool endpoint(const ap_dubins_path_t *p, float *n, float *e) {
    float course;
    for (int i = 2; i >= 0; --i)
        if (p->segments[i].length_m > 0)
            return ap_path_sample(p, (uint8_t)i, p->segments[i].length_m, n, e, &course);
    *n = p->segments[0].north_m; *e = p->segments[0].east_m; return true;
}
int main(int argc, char **argv) {
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    uint8_t bytes[16 + 16 * AP_MISSION_MAX_WAYPOINTS + 1];
    const size_t length = fread(bytes, 1, sizeof bytes, stdin);
    ap_mission_t mission;
    if (!ap_mission_decode(bytes, length, &mission)) return fail("C decoder rejected mission format, coordinates, spacing, or CRC.");
    if (argc == 2 && strcmp(argv[1], "--validate") == 0) { puts("{\"valid\":true}"); return 0; }
    if (argc != 7) return fail("Expected latitude longitude course ground-speed bank-limit L1-period.");
    double a[6];
    for (unsigned i=0; i<6; ++i) if (!number(argv[i+1], &a[i])) return fail("Invalid preview parameter.");
    if (fabs(a[0])>90 || fabs(a[1])>180 || a[2]<0 || a[2]>=360 || a[3]<5 || a[3]>150 ||
        a[4]<1 || a[4]>60 || a[5]<1 || a[5]>30) return fail("Preview parameter outside supported range.");
    if (mission.east_m_per_e7 < 1e-6f) return fail("Map preview is unavailable at the poles.");
    ap_nav_state_t nav = {(int32_t)llround(a[0]*1e7), (int32_t)llround(a[1]*1e7),
        (float)(a[3]*cos(a[2]*radians)), (float)(a[3]*sin(a[2]*radians)), true};
    ap_mission_runtime_t runtime = {0}; ap_mission_output_t output;
    if (!ap_mission_step(&mission, &nav, (float)a[5], (float)(a[4]*radians), &runtime, &output))
        return fail("Route cannot be planned: check start bounds and fly-by spacing. Reduce speed, increase bank limit, move waypoints apart, or use fly-over.");
    float n,e;
    if (!ap_mission_project(&mission,nav.lat_e7,nav.lon_e7,&n,&e)) return fail("Start outside local projection.");
    if (runtime.capture_active) {
        add(runtime.capture_path,"entry",runtime.leg_index,0);
        if (!endpoint(&runtime.capture_path,&n,&e)) return fail("Cannot sample entry.");
    }
    const float radius=runtime.turn_radius_m;
    for (unsigned leg=runtime.leg_index; leg+1<mission.count; ++leg) {
        ap_dubins_path_t route;
        if (!ap_mission_leg_path(&mission,(uint16_t)leg,radius,&route)) return fail("Infeasible fly-by corner.");
        const ap_path_segment_t *s=&route.segments[0];
        const float from=fmaxf(0,fminf(s->length_m,(n-s->north_m)*cosf(s->course_rad)+(e-s->east_m)*sinf(s->course_rad)));
        add(route,route.segments[1].length_m>0 ? "fly_by" : "straight",leg,from);
        if (!endpoint(&route,&n,&e)) return fail("Cannot sample route.");
        const ap_waypoint_t *b=&mission.waypoints[leg+1];
        if (leg+2<mission.count && b->type==AP_WAYPOINT_FLY_OVER) {
            ap_dubins_path_t turn;
            /* Preflight assumption: ideal inbound course and commanded speed at passage. */
            const float lookahead=fmaxf(30,(float)a[5]*b->airspeed_m_s);
            if (!ap_mission_fly_over_path(&mission,(uint16_t)leg,radius,lookahead,n,e,s->course_rad,&turn))
                return fail("Cannot plan fly-over connector.");
            add(turn,"fly_over",leg,0);
            if (!endpoint(&turn,&n,&e)) return fail("Cannot sample fly-over.");
        }
    }
    float total=0; for(unsigned i=0;i<count;++i) total+=pieces[i].path.length_m-pieces[i].from;
    printf("{\"radius_m\":%.3f,\"length_m\":%.3f,\"selected_leg\":%u,\"paths\":[",radius,total,runtime.leg_index);
    for(unsigned i=0;i<count;++i) {
        const piece_t *p=&pieces[i];
        printf("%s{\"kind\":\"%s\",\"leg\":%u,\"points\":[",i?",":"",p->kind,p->leg);
        unsigned emitted=0;
        for(uint8_t j=0;j<3;++j) {
            const float begin=j==0 ? p->from : 0;
            const float distance=p->path.segments[j].length_m-begin;
            if(distance<0.001f) continue;
            const unsigned samples=(unsigned)fminf(2000,fmaxf(1,ceilf(distance/20)));
            for(unsigned k=0;k<=samples;++k) {
                float pn,pe,course;
                if(!ap_path_sample(&p->path,j,begin+distance*k/samples,&pn,&pe,&course)) return fail("Invalid path sample.");
                const double lat=mission.origin_lat_e7*1e-7+pn/mission.north_m_per_e7*1e-7;
                double lon=mission.origin_lon_e7*1e-7+pe/mission.east_m_per_e7*1e-7;
                lon=fmod(lon+540,360)-180;
                printf("%s[%.8f,%.8f]",emitted++?",":"",lat,lon);
            }
        }
        printf("]}");
    }
    puts("]}"); return 0;
}

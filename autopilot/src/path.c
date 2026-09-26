#include "autopilot/path.h"
#include <math.h>
#include <stddef.h>

static const double pi = 3.14159265358979323846;
static double wrap(double angle)
{
    double result = fmod(angle, 2.0 * pi);
    return result < 0 ? result + 2.0 * pi : result;
}
static void advance(double *n, double *e, double *course, double distance, int turn, double radius)
{
    if (!turn) { *n += distance * cos(*course); *e += distance * sin(*course); }
    else {
        double end = *course + turn * distance / radius;
        *n += radius / turn * (sin(end) - sin(*course));
        *e += radius / turn * (cos(*course) - cos(end));
        *course = end;
    }
    *course = wrap(*course);
}

bool ap_dubins_plan(float north_m, float east_m, float course_rad,
                    float goal_north_m, float goal_east_m, float goal_course_rad,
                    float radius_m, ap_dubins_path_t *path)
{
    if (!path || !isfinite(north_m) || !isfinite(east_m) || !isfinite(course_rad) ||
        !isfinite(goal_north_m) || !isfinite(goal_east_m) || !isfinite(goal_course_rad) ||
        !isfinite(radius_m) || radius_m < 1.0f) return false;
    const double dn = (double)goal_north_m - north_m, de = (double)goal_east_m - east_m;
    const double direction = atan2(de, dn), d = hypot(dn, de) / radius_m;
    const double a = wrap(course_rad - direction), b = wrap(goal_course_rad - direction);
    const double sa = sin(a), sb = sin(b), ca = cos(a), cb = cos(b), cab = cos(a - b);
    /* Standard positive-angle families in N/E are RSR, LSL, RSL, LSR, LRL, RLR. */
    const int8_t turns[6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};
    double best = INFINITY, lengths[3] = {0}; int selected = -1;
    if (hypot(dn,de)<1e-6 && fabs(remainder((double)course_rad-goal_course_rad,2*pi))<1e-6) {
        best=0; selected=0;
    }
    for (int family = 0; family < 6; ++family) {
        double t = 0, p = 0, q = 0, value = 0, angle = 0;
        switch (family) {
        case 0:
            value = 2 + d*d - 2*cab + 2*d*(sa-sb);
            if (value < -1e-10) continue;
            p = sqrt(fmax(0,value)); angle = atan2(cb-ca,d+sa-sb);
            t = wrap(-a+angle); q = wrap(b-angle); break;
        case 1:
            value = 2 + d*d - 2*cab + 2*d*(-sa+sb);
            if (value < -1e-10) continue;
            p = sqrt(fmax(0,value)); angle = atan2(ca-cb,d-sa+sb);
            t = wrap(a-angle); q = wrap(-b+angle); break;
        case 2:
            value = -2 + d*d + 2*cab + 2*d*(sa+sb);
            if (value < -1e-10) continue;
            p = sqrt(fmax(0,value)); angle = atan2(-ca-cb,d+sa+sb)-atan2(-2,p);
            t = wrap(-a+angle); q = wrap(-b+angle); break;
        case 3:
            value = d*d - 2 + 2*cab - 2*d*(sa+sb);
            if (value < -1e-10) continue;
            p = sqrt(fmax(0,value)); angle = atan2(ca+cb,d-sa-sb)-atan2(2,p);
            t = wrap(a-angle); q = wrap(b-angle); break;
        case 4:
            value = (6-d*d+2*cab+2*d*(sa-sb))/8;
            if (fabs(value) > 1) continue;
            p = wrap(2*pi-acos(value)); t = wrap(a-atan2(ca-cb,d-sa+sb)+p/2);
            q = wrap(a-b-t+p); break;
        default:
            value = (6-d*d+2*cab+2*d*(-sa+sb))/8;
            if (fabs(value) > 1) continue;
            p = wrap(2*pi-acos(value)); t = wrap(-a-atan2(ca-cb,d+sa-sb)+p/2);
            q = wrap(b-a-t+p); break;
        }
        if (t+p+q < best) { best=t+p+q; selected=family; lengths[0]=t; lengths[1]=p; lengths[2]=q; }
    }
    if (selected < 0 || !isfinite(best * radius_m) || best * radius_m > 1000000) return false;
    ap_dubins_path_t result = {0}; result.radius_m = radius_m; result.length_m = (float)(best * radius_m);
    double n = north_m, e = east_m, course = course_rad;
    for (int i = 0; i < 3; ++i) {
        result.segments[i] = (ap_path_segment_t){(float)n,(float)e,(float)wrap(course),
                                                (float)(lengths[i]*radius_m),turns[selected][i]};
        advance(&n,&e,&course,lengths[i]*radius_m,turns[selected][i],radius_m);
    }
    if (hypot(n-goal_north_m,e-goal_east_m) > 0.1 ||
        fabs(remainder(course-goal_course_rad,2*pi)) > 0.00001) return false;
    *path=result; return true;
}

bool ap_path_sample(const ap_dubins_path_t *path, uint8_t segment, float along_m,
                    float *north_m, float *east_m, float *course_rad)
{
    if (!path || !north_m || !east_m || !course_rad || segment >= 3 ||
        !isfinite(path->radius_m) || path->radius_m < 1 || !isfinite(along_m)) return false;
    const ap_path_segment_t *s=&path->segments[segment];
    if (!isfinite(s->north_m) || !isfinite(s->east_m) || !isfinite(s->course_rad) ||
        !isfinite(s->length_m) || s->length_m < 0 || s->turn < -1 || s->turn > 1) return false;
    double n=s->north_m,e=s->east_m,course=s->course_rad;
    advance(&n,&e,&course,fmax(0,fmin(along_m,s->length_m)),s->turn,path->radius_m);
    if (!isfinite(n) || !isfinite(e)) return false;
    *north_m=(float)n; *east_m=(float)e; *course_rad=(float)course; return true;
}

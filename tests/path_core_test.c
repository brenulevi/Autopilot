#include "autopilot/path.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); return 1; } } while(0)

int main(void)
{
    ap_dubins_path_t path={0};
    CHECK(ap_dubins_plan(0,0,0,1000,0,0,100,&path));
    CHECK(fabsf(path.length_m-1000)<.01f);
    CHECK(ap_dubins_plan(0,0,0,0,0,0,100,&path) && path.length_m<.01f);
    CHECK(ap_dubins_plan(200,-100,.7f,200,-100,.7f,100,&path) && path.length_m<.01f);
    unsigned families=0;
    for(int x=-3;x<=3;++x) for(int y=-3;y<=3;++y) for(int h=0;h<8;++h) {
        const float heading=h*.78539816339f;
        CHECK(ap_dubins_plan(0,0,0,x*120.0f,y*120.0f,heading,100,&path));
        CHECK(isfinite(path.length_m) && path.length_m>=hypotf(x*120.0f,y*120.0f)-.01f);
        /* Integrate curvature independently of the planner/sample formulas. */
        double n=0,e=0,course=0,length=0;
        for(int s=0;s<3;++s) {
            const ap_path_segment_t *segment=&path.segments[s];
            CHECK(hypot(n-segment->north_m,e-segment->east_m)<.03);
            const int count=(int)ceil(segment->length_m/.5f);
            const double ds=count ? segment->length_m/count : 0;
            for(int k=0;k<count;++k) {
                double next=course+segment->turn*ds/path.radius_m;
                n+=ds*cos((course+next)/2); e+=ds*sin((course+next)/2); course=next;
            }
            length+=segment->length_m;
        }
        CHECK(hypot(n-x*120.0,e-y*120.0)<.04);
        CHECK(fabs(remainder(course-heading,6.28318530718))<.00001);
        CHECK(fabs(length-path.length_m)<.001);
        float sn,se,sc;
        CHECK(ap_path_sample(&path,2,path.segments[2].length_m,&sn,&se,&sc));
        CHECK(hypot(sn-x*120.0,se-y*120.0)<.04);
        unsigned family=path.segments[1].turn==0 ?
            (path.segments[0].turn>0 ? 0u:2u)+(path.segments[2].turn>0 ? 0u:1u) :
            (path.segments[0].turn>0 ? 4u:5u);
        families|=1u<<family;
    }
    CHECK(families==63); /* Grid includes every winning path family. */
    unsigned char previous[sizeof(path)]; memcpy(previous,&path,sizeof(path));
    CHECK(!ap_dubins_plan(NAN,0,0,100,0,0,100,&path));
    CHECK(!ap_dubins_plan(0,0,0,100,0,0,0,&path));
    CHECK(memcmp(previous,&path,sizeof(path))==0);
    float n=123,e=456,c=789;
    CHECK(!ap_path_sample(&path,3,0,&n,&e,&c) && n==123 && e==456 && c==789);
    return 0;
}

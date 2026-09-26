#include "flight_io/rc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

int main(void)
{
    fio_rc_config_t config = {
        {0,1000,1400,2000,false}, {1,1000,1500,2000,true},
        {3,1000,1500,2000,false}, {2,1000,1500,2000,false},
        {4,1000,1500,2000,false}
    };
    fio_ibus_frame_t frame = {{1700,2000,1500,1250,1000}, 123, false};
    flight_rc_sample_t output = {0};
    CHECK(fio_rc_config_valid(&config));
    CHECK(fio_rc_map(&config, &frame, true, &output));
    CHECK(output.valid && !output.request_auto && output.received_at_ms == 123);
    CHECK(output.controls.aileron == .5f && output.controls.elevator == -1.0f);
    CHECK(output.controls.rudder == -.5f && output.controls.throttle == .5f);
    frame.channels[0] = 1200;
    CHECK(fio_rc_map(&config, &frame, true, &output) && output.controls.aileron == -.5f);
    frame.channels[0] = 1000; frame.channels[2] = 1000; frame.channels[4] = 2000;
    CHECK(fio_rc_map(&config, &frame, true, &output));
    CHECK(output.controls.aileron == -1 && output.controls.throttle == 0 && output.request_auto);
    config.throttle.reversed = true;
    CHECK(fio_rc_map(&config, &frame, true, &output) && output.controls.throttle == 1);
    frame.channels[4] = 1500;
    CHECK(fio_rc_map(&config, &frame, true, &output) && !output.valid);
    frame.channels[4] = 2000; config.mode.reversed = true;
    CHECK(fio_rc_map(&config, &frame, true, &output) && output.valid && !output.request_auto);
    frame.failsafe = true;
    CHECK(fio_rc_map(&config, &frame, true, &output) && !output.valid);
    frame.failsafe = false;
    CHECK(fio_rc_map(&config, &frame, false, &output) && !output.valid);
    frame.channels[2] = 999;
    CHECK(fio_rc_map(&config, &frame, true, &output) && !output.valid);
    frame.channels[2] = 2001;
    CHECK(fio_rc_map(&config, &frame, true, &output) && !output.valid);
    unsigned char previous[sizeof(output)]; memcpy(previous, &output, sizeof(output));
    config.mode.channel = config.throttle.channel;
    CHECK(!fio_rc_map(&config, &frame, true, &output));
    CHECK(memcmp(previous, &output, sizeof(output)) == 0);
    config.mode.channel = 14;
    CHECK(!fio_rc_config_valid(&config));
    config.mode.channel = 4; config.aileron.center = config.aileron.minimum;
    CHECK(!fio_rc_config_valid(&config));
    CHECK(!fio_rc_config_valid(NULL));
    return 0;
}

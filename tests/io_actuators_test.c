#include "flight_io/actuators.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

int main(void)
{
    fio_actuator_config_t config = {4, {
        {1,1,0,0,0,1000,1500,2000}, {-1,1,0,0,0,1000,1500,2000},
        {0,0,-1,0,0,1100,1400,1900}, {0,0,0,1,0,1000,1000,2000}
    }};
    flight_controls_t controls = {.8f,.6f,-.5f,.25f};
    fio_actuator_output_t output = {0};
    CHECK(fio_actuators_map(&config, &controls, &output));
    CHECK(output.count == 4 && output.pulse_us[0] == 2000 && output.pulse_us[1] == 1400);
    CHECK(output.pulse_us[2] == 1650 && output.pulse_us[3] == 1250 && output.saturated_mask == 1);
    CHECK(output.pulse_us[4] == 0);
    controls = (flight_controls_t){-1,-1,1,0};
    CHECK(fio_actuators_map(&config, &controls, &output));
    CHECK(output.pulse_us[0] == 1000 && output.pulse_us[2] == 1100 && output.pulse_us[3] == 1000);
    controls.throttle = 1;
    CHECK(fio_actuators_map(&config, &controls, &output) && output.pulse_us[3] == 2000);
    unsigned char previous[sizeof(output)]; memcpy(previous, &output, sizeof(output));
    controls.elevator = NAN;
    CHECK(!fio_actuators_map(&config, &controls, &output));
    controls.elevator = -1; controls.throttle = -.1f;
    CHECK(!fio_actuators_map(&config, &controls, &output));
    controls.throttle = 0; config.channels[0].aileron_weight = FLT_MAX;
    config.channels[0].elevator_weight = FLT_MAX;
    CHECK(!fio_actuators_map(&config, &controls, &output)); /* Arithmetic overflow. */
    CHECK(memcmp(previous, &output, sizeof(output)) == 0);
    config.channels[0].maximum_us = config.channels[0].minimum_us;
    CHECK(!fio_actuator_config_valid(&config));
    config.count = 9;
    CHECK(!fio_actuator_config_valid(&config));
    CHECK(!fio_actuator_config_valid(NULL));
    return 0;
}

#include "autopilot/control/pitch.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; \
} } while (0)

int main(void)
{
    const control_pitch_step_config_t pitch = {10.0f, 3.0f};
    const control_surface_config_t elevator = {-0.05f, -0.5f, 0.5f};
    float command = 99.0f;

    CHECK(control_pitch_step(0.0f, 0.0f, 0.0f, &pitch, &elevator, &command));
    CHECK(fabsf(command - elevator.trim) < 1e-6f);

    CHECK(control_pitch_step(0.02f, 0.0f, 0.0f, &pitch, &elevator, &command));
    CHECK(fabsf(command + 0.25f) < 1e-6f); /* Nose-up needs negative elevator. */

    CHECK(control_pitch_step(0.02f, 0.0f, 0.01f, &pitch, &elevator, &command));
    CHECK(fabsf(command + 0.22f) < 1e-6f); /* Nose-up rate reduces the demand. */

    CHECK(control_pitch_step(0.1f, 0.0f, 0.0f, &pitch, &elevator, &command));
    CHECK(command == elevator.min_command);
    CHECK(control_pitch_step(-0.1f, 0.0f, 0.0f, &pitch, &elevator, &command));
    CHECK(command == elevator.max_command);

    command = 0.123f;
    CHECK(!control_pitch_step(NAN, 0.0f, 0.0f, &pitch, &elevator, &command));
    CHECK(command == 0.123f);
    return 0;
}

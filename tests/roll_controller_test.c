#include "autopilot/control/roll.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); \
    return 1; \
} } while (0)

int main(void)
{
    const control_roll_step_config_t roll = {4.0f, 0.5f};
    const control_surface_config_t aileron = {0.02f, -0.4f, 0.4f};
    float command = 99.0f;

    CHECK(control_roll_step(0.0f, 0.0f, 0.0f, &roll, &aileron, &command));
    CHECK(fabsf(command - aileron.trim) < 1e-6f);

    CHECK(control_roll_step(0.1f, 0.0f, 0.0f, &roll, &aileron, &command));
    CHECK(command == aileron.max_command); /* Positive bank error reaches the limit. */

    CHECK(control_roll_step(0.05f, 0.0f, 0.2f, &roll, &aileron, &command));
    CHECK(fabsf(command - 0.12f) < 1e-6f); /* Positive roll rate reduces right aileron. */

    CHECK(control_roll_step(-0.2f, 0.0f, 0.0f, &roll, &aileron, &command));
    CHECK(command == aileron.min_command);

    command = 0.123f;
    CHECK(!control_roll_step(NAN, 0.0f, 0.0f, &roll, &aileron, &command));
    CHECK(command == 0.123f);
    return 0;
}

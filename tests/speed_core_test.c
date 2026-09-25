#include "autopilot/autopilot.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

int main(void)
{
    ap_config_t config = {2.0f, 0.3f, 0.35f, 0.5f,
                          3.0f, 0.5f, 0.2f, 0.5f,
                          0.1f, 0.05f, 0.2f, 0.8f};
    ap_input_t input = {0};
    input.mode = AP_MODE_ATTITUDE_AIRSPEED_HOLD;
    input.dt_s = 1.0f;
    input.state.airspeed_m_s = 50.0f;
    input.requested.throttle = 0.5f;
    input.airspeed_command_m_s = 51.0f;
    ap_runtime_t runtime = {0};
    ap_output_t output = {0};

    CHECK(!ap_step(&config, &input, &output)); /* Stateful mode needs runtime. */
    CHECK(ap_step_with_runtime(&config, &input, &runtime, &output));
    CHECK(output.controls.throttle > 0.64f && output.controls.throttle < 0.66f);
    CHECK(runtime.airspeed_integral_norm > 0.049f && runtime.airspeed_integral_norm < 0.051f);
    CHECK(output.airspeed_error_m_s == 1.0f && !output.throttle_saturated);
    CHECK(ap_step_with_runtime(&config, &input, &runtime, &output));
    CHECK(output.controls.throttle > 0.69f && output.controls.throttle < 0.71f);

    const float held_integral = runtime.airspeed_integral_norm;
    input.airspeed_command_m_s = 100.0f;
    for (int i = 0; i < 10; ++i) {
        CHECK(ap_step_with_runtime(&config, &input, &runtime, &output));
        CHECK(output.controls.throttle == 0.8f && output.throttle_saturated);
        CHECK(runtime.airspeed_integral_norm == held_integral);
    }
    input.airspeed_command_m_s = 1.0f;
    CHECK(ap_step_with_runtime(&config, &input, &runtime, &output));
    CHECK(output.controls.throttle == 0.2f && output.throttle_saturated);
    CHECK(runtime.airspeed_integral_norm == held_integral);

    /* Once back inside the actuator range, integration can unwind. */
    input.airspeed_command_m_s = 49.9f;
    CHECK(ap_step_with_runtime(&config, &input, &runtime, &output));
    CHECK(runtime.airspeed_integral_norm < held_integral);

    const ap_output_t previous = output;
    const float previous_integral = runtime.airspeed_integral_norm;
    config.airspeed_ki = NAN;
    CHECK(!ap_step_with_runtime(&config, &input, &runtime, &output));
    CHECK(runtime.airspeed_integral_norm == previous_integral);
    CHECK(output.controls.throttle == previous.controls.throttle);
    config.airspeed_ki = 0.05f;
    input.mode = AP_MODE_MANUAL;
    CHECK(ap_step_with_runtime(NULL, &input, &runtime, &output));
    CHECK(runtime.airspeed_integral_norm == 0.0f);
    CHECK(output.controls.throttle == 0.5f);
    return 0;
}

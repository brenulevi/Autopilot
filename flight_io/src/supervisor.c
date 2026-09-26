#include "flight_io/supervisor.h"
#include "internal.h"

bool fio_supervisor_config_valid(const fio_supervisor_config_t *config)
{
    return config != NULL && config->rc_timeout_ms > 0 && config->rc_timeout_ms <= INT32_MAX &&
           config->autopilot_timeout_ms > 0 && config->autopilot_timeout_ms <= INT32_MAX &&
           fio_controls_valid(&config->disarmed_controls) && config->disarmed_controls.throttle == 0.0f &&
           fio_controls_valid(&config->failsafe_controls);
}

bool fio_supervisor_step(const fio_supervisor_config_t *config, uint32_t now_ms,
                         bool armed, const flight_rc_sample_t *rc,
                         const flight_control_sample_t *autopilot,
                         fio_supervisor_runtime_t *runtime,
                         fio_supervisor_output_t *output)
{
    if (runtime == NULL || output == NULL || !fio_supervisor_config_valid(config)) return false;
    fio_supervisor_runtime_t next = *runtime;
    fio_supervisor_output_t result = {0};
    const bool rc_ok = rc != NULL && rc->valid && fio_controls_valid(&rc->controls) &&
        (uint32_t)(now_ms - rc->received_at_ms) <= config->rc_timeout_ms;
    const bool ap_ok = autopilot != NULL && autopilot->valid && fio_controls_valid(&autopilot->controls) &&
        (uint32_t)(now_ms - autopilot->received_at_ms) <= config->autopilot_timeout_ms;
    if (!armed) {
        next.auto_eligible = false;
        result.controls = config->disarmed_controls;
        result.status = (flight_io_status_t){FLIGHT_AUTHORITY_DISARMED, FLIGHT_REASON_DISARMED};
    } else if (!rc_ok) {
        next.auto_eligible = false;
        result.controls = config->failsafe_controls;
        result.status = (flight_io_status_t){FLIGHT_AUTHORITY_FAILSAFE, FLIGHT_REASON_RC_UNAVAILABLE};
    } else if (!rc->request_auto) {
        next.auto_eligible = true;
        result.controls = rc->controls;
        result.status = (flight_io_status_t){FLIGHT_AUTHORITY_MANUAL, FLIGHT_REASON_PILOT_REQUEST};
    } else if (!ap_ok) {
        next.auto_eligible = false;
        result.controls = rc->controls;
        result.status = (flight_io_status_t){FLIGHT_AUTHORITY_MANUAL, FLIGHT_REASON_AUTOPILOT_UNAVAILABLE};
    } else if (!next.auto_eligible) {
        result.controls = rc->controls;
        result.status = (flight_io_status_t){FLIGHT_AUTHORITY_MANUAL, FLIGHT_REASON_REENGAGEMENT_REQUIRED};
    } else {
        result.controls = autopilot->controls;
        result.status = (flight_io_status_t){FLIGHT_AUTHORITY_AUTOPILOT, FLIGHT_REASON_AUTO_ACTIVE};
    }
    *runtime = next;
    *output = result;
    return true;
}

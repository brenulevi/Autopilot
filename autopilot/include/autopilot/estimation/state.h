#ifndef AUTOPILOT_ESTIMATION_STATE_H
#define AUTOPILOT_ESTIMATION_STATE_H

/* SI units, radians, body axes X forward, Y right, Z down.
 * Euler attitude relative to local NED. This is the controller's state
 * interface: JSBSim supplies exact values now; an estimator can supply
 * measured/estimated values later without changing the controller API. */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float p_rad_s;
    float q_rad_s;
    float r_rad_s;
    float airspeed_m_s; /* True airspeed. */
    float altitude_m;  /* Above mean sea level, positive up. */
    float climb_rate_m_s; /* Altitude rate, positive up. */
} ap_state_t;

#endif

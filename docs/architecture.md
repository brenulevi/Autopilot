# Autopilot architecture

The runtime path is intentionally one-way:

```text
mission -> guidance -> attitude P -> rate PI -> actuator commands
                       ^
             aircraft state and setpoints
```

`ap_mission_step()` owns mission navigation state. It selects the active leg,
handles capture and turns, and returns navigation references such as bank,
altitude, and airspeed. It does not compute servo commands.

The guidance layer converts references and navigation errors into attitude
references. The altitude module is an outer-loop PD law: altitude error gives a
pitch offset and climb rate supplies damping. Bank and pitch limiters enforce
attitude-command limits. Guidance therefore produces references, rather than
actuator values.

The control layer turns attitude references into normalized actuator commands.
Roll and pitch use outer attitude P loops and inner body-rate PI loops; airspeed uses throttle PI;
manual control only bounds pilot commands.

`autopilot/src/autopilot/step.c` is the composition boundary. It validates the
sample, invokes guidance and control for the selected mode, and commits the
output and caller-owned `ap_runtime_t` atomically. It does not contain mission
leg logic or simulator code.

The public interfaces are grouped by responsibility:

- `autopilot/autopilot.h`: one control step for all control modes.
- `autopilot/control/{roll,pitch,airspeed}.h`: controller-specific configuration,
  composed settings, plus the independent `_attitude.h` and `_rate.h` interfaces.
- `autopilot/guidance/altitude.h`: altitude configuration, measurements, and pitch reference.
- `autopilot/mission.h`: mission preparation, decoding, projection, and step.
- `autopilot/path.h`: reusable planar path geometry.
- `autopilot/config.h`: persistent configuration codec.

The source tree mirrors those boundaries: `src/mission`, `src/guidance`,
`src/control`, `src/autopilot`, and `src/config`.

## Data ownership and composition

`ap_state_t` remains the shared measurement snapshot. `ap_input_t` and
`ap_output_t` are the boundary with the simulator/firmware. Inside `ap_step()`,
each module receives only its own inputs and configuration and returns a small
result. Modules do not depend on the aggregate autopilot types.

`ap_config_t` composes `.roll`, `.pitch`, `.airspeed`, `.altitude`, and
`.attitude_limits`. Guidance limits attitude references before roll/pitch control;
actuator limits belong to the inner rate controllers. Roll and pitch each compose
`.attitude` and `.rate` settings. Attitude and altitude calculations are stateless;
`ap_runtime_t.roll`, `.pitch`, and `.airspeed` own their respective PI integrators.
Inactive axes reset, while failed ticks preserve all state and output.

`ap_controller_t` groups a configuration copy with persistent runtime. The caller
allocates it on the stack or in static storage. Initialization validates the full
configuration and resets runtime; each successful step updates that instance's
runtime. Invalid steps leave runtime and output unchanged. Fields remain public
for allocation and inspection; this is composition rather than opaque storage.

```c
ap_controller_t controller;
if (!ap_controller_init(&controller, &aircraft_config.control)) {
    /* Handle invalid configuration before running the control loop. */
}
/* Each tick, after successful initialization: */
if (ap_controller_step(&controller, &input, &output)) {
    /* Send output.controls to the simulator or actuator interface. */
}
```

The simulator uses this instance API. `ap_step(config, input, runtime, output)`
remains available for callers that manage configuration and runtime separately;
it validates the settings required by the selected mode. APCF v2 stores cascade gains and rate limits explicitly. The decoder migrates
compatible v1 records rather than reinterpreting old gain units. See
[the cascade guide](cascaded_control.md) and [configuration](configuration.md).

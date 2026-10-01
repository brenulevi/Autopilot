# Autopilot architecture

The runtime path is intentionally one-way:

```text
mission  ->  guidance  ->  control  ->  actuator commands
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
Roll and pitch use angle and body-rate feedback; airspeed uses the throttle PI;
manual control only bounds pilot commands.

`autopilot/src/autopilot/step.c` is the composition boundary. It validates the
sample, invokes guidance and control for the selected mode, and commits the
output and caller-owned `ap_runtime_t` atomically. It does not contain mission
leg logic or simulator code.

The public interfaces are grouped by responsibility:

- `autopilot/autopilot.h`: one control step for all control modes.
- `autopilot/mission.h`: mission preparation, decoding, projection, and step.
- `autopilot/path.h`: reusable planar path geometry.
- `autopilot/config.h`: persistent configuration codec.

The source tree mirrors those boundaries: `src/mission`, `src/guidance`,
`src/control`, `src/autopilot`, and `src/config`.

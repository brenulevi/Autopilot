# Autopilot C library design notes

This branch starts with the flight-mode distinction discussed before coding.
These are design intentions, not implemented behavior.

```text
AUTO: mission -> guidance -> targets --+
                                      +-> shared controllers -> actuators
FBW:  pilot sticks -> targets --------+

AUTO pilot intervention: pilot sticks -> target mixer -> shared controllers
```

## Terms

- **Flight mode** selects who supplies the targets. The first candidates are
  AUTO (mission guidance) and FBW (pilot sticks).
- **Mission state** selects what AUTO is trying to do, such as following a leg
  or returning home. It is separate from the flight mode.
- **Controller** tracks a target, such as bank or pitch. Both flight modes can
  reuse the same controllers.
- **Pilot mixing** is a possible way to influence AUTO without changing flight
  mode. Its authority, limits, and pitch/altitude interaction must be specified
  before implementation.

## Decisions to make before implementation

1. Define the estimated aircraft state, units, signs, and valid-data rules.
2. Define the pilot input, mission target, controller output, and runtime state
   as separate types. Decide which layer owns each target.
3. Define mode transitions and what happens to mission progress when leaving
   or re-entering AUTO.
4. Define how pilot input mixes with AUTO guidance, including limits and what
   pitch input means while altitude guidance is active.
5. Define actuator bounds, failure behavior, and the tests for each control
   loop before using it in a mission.

The earlier implementation can be inspected on `main` as a reference, without
carrying its control laws or mission assumptions into this scaffold.

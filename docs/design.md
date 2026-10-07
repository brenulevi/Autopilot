# Autopilot design notes

## Current scope and boundary

The portable library owns control calculations. A caller supplies measured aircraft state, targets, elapsed time, controller configuration, and trim commands stored in that configuration to `autopilot_step`. It returns normalized aileron, elevator, and throttle commands. The caller owns the controller state and all I/O. JSBSim integration lives in the C++ simulator, outside the C library.

```text
Experiment target ─┐
                   ├─> autopilot_step (C) ─> aileron, elevator, throttle ─> JSBSim
JSBSim measurements┘                                               │
          ^                                                         │
          └──────────────── next simulation step ───────────────────┘
```

The public API is in `autopilot/include/autopilot/autopilot.h`. `autopilot_config_t` contains gains, trim, and command limits; `autopilot_input_t` contains targets, measurements, and time step; `autopilot_state_t` holds the airspeed integral; `autopilot_output_t` contains commands. `autopilot_step` returns false for invalid input without changing the caller's state or output.

Angles are radians, angular rates are radians per second, airspeed is metres per second, and the time step is seconds. Aileron and elevator commands are normalized, typically within `[-1, 1]`; throttle is within `[0, 1]`. The configured command limits constrain logical surface commands. Conversion to PWM, servo calibration, physical travel limits, and a hardware failsafe belong to the aircraft I/O layer.

## Control loops

Each loop compares a target with a measurement and produces an actuator command. Saturation limits prevent the command from exceeding the configured range.

| Loop | Control law before saturation | Actuator |
| --- | --- | --- |
| Roll | `aileron_trim + K_bank * (bank_target - bank) - K_p * body_roll_rate` | Aileron |
| Pitch | `elevator_trim - K_pitch * (pitch_target - pitch) + K_q * body_pitch_rate` | Elevator |
| Airspeed | `throttle_trim + K_v * (airspeed_target - airspeed) + integral` | Throttle |

The pitch signs reflect the C172X elevator convention used by this adapter: a more negative elevator command raises the nose. The roll and pitch controllers are PD controllers: angle error provides the restoring action and body rate damps motion. Body roll and pitch rates are convenient feedback signals, but they are not exactly the Euler bank and pitch angle derivatives in a general attitude.

The airspeed controller is PI. It advances `integral` by `K_i * speed_error * dt`, except when doing so would push an already saturated throttle farther into saturation. The caller retains that state between steps. The pitch controller has no integral term, following the current decision to keep that loop simple.

## JSBSim experiment

`sim/` runs the pinned JSBSim C172X model at a fixed 0.01 s step. It starts near 3000 ft and 100 kt, trims the aircraft, and applies a step target at 2 s. The hardcoded gains, trims, and command bounds for this model are in `sim/include/sim/c172x_config.hpp`; a configuration file is not implemented. The adapter converts JSBSim calibrated airspeed from knots to metres per second before calling the C library. Rudder remains at its trimmed command.

The `--mode roll|pitch|airspeed` option selects the target changed by the experiment. It does **not** enable or disable a controller: roll, pitch, and airspeed loops all run on every step. In the logs, roll targets are absolute bank angles, while pitch and speed targets are offsets from trim. The CSV also includes logical control commands and the requested and actual physical elevator positions.

The C172X elevator actuator has lag and hysteresis. Its actual position can remain offset from the requested position, leaving a small persistent pitch error. That behavior can be inspected in the CSV before changing controller gains or adding integral action. Gains are tuned only for this simulated aircraft and approximate trim condition; they are not flight-ready settings.

## Operational architecture under consideration

An eventual **FBW** mode would convert pilot demands into controlled attitude or rate targets. A future **AUTO** mode would obtain targets from guidance, which in turn interprets a mission. A pilot takeover policy would choose or combine those demands before they reach the controllers. These operational modes and that mixer are design ideas only; they are not implemented here. Likewise, the simulator's experiment mode is only a test selection.

```text
Mission ─> guidance ────────────┐
                                ├─> target selection / pilot intervention ─> control ─> actuator commands
Pilot input ─> FBW demands ─────┘
```

The planned hardware boundary has a separate Flight I/O MCU receive the RC receiver and drive the servos. The Flight Control MCU would send autopilot demands to Flight I/O. Flight I/O must be able to pass pilot commands to the servos if Flight Control fails. No firmware, link protocol, arbitration policy, or failover behavior is implemented in this branch; the JSBSim adapter currently plays the role of the surrounding application.

## Altitude and energy: next design step

With pitch and airspeed inner loops observable, an initial altitude experiment could use an outer altitude loop to request a bounded climb rate or pitch target. The pitch loop would move the elevator toward that target, while the airspeed loop would continue to command throttle. Log altitude, vertical speed, pitch target, airspeed, and throttle together so a climb request can be checked against available power. When throttle is at its upper bound and speed falls, the climb or pitch demand needs to be reduced rather than held indefinitely.

This simple cascade is a useful learning step, but altitude and speed both depend on the aircraft's energy. A later TECS-style controller could coordinate throttle for total energy and pitch for the altitude/speed tradeoff, with limits for climb, descent, pitch, and speed. The current branch implements neither altitude control nor TECS. ArduPilot's [TECS tuning guide](https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html) is a reference for that later stage.

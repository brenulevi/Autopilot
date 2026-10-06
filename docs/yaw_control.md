# Coordinated yaw-rate control

A yaw damper uses rudder feedback to reduce unwanted yaw oscillations, including
Dutch roll. During a turn it must allow the expected yaw motion. This implementation
uses a coordinated body yaw-rate reference and a PI rudder loop to damp deviations
and track that reference. It does not yet close a loop around sideslip or lateral
specific force. [ArduPilot's yaw tuning guide](https://en.ardupilot.org/plane/docs/roll-pitch-controller-tuning.html)
describes yaw damping, turn coordination, and sideslip feedback as separate functions.

## Data flow and geometry

`turn_coordination.c` consumes measured bank, measured pitch, and true airspeed.
For an approximate steady coordinated horizontal turn, it computes:

```text
r_target = clamp(g * sin(bank) * cos(pitch) / max(TAS, speed_floor), +/-r_max)
q_feedforward = r_target * tan(bank)
rudder = clamp(trim + rudder_sign * (Kp * (r_target - r_measured) + I), +/-authority)
```

These are body rates. The horizontal heading rate is approximately
`g*tan(bank)/TAS` near level pitch; feeding it directly to the body r loop is
incorrect. Using measured bank lets the yaw reference follow the actual turn
instead of leading the roll response. The pitch feedforward accounts for the
body q motion needed in a steady banked turn and is added to the attitude P
reference before the final pitch-rate limit. This geometry follows the
[PX4 turn-coordination description](https://docs.px4.io/main/en/flight_stack/controller_diagrams#turn-coordination).
It is an approximation for ordinary upright turns, not a general attitude mapping.

The measured bank is clamped below 90 degrees before evaluating `tan(bank)`.
The positive TAS floor avoids division by zero; it is not stall protection.
Reference limits, speed guarding, and bank guarding are exposed in the output.
Using TAS in the reference does not implement airspeed gain scheduling.

## Ownership, modes, and limits

`config.yaw` owns enablement, gains, rudder authority/sign, and model guards.
`runtime.yaw` retains the integral in normalized effort. A zero-initialized
runtime starts without an integral; reinitialize after changing gains or sign.
Conditional anti-windup blocks integration farther into saturation and allows
unwinding for either rudder sign. The authority bounds the complete rudder
command, including trim. All controller memory belongs to the caller; the C
helpers perform no allocation, file access, or simulator access.

With yaw enabled, attitude hold, attitude/airspeed hold, and altitude/airspeed
hold (including mission guidance) control the rudder. Manual, roll-only, and
pitch-only modes pass through the requested rudder and clear the yaw integral.
Disabling yaw also restores the previous pitch law. Mode changes are not
bumpless. A rejected tick commits neither partially updated runtime nor output.

APCF v3 stores yaw parameters. Older v1/v2 profiles decode with yaw disabled;
the supplied `configs/c172x.apcf` is still v1. Creating a new profile uses the
current C172X defaults. See [configuration](configuration.md) for exact wire
fields and validation ranges.

## Experiments and verification

The simulator accepts `--yaw-enabled 0|1`, `--yaw-rate-kp`, `--yaw-rate-ki`,
`--rudder-sign -1|1`, `--rudder-limit`, `--yaw-rate-limit-deg-s`,
`--yaw-min-airspeed-m-s`, and `--yaw-bank-limit-deg`. Explicit overrides apply
after a loaded profile regardless of argument order. A manual `--rudder-pulse`
applies during 2–2.5 seconds and disables the default aileron pulse.

CSV appends yaw activation, target/error, integral, guard flags, rudder saturation,
pitch feedforward, effective parameters, sideslip, and lateral specific force.
Sideslip is JSBSim `aero/beta-rad`; lateral specific force is its pilot-site
`accelerations/a-pilot-y-ft_sec2` converted to m/s². These diagnostics remain
in the simulator adapter and are not controller feedback.

```powershell
ctest --preset core -R '^autopilot.yaw_core$' -V
ctest --preset host -R '^autopilot.yaw_control$' -V
./build/host/bin/Debug/autopilot_sim.exe --mode manual --rudder-pulse 0.03 --output logs/rudder_pulse.csv
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --yaw-enabled 0 --output logs/turn_yaw_off.csv
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --output logs/turn_yaw_on.csv
```

The C test checks reference geometry, both signs, limits, guards, PI persistence,
anti-windup, inactive-mode reset, and transactional failure. The JSBSim test
first probes both rudder signs, then checks direct ±0.02 rad/s rate steps and
returns with outer yaw coordination bypassed. It compares yaw on/off in ramped
±15-degree bank turns, preserving surface and attitude limits. Its eight CSVs
are saved in `build/host/yaw_control_logs`.

The compiled C172X simulation profile uses yaw Kp = 10 normalized effort/(rad/s),
Ki = 10 normalized effort/rad, authority 0.5, and rudder sign -1. Its body r
reference is capped at 0.3 rad/s, TAS floor is 25 m/s, and bank guard is 45 degrees.
At 3000 ft / 100 kt CAS, direct ±0.02 rad/s steps have worst tracking error
0.005730 rad/s during 4–5 seconds and worst return error 0.001199 rad/s during
8–9 seconds. These are finite windows with bank/pitch stabilized, not steady
heading-hold capability with the rudder.

The turn comparison uses the same roll/pitch gains and initial trim in both
cases. Yaw on also enables pitch coordination feedforward. Bank ramps over two
seconds; RMS values use 12–20 seconds after settling:

| Bank | Sideslip RMS, off → on (deg) | Lateral specific force RMS, off → on (m/s²) | Body r error RMS, on (rad/s) |
| --- | --- | --- | --- |
| +15 deg | 0.3781 → 0.2115 | 0.01484 → 0.004052 | 0.00002808 |
| -15 deg | 0.3670 → 0.1601 | 0.06209 → 0.01100 | 0.0001228 |

Both directions reduce sideslip and lateral-force RMS by more than the fixed
20% regression requirement. No surface saturates in these cases. With yaw on,
late bank/pitch errors stay below 0.149/0.362 degrees. Residual sideslip remains;
tracking the approximate turn rate does not guarantee zero sideslip.

This is ideal-state C172X simulation evidence. It does not establish Skyward
gains, disturbance rejection with real sensors, flight-envelope protection,
or zero sideslip throughout arbitrary maneuvers.

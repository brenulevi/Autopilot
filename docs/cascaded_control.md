# Attitude and body-rate cascades

The command path is guidance -> attitude P -> rate PI -> actuator demand.
Guidance retains route, altitude, and attitude-reference limits. `roll_attitude.c`
and `pitch_attitude.c` generate body-rate targets; `roll_rate.c` and `pitch_rate.c`
track those targets using measured `p` and `q`.

## Equations and units

For roll, `p_target = clamp(K_attitude * bank_error, +/-p_max)` and
`aileron = trim + K_rate * (p_target - p_measured) + I_roll`.
For pitch, `q_target = clamp(K_attitude * pitch_error, +/-q_max)` and
`elevator = trim - (K_rate * (q_target - q_measured) + I_pitch)`.
Outer gains have units 1/s. Inner Kp is normalized effort/(rad/s), and Ki is
normalized effort/rad. `dt_s` now affects roll and pitch integration.

The outer mappings are near-level approximations. Euler bank/pitch derivatives
are not identified with body rates in general maneuvers. There is no quaternion
attitude mapping, coordinated yaw-rate target, airspeed scheduling, or control
allocation in this implementation. The C172X tests cover the stated simulator
operating point; further envelope testing is required for another aircraft.

## Ownership and limiting

`config.roll.attitude` and `.pitch.attitude` own outer gains and rate limits.
`config.roll.rate` and `.pitch.rate` own PI gains and actuator authority.
`runtime.roll` and `.pitch` each retain an integral measured in normalized effort.
Positive effort increases positive body rate; elevator uses the opposite servo
sign. All memory remains caller-owned and there is no allocation or I/O in control.

Integral candidates are bounded by actuator authority. Conditional anti-windup
rejects integration farther into saturation and permits integration back out,
including the reversed elevator sign. Inactive attitude axes clear their integral;
manual clears all controller integrals. Reinitialization clears all state. A
failure at any stage commits neither earlier axis state nor partial output.
Reset the instance after changing gains/authority. Mode transitions are not
bumpless and references are not ramped.

## Validation order

The compiled C172X baseline in `sim/include/sim/c172x_config.hpp` uses:

| Axis | Attitude gain (1/s) | Rate Kp (effort/(rad/s)) | Rate Ki (effort/rad) | Rate limit (rad/s) | Actuator limit |
| --- | --- | --- | --- | --- | --- |
| Roll | 2 | 2 | 1 | 1 | 1 |
| Pitch | 1.5 | 4 | 4 | 0.5 | 0.5 |

These gains were tuned in ideal-state JSBSim runs. They are a C172X simulation
baseline; they require separate tuning and validation for the Skyward.

Measured maxima from the final defaults at 100 kt CAS / 3000 ft MSL:

| Case, both directions | Tracking error | Return error |
| --- | --- | --- |
| Direct roll rate, +/-0.08 rad/s | 0.008592 rad/s | 0.006527 rad/s |
| Direct pitch rate, +/-0.03 rad/s | 0.003872 rad/s | 0.003333 rad/s |
| Roll attitude, +/-5 degrees | 0.100 degrees | 0.101 degrees |
| Pitch attitude, +/-2 degrees | 0.420 degrees | 0.414 degrees |

Rate errors use the 4–5 s tracking and 8–9 s return windows. Attitude errors
use 6–10 s and 14–20 s. Combined attitude checks at 90/100/110 kt and the
negative command at 100 kt had at most 0.156 degrees bank and 0.491 degrees
pitch error in those windows. Small-command cases remained unsaturated.
These are finite-window measurements, not stability margins or an envelope.

First build and run `autopilot.rate_loop` with JSBSim. Its four CSVs in
`build/host/rate_loop_logs` contain direct +/-0.08 rad/s roll and +/-0.03 rad/s
pitch steps and returns. The tested axis bypasses its outer attitude loop; the
other attitude axis remains stabilized. This distinguishes inner rate tracking
from a nested controller that passes only through outer-angle feedback.

```powershell
cmake --build --preset host --target rate_loop_test --parallel 4
ctest --preset host -R '^autopilot.rate_loop$' -V
```

Only after rate tracking passes, run roll/pitch attitude, combined attitude,
airspeed, altitude, and mission checks. Existing regression bounds are retained.
`cascade_core_test` separately checks rate targets and limits, both actuator
signs, persistent PI state, saturation/unwinding, inactive-axis reset, and rollback.

The simulator accepts explicit `--roll-attitude-gain`, `--roll-rate-kp`,
`--roll-rate-ki`, `--roll-rate-limit-deg-s` and corresponding pitch options.
The historical `--roll-kp` and `--pitch-kp` set the equivalent proportional angle
coefficient `K_attitude * K_rate`. Historical `--roll-kd` and `--pitch-kd` change
inner Kp while preserving that product. These legacy damping values must now be
strictly positive. Use explicit cascade flags when tuning stages independently.

CSV retains `roll_kp`/`pitch_kp` as equivalent proportional angle coefficients and
`roll_kd`/`pitch_kd` as rate Kp for existing plots. New columns expose each rate
target, measured-rate error, target limiting, integral, actual stage gains, and
rate limit. APCF v2 saves all settings; see [configuration](configuration.md) for
v1 conversion and the changed gain units.

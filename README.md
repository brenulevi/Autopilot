# Autopilot

Undergraduate flight-control project: a portable **C11 autopilot library**, a
**C++17 JSBSim adapter**, and a native simulation runner built with **CMake**.
The first aircraft is JSBSim's C172X. A representative Skyward model comes later.

The project includes the complete H723 flight-controller firmware and F405 I/O
firmware as planned deliverables. The portable libraries are their testable
algorithm and protocol components. A feature described as a firmware
responsibility is work to implement here, not an external project prerequisite.
At present the libraries and host simulation exist; MCU firmware targets do not.

The current priority is a complete waypoint-mission FCS validated in JSBSim,
using simulator truth for state inputs. Firmware bring-up, the MCU link, and
physical flash integration follow that milestone. Nearest-leg selection,
Dubins entry capture, and APM3 fly-by/fly-over waypoint turns are implemented.
The next simulation work is systematic validation across turns,
altitude/speed changes, wind, and aircraft models, followed by mission
completion behavior and state-estimation integration.

The C library supports **manual passthrough**, **roll hold**, **pitch attitude
hold**, **combined attitude hold**, **attitude plus true-airspeed hold**,
**altitude plus true-airspeed hold**, and **waypoint mission guidance**.
The runner can apply open-loop control pulses, then test the loops in JSBSim.

## Structure

```text
common/                    Shared types and binary flight logger (Flight::Common)
flight_io/                 Portable C11 RC, authority, and output mapping (Flight::IO)
autopilot/                 C library; no JSBSim, OS, heap, or board dependencies
  include/autopilot/autopilot.h  Public ap_step() API and mode/configuration types
  include/autopilot/mission.h    Compiled-mission decoder and L1 guidance API
  include/autopilot/control/actuators.h  Normalized actuator-command type
  include/autopilot/estimation/state.h   State interface for simulator/estimator
  src/autopilot.c         Input validation and mode dispatch
  src/control/manual.c    Bounds manual commands and untouched control axes
  src/control/roll.c      Bank-error and body-roll-rate feedback
  src/control/pitch.c     Pitch-error and body-pitch-rate feedback
  src/control/airspeed.c  Throttle PI with conditional anti-windup
  src/guidance/bank.c     Limits the requested bank angle
  src/guidance/pitch.c    Limits the requested pitch angle
  src/guidance/altitude.c Produces a pitch request from altitude and climb rate
  src/mission.c           APM2/APM3 decoder, capture, turns, and L1 guidance
  src/path.c              Planar Dubins path geometry
sim/                      C++ JSBSim adapter and command-line runner
missions/                 Editable JSON mission examples
tools/compile_mission.py  Host-side JSON-to-APM2/APM3 compiler
cmake/JSBSim.cmake         Pinned dependency and optional local source override
tests/                    C API checks and a real C172X integration check
```

`autopilot/` and `flight_io/` share `flight_controls_t` through `common/`;
`ap_controls_t` remains a compatibility alias. Neither library depends on the
other. Flight I/O provides iBUS decoding, configurable RC calibration, explicit
pilot/autopilot selection with stale-command fallback and re-engagement latching,
and actuator mixing into pulse-width demands. STM32 UART/timer drivers and the
MCU-to-MCU wire protocol remain firmware work. See the
[Flight I/O guide](flight_io/README.md) for its API, policy, and F405/H723 boundary.

Both firmware applications can use the shared **FLG1 binary flight logger**, with
source/session IDs, local timestamps, sequence numbers, CRCs, and a copying
enqueue callback. A PC decoder exports flash dumps to CSV. See
[flight logging](docs/flight_logging.md) for the format, usage, and remaining
firmware queue/flash/download work. The existing simulator CSV is unchanged.

Each simulation tick reads JSBSim state, calls the C autopilot, applies the
returned commands through the adapter, and advances the aircraft by 0.01 seconds.
The loop normally runs as fast as the computer allows. `--flightgear` sends
aircraft state over localhost UDP and paces the simulation in real time.
`ap_step()` handles stateless modes; `ap_step_with_runtime()` handles modes with
throttle PI state. The control and guidance
modules have private headers and can evolve without changing callers. The
`estimation/state.h` file defines the state passed into control; it does not
implement an estimator yet. JSBSim currently fills that structure with exact
simulated state. When sensors are added, the estimator can populate the same
structure. Mission guidance produces a bank request upstream of attitude
control. It uses simulator position and ground velocity, not an estimator yet.

## Build

Requirements: CMake 3.24+, a native C11/C++17 compiler, and internet access on the
first simulation configuration. On Windows, Visual Studio with the **Desktop
development with C++** workload works; CMake can detect it without `cl` on PATH.
The STM32 ARM cross-compiler alone cannot build this Windows simulation executable.

Run from the project root:

```powershell
cmake --preset host
cmake --build --preset host --parallel
ctest --preset host
```

The first configure downloads JSBSim v1.3.1 at commit
`3b25f25e49b42d0489c04ac805674fc1450ca579`, verified with SHA-256. Subsequent builds
reuse it. Python, documentation, and other optional upstream tools are disabled.
No global JSBSim installation is required.

For an existing/offline JSBSim source checkout, including its data folders:

```powershell
cmake --preset host -DJSBSIM_SOURCE_DIR="C:/path/to/jsbsim"
```

To build just the C library and its tests, without C++ or JSBSim:

```powershell
cmake --preset core
cmake --build --preset core --parallel
ctest --preset core
```

For a future embedded toolchain, use a separate build directory:

```text
cmake -S . -B build/board -DCMAKE_TOOLCHAIN_FILE=<your-toolchain.cmake> -DAUTOPILOT_BUILD_SIM=OFF -DBUILD_TESTING=OFF
cmake --build build/board --target autopilot
```

This currently produces the C library. Board startup, drivers, linker scripts,
and flashing targets will be added as this project's firmware implementation.
The command above does not yet build a flashable MCU application.

## First experiment

Aircraft control/guidance settings can now be loaded from an **80-byte binary
APCF file**, with version, sequence number, and CRC32. The C codec is portable
to future EEPROM storage; the host tool reads/writes files without JSON.

```powershell
./build/host/bin/Debug/autopilot_config.exe show configs/c172x.apcf
./build/host/bin/Debug/autopilot_config.exe set configs/c172x.apcf logs/c172x_tuned.apcf roll_angle_gain=3 roll_rate_gain=0.6
./build/host/bin/Debug/autopilot_sim.exe --config logs/c172x_tuned.apcf --mode roll-hold --output logs/configured_roll.csv
```

Explicit gain options override loaded values regardless of argument order.
Without `--config`, the existing defaults apply. The supplied profile is for
the C172X simulation, not Skyward. The writer refuses existing output paths.
See [binary configuration](docs/configuration.md) for creation, the wire format,
validation rules, and the firmware/library boundary.

```powershell
./build/host/bin/Debug/autopilot_sim.exe --aileron-pulse 0 --output logs/baseline.csv
./build/host/bin/Debug/autopilot_sim.exe --output logs/c172x_pulse.csv
```

On Linux/macOS the executable has the same path without `.exe` when using these
Debug presets. Use `--help` for options. The default experiment is:

- C172X at 3000 ft MSL and 100 kt **calibrated** airspeed, initialized in the air.
- Engine running, full trim, built-in autopilot channels explicitly disabled.
- Trimmed commands held for 10 seconds; +0.05 normalized aileron demand added
  during `[2.0, 2.5)` seconds. This is 5% of normalized command, not 5 degrees.
- CSV states are in SI units/radians, and airspeed is **true** airspeed.
- Each CSV row contains the state at time `t` and commands for `[t, t + 0.01)`.
  Ten seconds produces 1000 rows, from 0 to 9.99 seconds; durations round up to a tick.
- Existing CSV files at the requested output path are overwritten.

Plot `roll_rad` and `p_rad_s` against `time_s`, alongside `aileron_cmd_norm`.
Compare the baseline and pulse runs. Returning the aileron to trim does not command
a return to wings-level flight because this manual-mode experiment has no feedback.

## Roll hold

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode roll-hold
./build/host/bin/Debug/autopilot_sim.exe --mode roll-hold --bank-deg -5 --output logs/c172x_roll_hold_negative.csv
```

The default roll-hold experiment lasts 20 seconds:

- Before 2 s: hold the initial trimmed bank angle (approximately -0.162 degrees).
- At 2 s: command +5 degrees, or the value supplied with `--bank-deg`.
- At 10 s: command zero bank (wings level).
- Elevator, rudder, and throttle remain at their original trimmed commands.

The C controller uses:

```text
aileron = aileron_trim + Kp * (limited_bank_command - measured_bank) - Kd * body_roll_rate
```

This is **proportional bank-angle control with roll-rate damping**, often called
PD-like near wings-level flight. It has no integral term, so it is not PI or PID.
It is not a strict derivative of bank-angle error either: body roll rate `p` is
only approximately the derivative of Euler bank angle near level flight. Rate
feedback avoids a derivative kick when the commanded angle steps.

All angle/rate inputs to the library use radians and radians/second. Aircraft
parameters live in `sim/include/sim/c172x_config.hpp`, outside the generic C library:

| Parameter | Default | Meaning |
| --- | --- | --- |
| Kp | 4.0 | Normalized aileron command per radian of bank error |
| Kd | 0.5 | Normalized aileron command per radian/second of body roll rate |
| Bank command limit | +/-20 degrees | Limit on the requested reference |
| Aileron limit | +/-0.5 | Limit on the total command, including trim |

The command limit is not a demonstrated flight envelope. Checks so far cover only
small +/-5-degree commands at the configured C172X cruise condition, with exact
simulator feedback. There is no integral term, so small residual tracking errors
can remain. No altitude or coordinated-turn control is active; true-airspeed
control is a separate mode described below.

For tuning, override the gains explicitly:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode roll-hold --roll-kp 4 --roll-kd 0.5
```

Manual mode remains the default. `--aileron-pulse` belongs to manual mode;
`--bank-deg`, `--roll-kp`, and `--roll-kd` belong to roll-hold mode. Incompatible
options are rejected. `--duration` changes the total run time but not the 2 s and
10 s command transitions; use at least 20 s for the complete default experiment.

### Plot and measure roll tracking

After installing the optional plotting dependencies described below:

```powershell
./.venv/Scripts/python.exe tools/plot_logs.py --roll-hold
./.venv/Scripts/python.exe tools/plot_logs.py --roll-hold --experiment logs/c172x_roll_hold_negative.csv --output logs/plots/c172x_roll_hold_negative
```

This produces PNG/SVG plots of commanded versus measured bank, tracking error,
body roll rate, and aileron demand. A `.metrics.json` file records each command
step's overshoot, settling time, endpoint error, and saturation counts. The CSV
also records raw/effective bank commands, gains, limits, and clipping flags.
Manual-mode logs leave the bank-reference/error columns empty.

Settling time is the last entry into a +/-0.25-degree band that persists through
the remaining observed command interval. Change the band with
`--settling-band-deg`. If it is not reached, the JSON reports `null`, not zero.
Overshoot percent uses the actual reference change, including the initial trim
bank. Endpoint error is command minus measured bank at the last recorded sample;
it is not a claim of asymptotic steady-state error.

Measured results with the default gains, JSBSim 1.3.1, 100 Hz, 3000 ft MSL and
100 kt CAS (no added wind or sensor errors):

| Step | Overshoot | Settling (+/-0.25 deg) | Endpoint error |
| --- | --- | --- | --- |
| Initial trim to +5 deg | 0.199 deg | 1.66 s | +0.107 deg at 9.99 s |
| +5 deg to wings level | 0.217 deg | 1.58 s | -0.096 deg at 19.99 s |
| Initial trim to -5 deg | 0.196 deg | 1.65 s | -0.100 deg at 9.99 s |
| -5 deg to wings level | 0.214 deg | 1.58 s | +0.101 deg at 19.99 s |

Neither run saturated the aileron. The largest absolute demand was approximately
0.411. A short tuning comparison evaluated (Kp, Kd) = (2, 0.3), (3, 0.3),
(3, 0.6), and (4, 0.5). The selected pair reduced overshoot and gave settling
below 1.7 s for both transitions in the positive-bank experiment; the negative
experiment then verified the response in the other direction. These gains are
specific to this simulated model and operating point, not the Skyward or custom PCB.

## Pitch attitude hold

At the same trimmed 3000 ft / 100 kt C172X condition, a +0.15 normalized elevator
pulse produced nose-down pitch; a -0.15 pulse produced nose-up pitch. A 0.03 pulse
did not move the elevator because the model's actuator has a hysteresis band.
The pulse experiment is available with `--mode manual --elevator-pulse VALUE`.

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode pitch-hold --output logs/c172x_pitch_hold.csv
./build/host/bin/Debug/autopilot_sim.exe --mode pitch-hold --pitch-deg -2 --output logs/c172x_pitch_hold_negative.csv
./.venv/Scripts/python.exe tools/plot_logs.py --pitch-hold
```

The pitch request is an **offset from the trimmed pitch attitude** (about
+0.59 degrees), not an absolute nose attitude. The runner holds trim until 2 s,
commands trim +2 degrees by default until 10 s, then returns to trim. Aileron,
rudder, and throttle remain at trim. Pitch hold controls only the pitch axis;
`attitude-hold` below controls pitch and roll together.

The C law is `elevator = trim - Kp × (pitch_command − pitch) + Kd × q`.
`q` is body pitch rate, which is only approximately Euler pitch rate outside
simple attitudes. The C172X gains are Kp = 10 and Kd = 3, with a ±10-degree
absolute pitch-command limit and ±0.5 normalized elevator limit. The gains
assume negative elevator command gives nose-up response; verify actuator sign
again before adapting the library to the Skyward.

For ±2-degree pitch steps, the current simulation's maximum absolute tracking
error during 6–10 s and 14–20 s is below 0.5 degree in both directions, without
elevator saturation. These are small-signal C172X checks only. Constant trimmed
throttle means airspeed and altitude change during a pitch step; pitch hold is
neither airspeed hold nor altitude hold. The plot shows pitch target and response,
error, pitch rate, elevator demand, airspeed, and altitude.

## Combined attitude hold

`attitude-hold` passes the same state and both attitude commands to one C
`ap_step()` call. The existing roll and pitch modules independently compute
aileron and elevator commands; the result is applied to JSBSim in the same
simulation tick. If either axis rejects its input, `ap_step()` returns failure
without changing the caller's output. Rudder and throttle stay at trim.

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold
./.venv/Scripts/python.exe tools/plot_logs.py --attitude-hold
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --airspeed-kts 90 --output logs/c172x_attitude_hold_90kt.csv
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --airspeed-kts 110 --output logs/c172x_attitude_hold_110kt.csv
```

From 2–10 s the default commands are +5 degrees bank and trimmed pitch +2
degrees; after 10 s they return to wings level and trimmed pitch. Both axes
retain their individual command and actuator limits. `--bank-deg`,
`--pitch-deg`, and both gain pairs can be overridden in this mode. The optional
`--airspeed-kts` accepts any finite positive initial calibrated airspeed in knots.
The model must trim successfully at that speed before the experiment; otherwise
the runner reports JSBSim's trim failure and stops. Accepting a speed does not
establish aircraft capability or controller performance at that speed. This
option does not hold speed later.

At initial speeds of 90, 100, and 110 kt, the +5/+2-degree simulation had
maximum bank error below 0.3 degree and pitch error below 0.5 degree during
the later 6–10 s hold and 14–20 s return intervals. A -5/-2-degree case at
100 kt also stayed within those bounds. None saturated aileron or elevator.
These are narrow C172X simulation checks with exact state feedback, not a
validated flight envelope or Skyward tuning. The plot shows both tracking
responses, both actuator demands, airspeed, and altitude.

## True-airspeed hold with throttle

First, the throttle response was measured with bank and pitch held near trim.
A five-second ±0.10 normalized throttle pulse changed true airspeed in the
expected direction and also moved pitch and altitude. Reproduce the pulse
comparison and plot:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --bank-deg 0 --pitch-deg 0 --throttle-pulse 0 --duration 30 --output logs/throttle_attitude_0.csv
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --bank-deg 0 --pitch-deg 0 --throttle-pulse 0.1 --duration 30 --output logs/throttle_attitude_0.1.csv
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-hold --bank-deg 0 --pitch-deg 0 --throttle-pulse -0.1 --duration 30 --output logs/throttle_attitude_-0.1.csv
./.venv/Scripts/python.exe tools/plot_logs.py --throttle-step
```

The `attitude-airspeed-hold` mode keeps bank and pitch at their trimmed angles
and uses throttle to track **true airspeed**. It starts from the trimmed speed,
commands a +1 m/s step from 2–30 s, then returns to the starting speed.
The default run lasts 60 s:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-airspeed-hold
./.venv/Scripts/python.exe tools/plot_logs.py --airspeed-hold
./build/host/bin/Debug/autopilot_sim.exe --mode attitude-airspeed-hold --speed-step-m-s -1 --output logs/c172x_airspeed_hold_negative.csv
```

The throttle law is `trim_throttle + Kp × speed_error + integral`, where the
integral grows by `Ki × speed_error × dt`. The C172X simulation gains are
Kp = 0.08 and Ki = 0.005 in normalized-throttle/SI units. Throttle is limited
to [0, 1]; integration stops when it would push farther into a limit. The
integrator is stored in caller-owned `ap_runtime_t`, initialized to zero and
passed to `ap_step_with_runtime()` on every tick. The earlier `ap_step()` modes
remain stateless. Switching to another mode through `ap_step_with_runtime()`
clears the speed integrator.

At 100 kt initial calibrated airspeed, ±1 m/s true-speed steps had less than
0.2 m/s maximum tracking error in the later 20–30 s hold and 50–60 s return
intervals, without throttle saturation. The +1 m/s case also passed at 90 and
110 kt, within 0.3 m/s in those intervals. These are narrow C172X simulation
checks using exact state, not Skyward gains. Altitude still changes because this
mode does not control altitude. The measured `airspeed_m_s` is **true** airspeed;
it is distinct from the initial-condition `--airspeed-kts` (calibrated speed).

## Altitude hold and banked segment

`altitude-hold` adds an outer loop that commands pitch while the existing
attitude controllers track pitch and bank. The throttle PI holds the initial
true airspeed. Vertical speed is JSBSim altitude rate converted to m/s,
positive upward. The outer loop uses:

```text
pitch_command = trim_pitch + clamp(Kh * (altitude_command - altitude)
                                   - Kv * climb_rate, +/-3 degrees)
```

The C172X defaults are Kh = 0.015 rad/m and Kv = 0.05 rad/(m/s). These are
simulation gains at the configured trim condition, not Skyward settings. Run
the default +5 m step and a separate banked segment experiment:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode altitude-hold
./.venv/Scripts/python.exe tools/plot_logs.py --altitude-hold
./build/host/bin/Debug/autopilot_sim.exe --mode altitude-hold --turn-bank-deg 5 --output logs/c172x_altitude_turn.csv
./.venv/Scripts/python.exe tools/plot_logs.py --altitude-hold --experiment logs/c172x_altitude_turn.csv --output logs/plots/c172x_altitude_turn
```

The altitude command is the initial MSL altitude until 2 s, initial +5 m
during [2, 30) s, then initial altitude again. The optional bank command is
+5 degrees during [40, 60) s and zero otherwise. A negative step or bank can
be selected with `--altitude-step-m` or `--turn-bank-deg`. The banked segment
tests altitude control while the aircraft turns; it does not implement
heading/course guidance or coordinated rudder control. CSV columns and plots
show target and measured altitude, climb rate, pitch and bank, true airspeed,
and limiter flags. `--altitude-kh` and `--altitude-kv` allow gain experiments.

At 3000 ft and 100 kt CAS, the 100 Hz C172X tests check ±5 m altitude steps
and a +5-degree banked segment. Late hold and return errors stay below 0.25 m;
the banked segment also stays within 0.25 m after its initial transient.
The pitch offset limiter and elevator may activate briefly immediately after
the altitude steps. This uses perfect simulated state and no wind or sensor
noise; it is not a flight validation.

## Compiled waypoint mission

Mission engagement selects the nearest route leg, allowing earlier waypoints to
be skipped, and uses a Dubins entry when needed to join it in the forward
direction. Starting position and heading are configurable in JSBSim. See
[mission capture](docs/mission_capture.md) for the policy, implementation,
repeatable offset-start commands, and trajectory plots.

Edit `missions/c172x_line.json`, then compile it on the PC with Python 3.10+.
The compiler uses only the standard library:

```powershell
python tools/compile_mission.py missions/c172x_line.json logs/c172x_line.apm
./build/host/bin/Debug/autopilot_sim.exe --mode mission --mission logs/c172x_line.apm --duration 90 --output logs/c172x_mission.csv
```

On Linux/macOS, use `python3` and the executable without `.exe`. The sample
starts at latitude 30 degrees, longitude 0 degrees, at 3000 ft MSL and the
trimmed C172X airspeed. Each waypoint has `lat_deg` and `lon_deg` in decimal
degrees, an MSL `altitude_m`, and a **true** `airspeed_m_s`. The first waypoint
anchors the local frame and should be near the initialized aircraft. A mission
needs 2–32 waypoints; adjacent points must be at least 1 m apart. The current
short-route projection limits waypoints to ±20 km north/east of the first one.

The compiler checks the source and writes a versioned `APM2` or `APM3` binary: a 12-byte
header, 16-byte records, and a trailing CRC32. Latitude and longitude are
stored as integer degrees × 10⁷; altitude and speed use centimeters and
centimeters/second. The C library checks version, exact length, CRC, reserved
fields, and waypoint bounds. On load, `ap_mission_decode()` derives local
north/east coordinates from the stored lat/lon; it does not require the PC
compiler to supply local coordinates. The binary is separate from the
firmware/library build, and the decoder takes caller-owned bytes for a future
board storage interface. Older `APM1` files are intentionally rejected; convert
their waypoint sources to lat/lon and recompile.

At each tick, the adapter supplies geodetic position and **ground** velocity.
The C mission code projects aircraft position into the same local frame.
`ap_mission_step()` chooses a lookahead target on the active line leg, requests
lateral acceleration using `2 × groundspeed² × sin(course error) / lookahead`,
and converts that acceleration to a bounded bank command. Its default lookahead
is `max(30 m, 4 s × groundspeed)`; experiment with `--l1-period-s 1..30`.
The existing altitude/airspeed and attitude loops track the mission's altitude,
true airspeed, and bank requests. The CSV adds `lat_deg`, `lon_deg`, derived
`north_m`/`east_m`, ground velocity, `cross_track_m`, and `mission_leg` columns.
A positive cross-track
value means right of the directed line. The runner stops at the final waypoint
and prints `Mission complete`; if `--duration` expires first, it prints
`Mission incomplete` and exits with status 2. The CSV is kept for inspection.

The current route geometry supports straight legs plus per-waypoint fly-by and
fly-over turns. Gust/turbulence studies, mission upload,
takeoff/landing, loss-of-navigation handling, and an onboard safe mode are not
implemented. In particular, the runner stopping at the final waypoint is a
simulation behavior, not an aircraft action. The C172X gains and route are
simulation examples, not settings for the Skyward. The current turn-radius planner is a fixed-speed/bank
approximation that still needs validation over different speeds, winds, route
corners, and aircraft models.

### Steady-wind mission comparison

The JSBSim runner accepts steady wind components in meters/second; positive
values mean the wind blows toward north or east. Wind is applied immediately
after trim, as a step at simulation time zero. The magnitude of the north/east
vector is limited to 20 m/s. This is a controlled simulation disturbance, not
a turbulence model or a Skyward wind limit. Run the same compiled mission in
calm air and with crosswind from both directions:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode mission --mission logs/c172x_line.apm --duration 120 --wind-east-m-s 0 --output logs/mission_calm.csv
./build/host/bin/Debug/autopilot_sim.exe --mode mission --mission logs/c172x_line.apm --duration 120 --wind-east-m-s 5 --output logs/mission_east5.csv
./build/host/bin/Debug/autopilot_sim.exe --mode mission --mission logs/c172x_line.apm --duration 120 --wind-east-m-s -5 --output logs/mission_west5.csv
python tools/mission_metrics.py logs/mission_calm.csv logs/mission_east5.csv logs/mission_west5.csv --output logs/mission_wind_comparison.json
```

`--wind-north-m-s` is available too, including in non-mission modes. CSV logs
record both commanded steady wind components. The metrics tool reports
cross-track RMS/peak error, final-10-second cross-track error, ground-speed
minimum, control saturation counts, and maximum altitude and airspeed errors.
Mission completion comes from the runner's exit status: zero means the final
waypoint was reached, while status 2 means the duration elapsed first.

Historical C172X baseline before nearest-leg capture and the revised terminal
acceptance circle: all three runs completed. Timing below uses that earlier
completion criterion; regenerate metrics when comparing the current code.

| East wind (m/s) | Completion (s) | Cross-track RMS (m) | Peak cross-track (m) | Final 10 s RMS (m) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 71.68 | 2.58 | 22.25 | 0.044 |
| +5 | 71.33 | 2.60 | 22.12 | 0.051 |
| -5 | 72.61 | 2.54 | 22.11 | 0.032 |

The peak is concentrated around the leg transition. Aileron saturation lasted
about 0.34–0.35 s in each run; these experiments do not establish acceptable
limits for the physical aircraft. The ±5 m/s crosswind case is now part of the
JSBSim integration test. Next, vary airspeed, wind direction/magnitude, and
route corner angle before using these results to tune lookahead and plan turns.

## Live FlightGear visualization

FlightGear is a visual display for the same standalone JSBSim C172X simulation;
the C autopilot still computes the controls. Start FlightGear first in one
PowerShell window (adjust the executable path if needed):

```powershell
& 'C:\Program Files\FlightGear 2024.1\bin\fgfs.exe' --launcher=no --aircraft=c172p --fdm=null '--native-fdm=socket,in,50,,5600,udp' --lat=30 --lon=0 --altitude=3000
```

Wait for its scene to finish loading. From the project root in another PowerShell
window, run:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --mode roll-hold --flightgear --duration 60 --output logs/c172x_flightgear.csv
```

The aircraft should bank to +5 degrees at 2 s and return to wings level at 10 s.
`--flightgear` transmits FlightGear native FDM packets to localhost UDP port 5600
at 50 Hz and paces the 100 Hz simulation against wall time. The CSV remains
available for quantitative plots. FlightGear's C172P is a visual stand-in for
JSBSim's C172X; this is not a Skyward model. The socket is one-way: controls in
FlightGear do not drive the C autopilot. Without `--flightgear`, simulation runs
at full speed as before. The localhost stream is unencrypted; keep it local.

## Plotting setup and open-loop comparison

An optional Python script compares the two runs with Matplotlib. This is separate
from the C/C++ build and does not change its dependencies. With Python 3.10+ installed,
run from the project root on Windows:

```powershell
python -m venv .venv
./.venv/Scripts/python.exe -m pip install -r tools/requirements-plot.txt
./.venv/Scripts/python.exe tools/plot_logs.py
```

On Linux/macOS, create the environment with `python3 -m venv .venv` and use
`./.venv/bin/python` for the remaining commands. No activation is required.

Plots are saved as `logs/plots/c172x_comparison.png` and `.svg`. The figure compares
aileron command, body roll rate, bank angle, pitch, true airspeed change, and
altitude change. Displayed angles are converted to degrees; the CSV stays in radians.
Shading marks samples where the experiment's aileron command differs from the
baseline, when timestamps match. Altitude and speed changes use each run's own
initial value. No smoothing or downsampling is applied.

To compare different logs or change labels:

```powershell
./.venv/Scripts/python.exe tools/plot_logs.py --baseline logs/baseline.csv --experiment logs/c172x_pulse.csv --experiment-label "Aileron pulse" --output logs/plots/comparison
```

Rerun the script after generating new CSV files. It overwrites the requested plot
files. The SVG is suitable for scaling in your project report. Use `--help` for options.

The runner embeds the build-time JSBSim data path. To relocate the data, use:

```powershell
./build/host/bin/Debug/autopilot_sim.exe --jsbsim-root "C:/path/to/jsbsim" --duration 5
```

The root must contain `aircraft/`, `engine/`, and `systems/`; copying just the
executable is not sufficient. Initial conditions and property mappings live in
`sim/src/jsbsim_adapter.cpp`; the experiment lives in `sim/src/main.cpp`.

## Interface and next steps

`ap_step(config, input, output)` runs the stateless manual and attitude modes.
`ap_step_with_runtime(config, input, runtime, output)` adds the true-airspeed
PI and altitude modes; its caller owns and initializes `ap_runtime_t`. Inputs include aircraft
state, trimmed controls, timestep, explicit mode, and the active setpoints.
Outputs include bounded actuator commands, effective setpoints, and clipping
flags. Invalid samples, configuration, modes, and arithmetic overflow return
`false` without changing output or runtime; the runner terminates. This is an
interface contract, not a flight failsafe implementation. Mode transitions are
immediate, with no reference ramp or bumpless-transfer mechanism yet.

Attitude uses radians relative to local North-East-Down; body axes are forward,
right, down. Altitude is meters above mean sea level, positive up. The adapter
owns JSBSim's feet-to-meters conversion. Commands map to the C172X `fcs/*-cmd-norm`
properties; future board drivers must define physical servo directions and travel.

Next: expand the mission matrix to changed speed, wind direction, lookahead
period, sharper route corners, and mixed fly-by/fly-over sequences. Add explicit
mission-end actions and failure handling, then replace simulator truth with
estimator outputs and introduce a Skyward model.
This project does not yet validate hardware for flight.

Tests cover manual passthrough, feedback/damping direction, command and actuator
limits, PI anti-windup, altitude and mission guidance, mission decoding,
invalid inputs, overflow, and mode switching. JSBSim
integration tests cover trim drift, SI conversion, the open-loop aileron
response, independent and combined attitude loops, true-airspeed steps,
altitude steps with a banked segment, and a two-leg mission.
After splitting the C modules, the manual-run CSV matched the prior run exactly;
the roll-run state and command differences from floating-point evaluation order
were below 0.000004 degrees of bank and 0.0000002 normalized aileron command.

Plot-metric tests are separate from CMake so Python remains optional:

```powershell
./.venv/Scripts/python.exe tests/test_plot_metrics.py
```

They check overshoot direction, settling after leaving and reentering the band,
and explicit reporting of an unsettled response. These are software/model checks,
not validation against a physical aircraft.

The separate mission-compiler test uses only the Python standard library:

```powershell
python tests/test_mission_compile.py
```

## References

- [JSBSim source and aircraft data](https://github.com/JSBSim-Team/jsbsim)
- [JSBSim reference manual](https://jsbsim-team.github.io/jsbsim-reference-manual/)
- [Small Unmanned Aircraft companion material](https://github.com/byu-magicc/mavsim_public)

JSBSim retains its upstream LGPL license; see `COPYING` in
its source tree when distributing builds that include it.

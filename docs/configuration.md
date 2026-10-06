# Binary aircraft configuration

The control library takes configuration in RAM. It never opens a file, reads
EEPROM, or writes hardware. The same portable C codec can decode bytes from a
PC file now or a firmware storage driver later. There is no JSON configuration
parser and no text stored in the parameter payload.

## Use on the PC

Build with `cmake --build --preset host --parallel`. The host build produces
`build/host/bin/Debug/autopilot_config.exe` alongside `autopilot_sim.exe`.
The repository includes `configs/c172x.apcf`, containing the original C172X
simulation profile, sequence 0, with `max_aileron=0.5`. The current compiled
defaults use `max_aileron=1.0`; `create` uses those compiled defaults, while
loading an existing file preserves its saved values. Neither is a Skyward profile.

```powershell
# Inspect the supplied binary record.
./build/host/bin/Debug/autopilot_config.exe show configs/c172x.apcf

# Create a new record from the C172X simulation defaults.
./build/host/bin/Debug/autopilot_config.exe create logs/my_c172x.apcf

# Edit a record into a new file. Parameter names match the C API; angles are radians.
./build/host/bin/Debug/autopilot_config.exe set configs/c172x.apcf logs/c172x_tuned.apcf roll_attitude_gain=2 roll_rate_kp=2 roll_rate_ki=1 max_roll_rate_rad_s=1 l1_period_s=5

# Run using the saved parameters.
./build/host/bin/Debug/autopilot_sim.exe --config logs/c172x_tuned.apcf --mode roll-hold --output logs/configured_roll.csv

# Optional one-run override. Does not edit the saved file.
./build/host/bin/Debug/autopilot_sim.exe --config logs/c172x_tuned.apcf --mode roll-hold --roll-kp 4 --output logs/configured_override.csv
```

`create` also accepts `NAME=VALUE` assignments. `show` lists all editable names.
`set` validates the complete edited record and increments its sequence number.
It rejects an exhausted sequence (`UINT32_MAX`) instead of wrapping it. Writers
refuse existing output paths, including editing a file in place; choose a new
filename to retain the previous record. The host writer checks stream errors,
but does not implement crash-atomic replacement, concurrent writes, or EEPROM
transactions. A partially written new file fails validation when loaded.

The simulator uses built-in C172X defaults when `--config` is absent. When it is
present, the file must be valid; errors do not silently fall back to defaults.
Explicit gain and `--l1-period-s` options override file values regardless of
argument order, subject to the existing mode restrictions. The effective
configuration is validated before starting JSBSim or opening the output CSV.

The profile changes controller/guidance parameters, not the simulated aircraft:
the runner still loads C172X, uses a 100 Hz loop, and starts at its existing
3000 ft / 100 kt condition. Initial speed/wind remain experiment options;
waypoints, target altitude, and target speed remain mission data. JSBSim still
provides trim. Hardware mixing, sensor settings, and failsafe behavior are not
implemented by this format. Numeric validation is not flight-envelope validation.

## APCF version 2 format

APCF v2 is exactly **96 bytes**, with no compiler padding. Multi-byte integers and IEEE-754
binary32 floats are little endian. The target must support 8-bit bytes and
IEEE-754 binary32 `float`. Do not save a C struct's memory image.

| Byte offset | Size | Encoding | Meaning |
| --- | --- | --- | --- |
| 0 | 4 | ASCII `APCF` | Magic identifier |
| 4 | 2 | uint16 | Format version: 2 |
| 6 | 2 | uint16 | Payload length: 80 |
| 8 | 4 | uint32 | Sequence number |
| 12 | 80 | 20 float32 values | Parameters below, in order |
| 92 | 4 | uint32 | CRC32 of bytes 0 through 91 |

CRC32 is CRC-32/ISO-HDLC, with reflected polynomial `0xEDB88320`, initial value
`0xFFFFFFFF`, and final XOR `0xFFFFFFFF` (compatible with Python `zlib.crc32`).
It detects corruption; it is not authentication. Sequence is storage metadata
and does not affect the control equations.

| Payload index | Parameter | Units / allowed numeric range |
| --- | --- | --- |
| 0 | `roll_attitude_gain` | 1/s; > 0 |
| 1 | `roll_rate_kp` | normalized effort/(rad/s); > 0 |
| 2 | `max_bank_rad` | rad; > 0 and < pi/2 |
| 3 | `max_aileron` | normalized magnitude; > 0 and <= 1 |
| 4 | `pitch_attitude_gain` | 1/s; > 0 |
| 5 | `pitch_rate_kp` | normalized effort/(rad/s); > 0 |
| 6 | `max_pitch_rad` | rad; > 0 and < pi/2 |
| 7 | `max_elevator` | normalized magnitude; > 0 and <= 1 |
| 8 | `airspeed_kp` | normalized throttle/(m/s); >= 0 |
| 9 | `airspeed_ki` | normalized throttle/((m/s)*s); > 0 |
| 10 | `min_throttle` | normalized; >= 0 and < 1 |
| 11 | `max_throttle` | normalized; > min_throttle and <= 1 |
| 12 | `altitude_gain` | rad/m; > 0 |
| 13 | `climb_rate_gain` | rad/(m/s); >= 0 |
| 14 | `max_pitch_offset_rad` | rad; > 0 and < pi/2 |
| 15 | `l1_period_s` | seconds; 1 through 30 |
| 16 | `roll_rate_ki` | normalized effort/rad; >= 0 |
| 17 | `max_roll_rate_rad_s` | rad/s; > 0 |
| 18 | `pitch_rate_ki` | normalized effort/rad; >= 0 |
| 19 | `max_pitch_rate_rad_s` | rad/s; > 0 |

All fields must be finite. Header values, exact record length, CRC, and parameter
bounds are checked. Unknown versions are rejected; the supported v1 migration is described below. The wire order
is explicit in the codec; changing the in-memory layout does not change this
format. Adding or changing wire fields requires a new format version.

## Legacy APCF v1 migration

The decoder also accepts the original 80-byte v1 layout and CRC. With positive
old roll and pitch damping gains, it derives `attitude_gain = old_angle_gain /
old_rate_gain` and `rate.kp = old_rate_gain`. Both new rate integrals start with
`ki = 0`; roll and pitch acquire target limits of 1 and 0.5 rad/s respectively.
Below those rate limits, with zero integral state, the proportional actuator law
matches the previous angle-P plus body-rate damping law. Above them, the new
rate limits intentionally change the demand.

V1 records with zero damping cannot define this rate-feedback cascade and are
rejected without changing the destination. Unknown versions are also rejected.
Stored `configs/c172x.apcf` remains an original v1 example; decoding does not
rewrite it. Use `autopilot_config set` with explicit new fields to save a v2
record. The tool's field names are the v2 names in the table; old `*_angle_gain`
names are rejected so their different units cannot silently change meaning.

## Library and firmware boundary

`autopilot/include/autopilot/config.h` declares:

```c
bool ap_config_validate(const ap_aircraft_config_t *config);
bool ap_config_encode(const ap_aircraft_config_t *config,
                      uint8_t *bytes, size_t capacity);
bool ap_config_decode(const uint8_t *bytes, size_t length,
                      ap_aircraft_config_t *config);
```

The record contains `.control` (`ap_config_t`), `.l1_period_s`, and `.sequence`.
The in-memory `.control` groups settings into `.roll`, `.pitch`, `.airspeed`,
`.altitude`, and `.attitude_limits`. Each attitude axis composes `.attitude` and
`.rate`. For example, `roll_attitude_gain` maps to `.control.roll.attitude.gain`,
and `roll_rate_ki` maps to `.control.roll.rate.ki`. The codec uses explicit field
order rather than raw struct storage.
Both encoder and decoder leave their destination unchanged on failure. They
use caller-owned buffers and no heap, operating-system, or EEPROM dependencies.
The encoder writes exactly 96 bytes; the decoder accepts exactly one v2 or supported v1 record.
Input and output must not overlap.

`sim::load_config()` and `sim::save_config()` are PC filesystem adapters around
those C functions. Firmware must supply its own EEPROM driver and load/save
policy. At boot, read the record, decode into a candidate, and install accepted
settings into RAM. Pass `.control` and a zero-initialized `ap_runtime_t` to `ap_step()` and `.l1_period_s`
to `ap_mission_step()`. Do not access EEPROM on each control tick.

Future firmware must also define what happens when neither stored record is
valid, how active configurations switch between ticks, and how to handle
controller state when gains change. For interrupted-write protection, implement
and test two storage slots, write/read-back verification, and record selection
for the actual EEPROM. The sequence field supports that future policy; this
change does not implement board firmware, EEPROM slot selection, or takeover.

## Verification

`ctest --preset host` runs C codec tests and, when Python 3 is found, the host
configuration integration test. `ctest --preset core` tests the codec without
C++ or JSBSim. To run the host integration test explicitly:

```powershell
./.venv/Scripts/python.exe tests/test_config_file.py --tool build/host/bin/Debug/autopilot_config.exe --sim build/host/bin/Debug/autopilot_sim.exe
```

Tests cover an independent binary-format oracle, round trips, corrupt and
unsupported records, nonfinite/out-of-range values, unchanged outputs on
failure, preserved input files, sequence exhaustion, simulator gain/limit
application, CLI precedence, and mission L1 configuration.

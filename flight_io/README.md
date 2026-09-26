# Flight I/O

`flight_io` is a portable C11 library for the proposed STM32F405 I/O controller.
It decodes RC input, calibrates logical controls, selects control authority, and
calculates actuator pulse-width demands. It also builds and runs on the PC.
There are no STM32 HAL calls, registers, OS calls, heap allocation, hidden clocks,
or dependencies on the autopilot/estimator library.

The H723 and F405 firmware applications are also part of this project. Below,
"firmware responsibility" identifies where we will implement a feature. It does
not place that feature outside the project. Those MCU targets are not built yet.

## Layout and dependencies

```text
common/include/flight_common/control.h    Shared in-memory types (Flight::Common)
             ^                   ^
             |                   |
     autopilot/              flight_io/
     Autopilot::Core         Flight::IO
     guidance/control        RC/authority/output mapping
             |                   |
     future H723 firmware    future F405 firmware
```

Both libraries depend on the small C11 `Flight::Common` target.
Neither depends on the other. `ap_controls_t` remains available as a typedef of
`flight_controls_t`, so existing simulation, control, and binary-configuration
code keeps its current API. A main-controller result can be assigned directly
to a `flight_control_sample_t.controls` member.

`Flight::Common` also supplies the shared binary flight logger and firmware sink
interface in `flight_common/log.h`. Applications can log both MCUs with the same
format and decode flash dumps to CSV; see [flight logging](../docs/flight_logging.md).
The algorithms do not perform hidden logging or flash writes.

| Module | Public API | Purpose |
| --- | --- | --- |
| `ibus` | `fio_ibus_decode`, `fio_ibus_feed` | Complete-frame or streaming iBUS decoding |
| `rc` | `fio_rc_config_valid`, `fio_rc_map` | Channel mapping, calibration, reversal, and mode selection |
| `supervisor` | `fio_supervisor_config_valid`, `fio_supervisor_step` | Select disarmed, manual, autopilot, or fixed failsafe outputs |
| `actuators` | `fio_actuator_config_valid`, `fio_actuators_map` | Configurable mixing and pulse-width calculation |

Public headers are under `include/flight_io/`; implementations are under `src/`.
Caller-owned parser and supervisor runtime objects must be initialized to `{0}`.

## One I/O cycle

1. Firmware receives UART bytes and supplies their local reception timestamps to
   `fio_ibus_feed()`. A complete checksum-valid frame yields channel data.
2. Call `fio_rc_map()` on that new frame. Publish the resulting RC sample even
   when its `valid` field is false; do not preserve an older valid sample instead.
3. Firmware accepts new main-controller messages into a
   `flight_control_sample_t`, setting `received_at_ms` from the F405's own clock.
4. Call `fio_supervisor_step()` periodically, supplying the current F405 time,
   firmware arming permission, and the latest RC/AP samples.
5. Call `fio_actuators_map()` on the selected logical controls to obtain physical
   output pulse widths. Firmware applies its output-enable policy and programs
   the timers. Status can be sent back to the H723 and logged.

Manual selection never calls `ap_step()` and requires no estimated aircraft
state. If an estimator fails while the pilot requests MANUAL, the independent
RC path still works, provided the I/O controller and RC inputs are healthy.

## Receiver decoding and calibration

The implemented iBUS channel frame is 32 bytes: `0x20 0x40`, fourteen little-endian
channel words, and a 16-bit subtractive checksum. This is the channel transport,
not the iBUS sensor/telemetry protocol. Firmware owns UART setup (115200, 8N1),
pin electrical compatibility, byte reception, and buffering. Fourteen transport
slots do not mean the FS-i6X/FS-iA10B supplies fourteen usable controls.

The decoder checks the checksum and header, extracts the lower twelve channel
bits, and recognizes the channel-1/channel-4 high-nibble loss indications also
handled by [ArduPilot's iBUS decoder](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_IBUS.cpp).
Absence of those bits is not proof of RF health on every receiver version.
The stream parser recovers from noise/dropped bytes and abandons a partial frame
after an inter-byte gap greater than 5 ms. Firmware must supply reception times
rather than pretending queued old bytes arrived on the current tick.

`fio_rc_config_t` explicitly maps five distinct, zero-based transport slots:
aileron, elevator, rudder, throttle, and the authority switch. Each has minimum,
center, maximum, and reversal settings. Surface calibration uses separate scales
on either side of center. Throttle maps minimum..maximum to 0..1.

Values outside calibrated endpoints invalidate the sample; include measured
endpoint tolerance in the configuration. Unassigned channel values are ignored.
The normalized switch selects MANUAL at <= -0.5 and AUTO at >= +0.5. Its middle
band is invalid; it does not select a third flight mode. Reversal changes the
switch direction. There are no assumed channel assignments or flight defaults.

`fio_rc_map()` also requires `firmware_link_ok`. This is an explicit integration
gate, not a receiver driver supplied by this library. Determine the real
receiver's transmitter-off behavior on the bench; implement the verified loss
indication (for example, a tested dedicated failsafe marker) before treating this
gate as reliable. Fresh serial traffic alone cannot establish a live RF link.
Recognized iBUS failsafe flags always invalidate RC regardless of that gate.

## Authority policy

The initial policy is for supervised experiments with a required RC link. It
does not implement autonomous continuation after RC loss, return-to-home, loiter,
or landing. `failsafe_controls` are explicit, fixed application-supplied outputs;
the library makes no claim that one set is suitable for every aircraft/failure.

Decisions are evaluated in this order:

| Condition | Selected authority/output | AUTO latch |
| --- | --- | --- |
| Firmware says disarmed | Configured disarmed controls (logical throttle must be zero) | Cleared |
| RC missing, invalid, or stale | Configured fixed failsafe controls | Cleared |
| Healthy RC requests MANUAL | Pilot controls | Eligible for next AUTO request |
| AUTO requested but AP missing, invalid, or stale | Pilot controls; AP-unavailable reason | Cleared |
| AUTO requested without prior healthy armed MANUAL observation | Pilot controls; re-engagement-required reason | Remains cleared |
| AUTO requested, eligible, AP healthy | Autopilot controls | Remains eligible |

The status reports authority and reason separately; a manual fallback is
distinguishable from an intentional manual request. After a detected fault,
fresh AP messages alone do not re-engage AUTO. The pilot must request MANUAL
and then AUTO. The H723 application must initialize its integrator and targets
appropriately before presenting a valid command for re-engagement.

Samples expire when age is **greater than** the configured timeout. Timeout
values must be 1..INT32_MAX milliseconds. Null sample pointers mean unavailable;
nonfinite/out-of-range logical commands also mean unavailable. Local unsigned
millisecond subtraction supports one counter wrap. Do not retain a sample for
a full 2^32-ms wrap or use clocks from different processors for age checks.

`armed` is permission supplied by firmware, not an implemented arming protocol.
Disarmed logical output is not electrical motor isolation. Firmware must verify
mapping and enforce output inhibition where appropriate.

## Actuator mapping

Each configured physical output is a row of four logical-control weights plus
a bias. Signed weights provide reversal and mixing. The sum is limited to
[-1,1] and mapped to minimum/neutral/maximum pulse width in microseconds, with
nearest-microsecond rounding. `saturated_mask` identifies clipped rows.

This supports up to eight output demands, including paired ailerons, elevons,
or V-tail arrangements when their coefficients are configured appropriately.
For a conventional throttle row, a throttle weight of 1 and neutral equal to
minimum maps 0..1 onto minimum..maximum. No mapping is enabled by default.

The same mapping applies to pilot and autopilot controls. Establish consistent
logical signs first: in the shared interface, positive aileron requests right
roll and negative elevator requests nose-up. Receiver calibration and servo
direction are different transformations; verify their combined effect.

The result contains pulse-width demands, not electrical PWM. Output channel
pins, repetition frequency, timer setup, enable/inhibit, ESC protocol, and power
remain firmware/board responsibilities.

## Errors and the future firmware link

Decoder/config/API errors return false without modifying the output. Stream
parsing advances its state even when no new frame is produced. RC health loss
returns a sample with `valid=false`; a supervisor fault returns a successful
decision with explicit fallback status. Actuator mapping rejects invalid input
or arithmetic overflow without modifying its output. Firmware must handle false
returns deliberately rather than replaying old autonomous output indefinitely.

The shared structs are **in-memory API types, not a serial protocol**. Do not
transmit `sizeof(struct)` bytes. The H723/F405 firmware link still needs an
explicit versioned encoding, framing, lengths, integrity check, sequence/session
handling, and rejection of duplicate/out-of-order messages. Only accepted NEW
messages refresh the local sample timestamp. A detected main-controller reboot
must clear the AUTO latch even if it happens faster than the command timeout.
Neither clock synchronization nor a link codec is implemented here.

`APCF` version 1 is unchanged and still stores only autopilot/guidance settings.
RC calibration, I/O timeouts, actuator mapping, and failsafe outputs are separate
I/O configuration structures; their persistent encoding is future work.

No F405/H723 firmware, watchdog servicing, receiver electrical interface, or
power-failure independence is implemented. A second MCU only provides an
independent path if board power/reset design and its firmware support it.

## Build and validation

```powershell
cmake --preset core
cmake --build --preset core --parallel
ctest --preset core

cmake --preset host
cmake --build --preset host --parallel
ctest --preset host
```

Link a firmware/application target to `Flight::IO`. Its only project dependency
is `Flight::Common`. `cmake --build --preset core --target flight_io` builds the
I/O library without compiling the autopilot library, C++, or JSBSim.

The four C tests link only `Flight::IO` and cover binary receiver frames,
malformed/truncated input, dropped-byte resynchronization, channel mapping,
receiver-loss gating, stale commands, timestamp wrap, manual independence,
re-engagement latching, output mixing, saturation, and invalid inputs.

`tests/io_integration_test.cpp` runs both libraries against the C172X in JSBSim:
manual engagement, AUTO, pilot override during an invalid estimator sample,
stalled H723 commands, resumed commands with AUTO locked out, and deliberate
MANUAL->AUTO re-engagement. It uses synthetic RC samples and a simulated link,
not physical iBUS, MCU timing, or hardware-in-the-loop. The normal simulation
runner continues to call the autopilot directly; no RC CLI was added.

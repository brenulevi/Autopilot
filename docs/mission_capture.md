# Mission entry in JSBSim

Engaging a mission selects the nearest **finite route segment**, allowing earlier
waypoints to be skipped. Waypoint 0 remains the geodetic reference origin; the
aircraft does not have to visit it. Distance is measured to the segment including
its endpoints, not its infinite extension. Distances within 1 cm are ties:
prefer alignment with ground course, then the lower leg index.

Selection happens once on the first call with a zero-initialized
`ap_mission_runtime_t`. The mission subsequently advances in order. Preserve
runtime to resume the same leg; reset it to select again from current position.
This prevents repeatedly jumping between nearby parallel legs.

`autopilot/src/mission/mission.c` implements selection and guidance;
`autopilot/src/mission/path.c` implements the entry geometry. This stage runs entirely
in JSBSim using simulator truth, with the same C APIs available for later firmware.

## Why Dubins appears here

If already within the leg's capture corridor, on the finite segment, and within
30 degrees of its forward course, the aircraft continues with line guidance.
Otherwise it plans a **Dubins entry** from current position/course to a merge
point on the selected leg, arriving in the leg's forward direction.

The planner evaluates all six combinations of straight sections and circular
arcs in the planar, forward-motion, bounded-curvature problem described in
[LaValle's Planning Algorithms](https://msl.cs.illinois.edu/~lavalle/planning/node821.html).
In our north/east coordinates, course increases clockwise from north and positive
curvature means a right turn.

The radius is fixed at engagement:

```text
radius = 1.6 * ground_speed² / (g * tan(max_bank))
```

The 1.6 factor provides margin for roll response and modest wind in these
experiments. This groundspeed-based geometry is a planning approximation, not
an exact wind/aircraft-dynamics trajectory. Feedback uses current ground velocity
and bank commands remain limited by the controller configuration.

The merge point is two radii ahead of the aircraft's projection, clamped to the
selected segment with a final straight section reserved. The reserve is the
smaller of L1 lookahead and one quarter of leg length. The planner minimizes
distance **to this chosen position and course**; it does not optimize the merge
point along the whole route. Some entries therefore contain large loops when
the selected leg leaves insufficient room for a short approach.

Guidance follows the entry arcs and straight section with a moving lookahead,
then returns to line guidance. Arc progress is unwrapped between samples. Call
frequently enough to resolve less than half a turn between samples; JSBSim uses
100 Hz. Restart selection after a discontinuous position jump. Keep the mission
and speed/bank settings fixed during the planned entry.

Existing line guidance also now handles targets behind the aircraft. Previously
the sine-only steering expression could request zero bank at exactly 180 degrees.
A recovery latch retains the turn direction until the target is ahead.

## Status and arrival

Simulator CSV adds `mission_phase` and `mission_target_distance_m`:

| Phase | Meaning |
| --- | --- |
| 0 | Entry path active, or outside the line-capture criteria |
| 1 | Tracking the selected route leg |
| 2 | Mission completed |
| 3 | Fly-by turn connector active |
| 4 | Fly-over passage/turn connector active |

`mission_leg` is zero-based. Target distance is horizontal distance to the leg's
ending waypoint. Cross-track error remains relative to the route leg, including
during entry; a large value during entry is not the tracking error to its arc.
The main runner exits on completion before another control row. The capture
test's CSV includes the completion sample.

The corridor is `max(20 m, min(lookahead/2, 100 m))`. The diagnostic tracking state
requires alignment within 60 degrees and proximity to the finite leg. The stricter
30-degree criterion above decides whether engagement requires a planned entry.
Intermediate legs retain the existing early switching. Final arrival now uses
the circle `max(20 m, min(lookahead/4, 50 m))`, including an exact endpoint sample.
It does not require crossing the terminal plane or verify altitude/speed arrival.

## Run and inspect

From the repository root:

```powershell
python tools/compile_mission.py missions/c172x_line.json logs/c172x_line.apm
./build/host/bin/Debug/autopilot_sim.exe --mode mission --mission logs/c172x_line.apm --start-lat-deg 30.0044966 --start-lon-deg -0.010383 --start-heading-deg 0 --duration 400 --output logs/capture_west.csv
```

This starts roughly 500 m along and 1 km west of the first leg. Heading is true
heading in [0,360), not ground course in wind. Add `--flightgear` after starting
FlightGear as described in the root README. Omitted initial-position options
preserve the adapter's previous defaults and non-mission experiments.

The former 500 m distance-to-waypoint-0 restriction is removed. The existing local
projection still requires north/east each within 50 km of mission origin, with
prepared waypoints each within 20 km. Capture is not guaranteed for every
position, speed, wind, or route, and planned loops can require substantial space.

`mission_capture_test.cpp` checks eight starts: west/east offsets, behind the
start and flying away, opposite course, a later leg, past the finish, and east/
west 5 m/s winds. Each must select its expected leg, sequence forward, join and
complete within 600 simulated seconds, respect commanded bank limits, and stay
inside the test's attitude/altitude/airspeed bounds. Actual and planned paths go
to `build/host/mission_capture_logs/`.

```powershell
ctest --preset host -R mission_capture -V
./.venv/Scripts/python.exe tools/plot_mission_capture.py build/host/mission_capture_logs --output logs/plots/mission_capture
```

The existing Matplotlib environment produces PNG/SVG comparisons. Core tests
exercise all six winning Dubins families over a position/course grid and
independently integrate curvature, plus entry selection, ties, reverse-course
steering, malformed inputs, and endpoint acceptance.

## Fly-by and fly-over

Fly-by allows turning before a waypoint; fly-over requires reaching it before
turning toward the next leg. These are waypoint semantics, as illustrated in the
[FAA explanation](https://www.faa.gov/Air_traffic/publications/atpubs/aim_html/chap1_section_2.html).
Dubins constructs geometry; tangent circular arcs are another option for suitable
fly-by corners.

APM3 adds a per-waypoint type word while retaining APM2 compatibility. APM2
missions decode as fly-by waypoints. Fly-by uses a tangent circular fillet and
starts the turn before the waypoint. Fly-over requires the aircraft to reach the
waypoint passage condition before the connector to the next leg starts. A missed
fly-over is recaptured instead of silently advancing.

The turn radius is selected once on engagement from the configured bank limit and
the maximum mission/entry speed. If a fly-by cannot fit its adjacent legs at that
radius, mission engagement fails rather than silently squeezing the geometry.
This is a deterministic teaching implementation; it is not yet a wind-optimal
or roll-rate-constrained planner. With nearest-leg engagement, earlier waypoints
are deliberately bypassed.

The current tests cover mixed fly-by/fly-over routes, left and right turns,
crosswind, missed fly-over recovery, and infeasible fly-by rejection. They do not
yet cover a broad aircraft-speed/wind/route matrix or noisy navigation inputs.

There is no obstacle/geofence checking, global path optimization, roll-rate-
constrained planning, altitude-profile planning, or wind-optimal geometry yet.
The terminal action still stops the simulation; mission-end loiter follows turn
management in the simulation roadmap.

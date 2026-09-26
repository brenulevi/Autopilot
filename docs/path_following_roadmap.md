# Path-following roadmap and simulation service

This document records the current assessment of the waypoint path follower and
the next work to review before moving toward physical-aircraft testing.

## Current state

The JSBSim implementation is in a good state for a student-level guidance
baseline. The current host validation passes 13/13 core tests and 26/26 host
tests. It supports:

- nearest finite-leg capture when the mission is engaged;
- Dubins entry paths for difficult starting positions and headings;
- L1 line following with bounded bank commands;
- APM2 compatibility and typed APM3 missions;
- fly-by turns using tangent-radius geometry;
- fly-over turns that wait for waypoint passage;
- mixed fly-by/fly-over routes;
- rejection of fly-by geometry that cannot fit the adjacent legs; and
- CSV logs and post-flight metrics for simulation runs.

This is a validated C172X simulation baseline. It is not yet a general
aircraft controller or a flight-ready FCS.

## Pending path-following work

### 1. Build a repeatable validation matrix

Run the same missions across:

- straight legs, 90-degree, 135-degree, and near-reversal corners;
- mixed fly-by and fly-over sequences;
- short and long legs;
- starts on the route, off the route, behind the selected leg, and near a later leg;
- several commanded airspeeds;
- calm air, headwind, tailwind, and crosswind from both directions; and
- different L1 periods and bank limits.

Record at least:

- mission completion and failure reason;
- RMS and peak cross-track error;
- capture time;
- minimum groundspeed;
- maximum bank and actuator saturation;
- altitude and airspeed error; and
- phase and waypoint transition timing.

Define acceptance limits before comparing aircraft models. A test that merely
completes the mission is not enough if it takes a large loop, saturates the
actuators, or loses too much altitude.

### 2. Improve turn-planner robustness

The current turn radius is selected once from speed and bank limit. It is a
useful deterministic approximation, but it does not yet model:

- roll-rate limits and roll response;
- servo travel and actuator bandwidth;
- changing airspeed during a turn;
- strong or changing wind; or
- wind-optimal route geometry.

Some difficult captures can therefore produce large loops. Keep the current
behavior deterministic while measuring this envelope. Later, decide whether the
planner needs speed-dependent re-planning, a roll-dynamics constraint, or a
different turn policy.

### 3. Complete mission state behavior

Still to define and test:

- loiter or hold after the final waypoint;
- pause, resume, abort, and restart semantics;
- navigation-loss behavior;
- missed fly-over recovery limits;
- explicit mission failure states; and
- safe transitions between manual, assisted, and mission control.

The simulator currently stops at mission completion. That is useful for a test,
but it is not an aircraft action.

### 4. Add vertical and energy-path behavior

Horizontal turn geometry is implemented, while altitude and airspeed are still
primarily waypoint setpoints. Future work should define how the aircraft should
manage altitude, speed, climb rate, and turn transitions instead of treating
each waypoint as an independent command step.

### 5. Add estimator-shaped inputs

The current guidance tests use simulator truth. Before hardware testing, add
GPS/IMU noise, delay, dropout, and bounded invalid-data cases, then feed the
guidance code through the same state interface that the estimator will provide.

This should follow the clean truth-state guidance baseline so estimator errors
do not hide basic path-following mistakes.

### 6. Create and validate the Skyward model

The C172X model is a development vehicle. The Skyward work still requires:

- a JSBSim aircraft model;
- trim and control-surface direction checks;
- aircraft-specific gains and limits;
- a new speed, wind, and route validation matrix; and
- comparison of the same guidance metrics between C172X and Skyward.

The portable C guidance code should remain aircraft-independent. Aircraft
configuration, gains, limits, and model-specific mappings belong at the
configuration and adapter boundaries.

## Proposed simulation and mission web service

The web-service idea is a good next layer, as long as it orchestrates existing
tools instead of moving flight-control logic into a server. The service can
become a repeatable experiment system for JSBSim and FlightGear:

1. A user selects an aircraft configuration, mission, starting state, wind,
   controller parameters, and simulator backend.
2. The service validates the inputs and creates an immutable run specification.
3. A worker runs JSBSim or FlightGear with that specification.
4. The worker stores raw telemetry and derived metrics.
5. The UI displays the route, aircraft track, waypoint phases, errors, control
   commands, and completion/failure reason.
6. Runs can be compared using the same mission and different aircraft,
   controllers, or disturbances.

The service should call the C autopilot library through a small simulator
adapter. It should not become the runtime dependency of the future H723
firmware. The firmware must still run from compiled code, local configuration,
and onboard sensors when disconnected from the service.

## Suggested first service boundary

Use files and command-line workers first, then place an API around them. A
minimal run specification could contain:

```text
aircraft_config:  skyward_v1.apcf
mission:         route_001.apm
backend:         jsbsim
initial_state:   position, heading, altitude, airspeed
environment:     wind and optional disturbances
controller:      gains, limits, L1 period, turn settings
duration_s:      simulation duration
```

Useful initial API concepts are:

- `POST /aircraft-configs` — validate and store a versioned configuration;
- `POST /missions` — validate and compile an editable mission source;
- `POST /runs` — create a reproducible simulation run;
- `GET /runs/{id}` — return status, logs, and metrics;
- `GET /runs/{id}/telemetry` — stream or download CSV/columnar telemetry; and
- `GET /runs/{id}/replay` — provide the route and state needed by a map/replay UI.

The first implementation can use the existing compiler, simulator executable,
CSV logger, and metric scripts as workers. A database and browser UI can come
after the run format is stable.

## Service design rules

- Version aircraft configurations, mission files, controller parameters, and
  simulator builds.
- Store the exact run specification beside every result.
- Keep raw telemetry immutable and compute derived metrics separately.
- Make every run reproducible from its specification and simulator version.
- Treat FlightGear visualization as a view of a run, not as the source of
  truth for automated metrics.
- Never allow an unvalidated remote configuration to be sent directly to a
  physical aircraft.

## Recommended order

1. Finish the JSBSim validation matrix and acceptance limits.
2. Add mission completion, failure, and estimator-shaped test cases.
3. Build a local run-specification file and worker command around the current
   simulator.
4. Add a small results database and telemetry/replay viewer.
5. Add the Skyward JSBSim model and compare it with C172X.
6. Only then connect the same versioned configuration concepts to firmware
   upload, logging retrieval, or physical-aircraft operations.

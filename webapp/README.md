# Autopilot mission planner

Local Node.js / TypeScript web tool with Leaflet waypoint editing, APM export,
shared C route geometry, and UDP → WebSocket flight telemetry. No Python runtime.

## Start

Requires Node.js 22.12+ (24 LTS recommended), npm, CMake 3.22+, and a native C compiler.
From this directory:

```bash
npm install
npm run dev
```

Open http://127.0.0.1:3000. If that port is occupied, use `PORT=3001 npm run dev`
on Linux/macOS. The VS Code launch **Webapp: mission planner** uses port 3001.
For a production build served locally:

```bash
npm run build
npm start
```

The server and UDP receiver bind only to `127.0.0.1`. `PORT` selects the HTTP port;
`TELEMETRY_PORT` selects the UDP port (default 5601). No database or account is required.
Online OpenStreetMap tiles require an internet connection and retain their attribution.
The editor and geometry still work when tiles are unavailable. Do not bulk-download
these public tiles; use a suitable tile provider for an offline deployment.

## Plan a mission

- Click the map in **Add waypoint** mode. Drag numbered markers to move them.
- Select a waypoint to edit latitude, longitude, altitude **MSL in meters**,
  **true airspeed in m/s**, and fly-by/fly-over behavior. Arrow buttons reorder it.
- The **S** marker sets the airborne start position. Configure initial **ground
  course**, ground speed, maximum bank, and L1 period in Aircraft & preview.
- **Save JSON** exports the editable APM3 mission. **Import JSON** accepts the
  existing APM2/APM3 JSON examples. The current valid draft and preview settings
  are stored in browser local storage; mission JSON contains only mission data.
- **Download APM** compiles on the Node server and validates with the real C decoder.
  Binary validity does not imply turn feasibility; check the preview separately.

Mission limits match the existing compiler: 2–32 waypoints, at least 1 m between
neighbors, ±20 km in each local horizontal axis, MSL altitude −500..10000 m, and
TAS 5..100 m/s. The first waypoint defines the projection origin.

## Route preview

`native/preview.c` links the repository's actual mission decoder, Dubins planner,
path sampler, and extracted leg/fly-over geometry functions. It receives the APM
on stdin, uses `ap_mission_step()` to select the entry leg and capture path, and
returns sampled coordinates to Node. No path-planning algorithm is duplicated in JS.
The helper is built into `.native-build/bin/mission_preview`.

The radius is fixed at engagement:

```
1.6 × max(entry ground speed, all mission airspeeds)² / (9.80665 × tan(max bank))
```

Green shows straight legs and turns. Amber shows Dubins entry capture. Dashed
lines connect the original waypoints. Nearest-leg selection may skip earlier
waypoints. Fly-by tangent distances must fit 45% of both adjacent legs; the
preview reports infeasible geometry and clears the previous result.

This is a **horizontal geometric preview**. Fly-over connectors assume exact
waypoint passage on the inbound course at commanded speed; the runtime uses its
actual passage state. Wind, roll dynamics, tracking errors, vertical flight
profiles, terrain clearance, and obstacles are not modeled by this preview.
The APM stores waypoints, not sampled curves or bank settings. Use matching bank,
L1, and starting conditions in the simulator. JSBSim `--start-heading-deg` is
heading, whereas the preview uses ground course; these differ in wind. Initial
`--airspeed-kts` is CAS, whereas the preview's start speed is ground speed.

## Live simulation telemetry

### VS Code launch

With the webapp already running, select **Telemetry: mission to web map** in
**Run and Debug** and press **F5**. It builds the simulator and flies the saved
`logs/mission.apm` in real time for up to 600 seconds. Download an APM from the
map and save it at that path first. Telemetry is sent to UDP 5601; the open map
connects automatically. Flight data is written to `logs/mission.csv`.

For both views at once, select **Webapp + FlightGear + mission**. This starts the
map, FlightGear on UDP 5600, and the same simulator process. The simulator sends
Native FDM packets to FlightGear and JSON telemetry to the map on UDP 5601. The
FlightGear launch uses the current mission origin; update its `--lat` and `--lon`
alongside the simulator start arguments if you move the mission.

To start both the webapp and telemetry from a stopped state, select the compound
**Webapp + mission telemetry**. It opens the map at http://127.0.0.1:3001.
Use the standalone telemetry launch when a server is already running on that port.
Stop the launch to stop the simulation; stopping the compound stops both processes.

The launch flies the saved APM, independently of unsaved map edits. Set these
arguments in `.vscode/launch.json` near your mission's first waypoint:

```json
"--start-lat-deg", "-27.1453118",
"--start-lon-deg", "-52.6025391",
"--start-heading-deg", "345.64"
```

These coordinates match the current `logs/mission.apm`. Update them when moving
the mission to another location. Heading is degrees clockwise from true north
(0 north, 90 east). The simulator defaults to 30° N, 0° E if these arguments are
omitted, and rejects a start outside ±50 km in either local axis from waypoint 1.
The map's **S** marker changes only the geometric preview; its position is not
stored in the APM or automatically passed to the simulator. Initial altitude is
currently fixed at 3000 ft MSL (914.4 m).

Match the launch's start, bank, and L1 options to the preview settings. The
telemetry launch already includes `--flightgear`; use it when FlightGear is
running separately as well. The optional **webapp: prepare telemetry mission** task
still compiles the repository's turns example to `logs/c172x_web_telemetry.apm`;
the telemetry launch uses your downloaded APM instead.

### Terminal

Build the simulator from the repository root:

```bash
cmake --preset host
cmake --build --preset host --target autopilot_sim --parallel 4
```

With the webapp running, open another terminal in `webapp`:

```bash
npm run mission:compile -- ../missions/c172x_turns.json ../logs/webapp_turns.apm
npm run telemetry:sim -- --mode mission --mission logs/webapp_turns.apm --duration 600 --output logs/webapp_flight.csv
```

The bridge starts the simulator from the **repository root**, adds `--realtime`
and `--telemetry-stdout`, and forwards its 10 Hz JSON samples through local UDP.
Simulator paths passed after `--` are relative to the repository root. Set
`AUTOPILOT_SIM` to an absolute executable path to use another build. The map marks
packets as stale after 3 seconds and keeps the most recent 2,000 points.

Add `--flightgear` to the bridge command to send the existing Native FDM stream
on UDP 5600 at the same time. Start FlightGear separately with the repository's
existing visualization launch. The map stream uses UDP 5601.

A duration timeout is a failed/incomplete mission (simulator exit code 2); successful
exit is shown as complete. The final status packet reuses the last received
position. Interrupted/error runs are marked failed when a sample was received.

## CSV replay and synthetic demo

```bash
npm run telemetry:csv -- ../logs/webapp_flight.csv 5
npm run telemetry:demo
```

CSV replay follows recorded timestamps (optional speed multiplier) and forwards
samples at up to 10 Hz of recorded time. CSV alone cannot establish mission success;
its terminal phase says the replay finished with mission outcome unknown. The demo
is explicitly labeled **synthetic geometry**, not a JSBSim flight. Only send one
telemetry source at a time; a new session clears the displayed history.

## Telemetry protocol

Each UDP datagram is a UTF-8 JSON object, at most 4 KB:

```json
{
  "version": 1,
  "session_id": "flight-001",
  "seq": 42,
  "time_s": 4.2,
  "lat_deg": 30.001,
  "lon_deg": 0.0,
  "altitude_m": 914.4,
  "heading_deg": 0.0,
  "airspeed_m_s": 56.2,
  "ground_speed_m_s": 56.0,
  "mission_leg": 0,
  "cross_track_m": 0.3,
  "phase": "tracking leg",
  "status": "running"
}
```

`ground_speed_m_s`, `mission_leg`, `cross_track_m`, and `phase` are optional.
Leg indices are zero-based. Status is `running`, `completed`, or `failed`.
Sequence numbers increase within a session. Duplicates and out-of-order samples
are discarded. Units are degrees, meters, seconds, and m/s; heading is clockwise
from true north. Invalid packets are ignored. New sessions reset the flight trail.

`GET /ws/telemetry` upgrades to WebSocket and sends `{type:"history", samples:[...]}`,
then `{type:"telemetry", sample:{...}}` messages. Samples include server wall-clock
`received_at` (Unix milliseconds), separate from simulation time. Slow clients are
closed and recover the bounded history on reconnect.

## HTTP API

| Endpoint | Request / result |
| --- | --- |
| `POST /api/compile` | Mission JSON → validated `.apm` bytes |
| `POST /api/preview` | `{mission, start, bank_deg, l1_period_s}` → radius, length, selected leg, sampled paths |
| `GET /api/example` | Repository turns mission |
| `GET /api/health` | Server status and UDP port |

`start` has `lat_deg`, `lon_deg`, `course_deg`, and `ground_speed_m_s`.
Errors return JSON `{error:"..."}`. Payloads are limited to 128 KB. The helper
has a 5-second deadline and at most four concurrent requests. HTTP and WebSocket
requests are restricted to this local origin. There is no remote deployment or
aircraft command/upload endpoint in this first version.

## Checks

```bash
npm run build
npm test
```

Tests cover binary compatibility with the C decoder, CRC corruption, validation,
continuous turn geometry, offset entry, skipped legs, infeasible corners, telemetry
ordering/history, HTTP export, and UDP/WebSocket forwarding. The integration test
also checks real simulator telemetry when the host executable has been built.

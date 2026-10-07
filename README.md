# Autopilot C library scaffold

This branch develops a small, portable flight-control library and exercises it against the JSBSim C172X model. It currently controls bank angle, pitch angle, and airspeed. The simulator and plotting tools make each loop observable before adding altitude control, guidance, or a mission system.

The library does **not** yet implement flight modes, mission execution, altitude hold, TECS, firmware, or a hardware failsafe. The simulator's `--mode` argument chooses which setpoint to step during an experiment; it is not an operational AUTO or FBW mode. All three controllers run in every simulator step.

## Layout

- `autopilot/include/autopilot/` and `autopilot/src/`: public C API and controllers.
- `sim/`: C++ adapter between the C library and JSBSim, plus hardcoded C172X tuning.
- `tests/`: controller and simulator checks.
- `tools/plot_attitude.py`: plots simulator CSV output.
- `.vscode/`: build, run, debug, and plot entries.
- `docs/design.md`: boundaries, control laws, architecture, and next steps.
- `third_party/jsbsim/`: pinned JSBSim Git submodule.

## Build and test

Requirements: CMake 3.22 or newer, a C11/C++17 toolchain, and Python 3 with Matplotlib for plots. Initialize the simulator dependency after cloning:

```sh
git submodule update --init --recursive
```

Build and test the portable C library without JSBSim:

```sh
cmake --preset core
cmake --build --preset core
ctest --test-dir build/core-scaffold --output-on-failure
```

Build and test the JSBSim host:

```sh
cmake --preset host
cmake --build --preset host
ctest --test-dir build/host-scaffold --output-on-failure
```

## Run experiments

The simulator trims the C172X near 3000 ft and 100 kt, then steps one target at 2 seconds. Roll uses an absolute bank target in degrees. Pitch and airspeed use offsets from their trimmed values. CSV output goes under the ignored `logs/` directory.

```sh
./build/host-scaffold/sim/attitude_hold_sim --mode roll --bank-deg 5 --output logs/c172x_roll_hold.csv
./build/host-scaffold/sim/attitude_hold_sim --mode pitch --pitch-offset-deg 2 --duration 60 --output logs/c172x_pitch_hold.csv
./build/host-scaffold/sim/attitude_hold_sim --mode airspeed --airspeed-offset-kts 5 --duration 60 --output logs/c172x_airspeed_hold.csv
```

Plot the corresponding CSV with:

```sh
python3 tools/plot_attitude.py --mode roll
python3 tools/plot_attitude.py --mode pitch
python3 tools/plot_attitude.py --mode airspeed
```

Use `--show` to display the plot interactively. VS Code has matching **Run and plot** and **Debug** entries for all three experiments.

The current pitch loop is PD only. The C172X elevator model includes actuator lag and hysteresis, which can leave a small pitch offset even when the controller asks for the correct surface movement. See [design notes](docs/design.md) for the controller equations, units, and planned altitude-control path.

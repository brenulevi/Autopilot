# Autopilot C library scaffold

This branch is a clean starting point for designing the autopilot C library. It
contains the build setup and module folders, but no flight-control behavior or
public API decisions. The previous implementation remains available on `main`.

## Layout

```text
autopilot/include/autopilot/  Public C headers
autopilot/src/mission/       Mission state and target selection
autopilot/src/guidance/      Navigation and altitude target generation
autopilot/src/control/       Attitude and actuator control
autopilot/src/estimation/    Estimated aircraft state interface
docs/                       Design notes
tests/                      Tests added with the behavior they verify
```

The module names are organizing folders, not fixed API boundaries. See
[the design notes](docs/design.md) for the current questions and concepts.

## Build

```sh
cmake --preset core
cmake --build --preset core
```

The build currently produces an empty static C library. Add tests when the
first behavior and its expected result are defined.

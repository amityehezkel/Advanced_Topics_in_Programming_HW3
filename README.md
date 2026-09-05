# Assignment 3 — Concurrent Drone Mapping Simulator

Submission by Amit Yehezkel, ID 212200943.

This repository contains the three independently buildable Assignment 3
components:

- `Algorithm_212200943.so`: the mapping strategy, in namespace
  `algorithm_212200943`.
- `MissionControl_212200943.so`: the single-mission coordinator and drone
  controller, in namespace `mission_control_212200943`.
- `simulator_212200943`: the comparative/competitive runner, in namespace
  `simulator`.

The staff-provided `common/` and component-specific common headers are used
without modification. `UserCommon/` is intentionally empty because no
student-owned type is genuinely shared by multiple projects.

## Dependencies

The project uses C++20 and the libraries declared in `vcpkg.json`:

- mp-units
- TinyNPY
- yaml-cpp
- GoogleTest (tests only)

On Linux, dynamic loading uses `dlopen`/`dlclose` and the standard threading
library.

## Build

From the repository root, with `VCPKG_ROOT` set:

```sh
cmake --preset default
cmake --build --preset default -j
```

The artifacts are created at:

```text
build/default/Algorithm/Algorithm_212200943.so
build/default/MissionControl/MissionControl_212200943.so
build/default/Simulator/simulator_212200943
```

Each project can also be configured and built independently. For example:

```sh
cmake -S Algorithm -B build-algorithm \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build-algorithm -j

cmake -S MissionControl -B build-mission-control \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build-mission-control -j

cmake -S Simulator -B build-simulator \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build-simulator -j
```

The component CMake files explicitly reuse the root vcpkg manifest.

## Run

Arguments may appear in any order. `num_threads` and `-verbose` are optional;
all other arguments shown for the selected mode are required.

Comparative mode runs one Algorithm against every MissionControl `.so` in a
folder:

```sh
./build/default/Simulator/simulator_212200943 \
  -comparative \
  simulation=inputs/sim_compose.yaml \
  mission_control_folder=build/default/MissionControl \
  algorithm=build/default/Algorithm/Algorithm_212200943.so \
  num_threads=4 \
  -verbose
```

Competition mode runs one MissionControl against every Algorithm `.so` in a
folder:

```sh
./build/default/Simulator/simulator_212200943 \
  -competition \
  simulation=inputs/sim_compose.yaml \
  mission_control=build/default/MissionControl/MissionControl_212200943.so \
  algorithms_folder=build/default/Algorithm \
  num_threads=4
```

Invalid syntax, unsupported arguments, missing arguments, bad paths, and plugin
folders containing no `.so` files print an error and both usage forms, then
return a nonzero status. Unsupported and missing arguments are reported
together rather than one at a time.

## Threading model

- Missing `num_threads`, or `num_threads=1`, executes all runs on the main
  thread.
- For `num_threads=N`, where `N >= 2`, the Simulator creates
  `min(N, number_of_jobs)` worker threads. The main thread waits for them.
- The configuration Cartesian product, result table, output directories, and
  all run object graphs are created before worker execution.
- Workers claim immutable job indices atomically and write only to their own
  preallocated result slot and unique files.
- Algorithm and MissionControl instances are never shared or cached.

This avoids result-table locks and keeps report ordering deterministic across
thread counts.

## Architecture

### Algorithm

`MappingAlgorithmImpl` adapts the Assignment 2 mapping strategy. It scans six
directions, searches mapped free space with BFS, respects per-command movement
limits, avoids unknown/occupied passages, and reports whether unmappable voxels
remain. The `.cpp` registers the class automatically with
`REGISTER_MAPPING_ALGORITHM`.

### MissionControl

`MissionControlImpl` owns its `DroneControlImpl`, runs one bounded mission,
saves the output map on every terminal path, and returns typed errors instead
of throwing ordinary runtime failures. `DroneControlImpl` validates Algorithm
commands, delegates to the injected hardware interfaces, and converts LiDAR
observations into map evidence. The `.cpp` registers the class with
`REGISTER_MISSION_CONTROL`.

When `-verbose` is present, every MissionControl instance creates one unique
trace beside its map. Without the flag, no verbose file is created.

### Simulator

The Simulator owns all simulation-only implementations: map storage, mock GPS,
LiDAR, movement/collision physics, map scoring, configuration loading, dynamic
plugin loading, run composition, thread scheduling, and reporting.

Plugin handles use move-only RAII. Factory and plugin-created objects are
destroyed before their shared-library handle, after which `dlclose` is called.
The executable exports the staff registration-constructor symbols needed by
plugin static initialization. Each `.so` is loaded at most once per invocation.

No explicit `new` or `delete` is used. `unique_ptr` owns run objects and plugin
instances; `shared_ptr` is limited to TinyNPY arrays because the TinyNPY-backed
map adapter shares that storage.

## Outputs

Comparative runs create
`<mission_control_folder>/comparative_results_<time>/`; competition runs create
`<algorithms_folder>/competition_<time>/`. Existing result directories are
never overwritten; a numeric suffix is added if a timestamp collides.

Each result directory contains:

- `comparative_results.yaml` or `competition_results.yaml`, using the exact
  `comparative_report` or `competitive_report` schema from the assignment;
- one `simulation_results_<plugin>.yaml` detailed Assignment 2-style report per
  successfully executed plugin;
- `errors.log`, which is empty when no errors occurred and is appended to
  immediately when recoveries or failures occur;
- unique per-job directories containing a map whose filename includes both the
  plugin and stable job identity;
- a `<unique-map-stem>_verbose.log` trace beside each map only when `-verbose`
  was requested.

Comparative results group MissionControls only when both total score and total
executed steps are exactly equal, then sort groups by number of agreeing
MissionControls descending. Competition results sort by total score descending,
then total steps ascending, then filename for deterministic ties. Rejected or
failed individual configurations contribute score `-1`; step totals count only
steps actually executed.

## Error handling

Malformed nested configuration files reject only their affected Cartesian
product runs and remain visible in detailed reports with score `-1`. Recoverable
Assignment 2 omissions use documented defaults and are logged immediately.

A plugin that cannot be opened or does not register exactly one factory of the
expected type is listed in the aggregate `errors` field; other plugins continue.
Standard and unknown exceptions at run and application boundaries are converted
to diagnostics. As allowed by the assignment, an actual process-level crash
inside third-party plugin code cannot be recovered by the Simulator.

## Tests and verification

Build and run the automated contract tests with:

```sh
cmake --build --preset default -j
ctest --test-dir build/default --output-on-failure
```

The tests cover complete command-line diagnostics, exact comparative grouping,
competition ranking, and YAML schema keys parsed back through yaml-cpp. The
small runtime corpus in `Simulator/tests/data/` is intended for quick manual
sequential/parallel checks.

Validation performed in the supplied Linux development image included:

- root and three clean standalone builds with strict warnings as errors;
- all 24 supplied configurations in both modes;
- identical sequential and parallel output-map SHA-256 hashes;
- argument reordering and worker counts larger than the number of jobs;
- verbose enabled/disabled file creation;
- malformed plugin isolation and registration-symbol export checks.

No known functional issues remain from these tests.

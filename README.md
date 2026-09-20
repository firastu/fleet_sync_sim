# FleetSyncSim

FleetSyncSim is a deterministic C++20 simulation platform for exploring
**sovereign positioning, mapping, navigation, and cooperative autonomy for
robots**.

The project studies robots that can:

* navigate from locally controlled map data;
* maintain independent world models;
* exchange useful information peer-to-peer;
* continue operating through communication or positioning loss;
* make degraded state observable instead of silently depending on infrastructure.

> **Status: M2 complete; M3 resilient localization is in progress.**
>
> The deterministic distributed-autonomy platform, movement/sensing layer,
> OSM/GeoJSON integration, and interactive scenario tooling are established.
> M3 currently studies positioning degradation: the localization boundary and
> deterministic noisy GNSS are implemented; outage and stale-estimate semantics
> are next.

FleetSyncSim began with **distributed robot-local map knowledge under unreliable
communication**. It now provides a broader deterministic autonomy test platform
with movement, truth-versus-belief sensing, real road networks, local planning,
distributed reconciliation, geospatial observability, and an emerging
localization/PNT layer.

---

## Core idea

Central infrastructure may improve capability, but it should not automatically
become a hard runtime dependency.

Examples:

```text
control station unavailable
        |
        v
robots continue locally
        |
        v
peer knowledge diverges
        |
        v
connectivity returns
        |
        v
knowledge reconciles
```

and:

```text
GNSS available
      |
      v
localization estimate

GNSS unavailable
      |
      v
last estimate becomes stale
      |
      v
later: dead reckoning / local sensing / map matching
      |
      v
controlled degradation
```

The deterministic simulator provides a behavioral reference for studying these
conditions reproducibly.

---

## Currently implemented

* deterministic C++20 discrete-event simulation;
* immutable base maps and robot-local dynamic overlays;
* deterministic A* planning and autonomous rerouting;
* robot movement and mission completion;
* unreliable P2P communication with latency, loss, duplication, reordering and
  partitions;
* sequenced distributed map reconciliation;
* ControlStation aggregation and reconnect synchronization;
* world truth separated from robot-local sensing and belief;
* deterministic JSON scenarios and structured JSONL traces;
* interactive scenario stepping, state inspection and event injection;
* WGS84 map geometry and one-way topology;
* deterministic OSM PBF import;
* deterministic GeoJSON debugging/export;
* localization truth/estimate boundary;
* perfect, unavailable and deterministic noisy GNSS models.

For exact active implementation scope, see:

* [`docs/CURRENT_MILESTONE.md`](docs/CURRENT_MILESTONE.md)

For a compact architecture map:

* [`docs/AI_CONTEXT.md`](docs/AI_CONTEXT.md)

For accepted architectural contracts:

* [`docs/design_decisions/`](docs/design_decisions/)

---

## Documentation

* [Current milestone](docs/CURRENT_MILESTONE.md)
* [Compact architecture context](docs/AI_CONTEXT.md)
* [Project vision](docs/PROJECT_VISION.md)
* [Documentation guide](docs/README.md)
* [Design decisions](docs/design_decisions/)
* [Geospatial tooling boundary](docs/geospatial.md)
* [Research](docs/research/)

`PROJECT_VISION.md` describes long-term direction.

`CURRENT_MILESTONE.md` defines what may be implemented now.

ADRs define durable architectural contracts.

---

## Building

Requirements:

* Linux;
* CMake >= 3.21;
* C++20 compiler;
* project development environment currently uses GCC 13.

The first configure may require network access to fetch pinned development
dependencies.

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Sanitized builds:

```sh
cmake --preset asan
cmake --build --preset asan
ctest --preset asan
```

TSan uses the repository's established per-process ASLR workaround in the
current GCC 13 environment:

```sh
setarch $(uname -m) -R cmake --build --preset tsan
setarch $(uname -m) -R ctest --preset tsan
```

Do not disable ASLR system-wide.

---

## Running a scenario

Built-in founding scenario:

```sh
./build/debug/apps/fleet_sim/fleet_sim
```

Run a declarative scenario with an explicit seed and JSONL trace:

```sh
./build/debug/apps/fleet_sim/fleet_sim \
    --scenario scenarios/station_partition.json \
    --seed 1234 \
    --trace run.jsonl
```

Seed precedence is:

```text
CLI --seed
    |
    v
scenario seed
    |
    v
default seed 0
```

Where the relevant ADR defines byte-stable behavior, the same scenario and
resolved seed reproduce the same structured trace.

---

## Interactive console

The console operates through `ScenarioRunner` public APIs.

It does not introduce separate simulation semantics.

```sh
./build/debug/apps/fleet_console/fleet_console \
    --scenario scenarios/world_sensing.json
```

Example session:

```text
> run 1500
> robots
> world C-D blocked
> robot robot_a
> finish
```

Loaded scenario events and injected events use the same effect path.

Stepped execution is tested against one-shot execution for deterministic
equivalence.

---

## Importing real maps

FleetSyncSim can deterministically import supported OSM PBF road data into:

```text
BaseMap
├── Graph
└── MapGeometry
```

Example:

```sh
./build/debug/apps/fleet_map_import/fleet_map_import map.osm.pbf \
    --map-geojson map.geojson
```

The importer preserves relevant road geometry, respects supported one-way
semantics, computes metric traversal costs, and fails explicitly on unsupported
topology rather than silently altering it.

GeoJSON output is outward-facing debug/inspection data and does not participate
in planning state.

---

## Deterministic reference behavior

Determinism is a core engineering property of the project.

Simulation code avoids:

* wall-clock time;
* behavior derived from pointer values;
* observable unordered-container iteration;
* unspecified standard-library probability distributions.

Randomness flows through the project's deterministic RNG abstraction.

The deterministic simulator remains the reference behavior even if
higher-fidelity physical simulation is added later.

---

## Long-term direction

FleetSyncSim's north star is:

> **Sovereign positioning, mapping, navigation, and cooperative autonomy for
> robots.**

Long-term experiments may include:

* GNSS degradation and outages;
* dead reckoning;
* IMU and wheel odometry;
* visual or LiDAR odometry;
* map matching;
* uncertainty-aware autonomy;
* cooperative localization;
* physical/sensor simulation through external environments such as NVIDIA
  Isaac Sim;
* eventually real robot adapters.

These are promoted incrementally through milestones and ADRs.

Their presence in the vision does not make them current implementation scope.

See [`docs/PROJECT_VISION.md`](docs/PROJECT_VISION.md).

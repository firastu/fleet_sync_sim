# FleetSyncSim — AI Context Snapshot

Purpose: compact repository orientation for coding agents.

This file is **not normative**. It summarizes current architecture so an agent
does not need to read every ADR at the beginning of every session.

Authority remains:

```text
AGENTS.md
CURRENT_MILESTONE.md
relevant ADRs
code + tests
```

If this file disagrees with an authoritative source, the authoritative source
wins.

---

## Current status

```text
M1  deterministic distributed reference simulator     COMPLETE
M2  movement, sensing, geospatial grounding           COMPLETE
M3  resilient positioning / localization              IN PROGRESS
```

M3 completed:

```text
#14 localization boundary       ADR-016
#15 deterministic noisy GNSS    ADR-017
#16 outage + stale estimates    ADR-018
#17 dead reckoning / drift      ADR-019  (owner-reviewed 2026-09-21)
```

Current next step:

```text
#18 GNSS reacquisition observability: implemented, awaiting review (ADR-021)
```

After the #18 review: review the accumulated measurement/failure semantics,
then decide whether estimator architecture is justified (its own ADR).

Note: Isaac Sim stage 1 (read-only replay, ADR-020) lives on the
`feat/isaac-stage1-replay` branch, off master; master does not carry it.

---

## North star

FleetSyncSim explores:

> Sovereign positioning, mapping, navigation, and cooperative autonomy for
> robots.

"Sovereign" means essential local autonomy should degrade predictably when
external infrastructure disappears rather than immediately becoming unusable.

---

## Core architectural model

```text
                        Mission
                           |
                        Robot
                           |
                    Route / Planner
                           |
                        MapView
                     /           \
               BaseMap          Overlay
               /     \             |
           Graph   MapGeometry   MapDelta
                             reconciliation

World truth
    |
    | sensing / measurement only
    v
robot-local belief


GroundTruthPose
    |
    | GnssModel / future sensor models
    v
LocalizationEstimate


Robot A <---- unreliable network ----> Robot B
   \                                  /
    \---------- ControlStation -------/
```

Participants do not share mutable dynamic map state.

---

## Important invariants

### Determinism

Reference behavior is deterministic for:

```text
(scenario, resolved seed)
```

No wall clock.

No unordered-container iteration may affect observable behavior.

Randomness flows through `DeterministicRng`.

Same scenario + seed is expected to reproduce the same structured trace where
the relevant ADR defines that contract.

### Map model

`BaseMap` is immutable.

```text
BaseMap
├── Graph
└── optional MapGeometry
```

`Graph` contains planning topology.

`MapGeometry` contains WGS84 geometry and is not consulted by the planner.

Dynamic participant knowledge lives in `DynamicMapOverlay`.

### Direction

`EdgeDirection` is topology.

Dynamic OPEN/BLOCKED state is knowledge.

`MapView::traversable_from(...)` is the topology-direction decision boundary.

### Distributed knowledge

Map observations become sequenced `MapDelta` values.

Reconciliation tracks source progression independently from the current
effective overlay winner.

Duplicate, stale, conflict, dominated, and applied outcomes are distinct.

### Robot boundary

Robot autonomy does not depend on:

* NetworkSimulator;
* EventQueue;
* ScenarioRunner;
* visualization;
* GIS tools;
* external simulators.

### Truth versus belief

`fleet::world` contains simulation truth.

Robots cannot read world truth directly.

Truth reaches robot-local state through an explicit sensing model.

### Localization

Current pipeline:

```text
GroundTruthPose
      |
      v
   GnssModel
      |
      v
optional<LocalizationEstimate>
```

Implemented models:

```text
PerfectGnss
UnavailableGnss
NoisyGnss
```

`LocalizationEstimate::estimated_at` is the timestamp of the state represented
by the estimate.

No estimator/filter has been selected yet.

No EKF, SLAM, map matching, or cooperative localization is currently an
architectural dependency.

---

## Noisy GNSS contract

Position noise is generated in local East/North meters.

Configuration uses:

```text
position_axis_sigma_m
heading_sigma_rad
```

Position sigma is per axis, not radial RMS.

Sampling uses deterministic midpoint-centered Irwin-Hall(12) draws from the
raw deterministic RNG path.

Zero-noise models consume no random draws.

Metric displacement is converted to WGS84 at one explicit boundary.

Longitude wraps across the antimeridian.

Unsupported pole-domain behavior fails explicitly rather than clamping.

The noise model is an engineering degradation model, not a high-fidelity GNSS
receiver simulation.

---

## Outage and stale localization (#16, ADR-018)

Localization is scenario opt-in (`"localization"` block + `set_gnss_model`
action; one model grammar: perfect / unavailable / noisy).

Per-robot GNSS sampling starts at tick 0 and repeats every `period_ms`,
strictly later each time. A model switch never samples immediately.
Equal ticks retain enqueue order. A loaded scripted model switch at tick T
(including 0) applies before the T sample; movement has no universal
priority over scripts or samples (ADR-018).

Each robot owns its OWN GNSS RNG stream, derived from the resolved seed
by specified arithmetic (`derive_stream_seed`; stable RobotId, fixed
GNSS domain) — adding, removing or rescheduling other robots never
shifts a robot's noise sequence.

The retained estimate is robot-local (`LocalizationTracker` on `Robot`):
a fix replaces it, a no-fix retains it, and age is derived
(`now - estimated_at`), never stored.

Truth pose is derived simulation-side by `world::truth_pose` from
movement timing + MapGeometry arc length (direction-aware, exact
canonical endpoints, explicit failure when geometry is missing).
Stationary orientation is real truth, not a placeholder: an explicit
initial condition (north) retained kinematically across arrivals as the
completed traversal's final-segment bearing.

Trace: `gnss_sample` / `gnss_model` events carry belief-side fields
only. Console: `robot <name>` shows the localization block.

Zero-consumption RNG contracts hold; same scenario + seed is
byte-identical.

---

## Dead reckoning (#17, proposed ADR-019)

`localization.dead_reckoning` opts into fixed distance-scale and per-meter
heading bias. Robot-owned traversal timing and local map geometry produce
relative increments; `LocalizationTracker::propagate` integrates from belief,
never from world truth. Segment geometry is cached per departure. No filter or
new random stream is introduced; propagation consumes no RNG draws.

Without the opt-in block, #16's frozen estimate remains unchanged. With it,
`estimated_at` advances while `last_fix_at` stays pinned during an outage.
Trace and console show source/last-fix age; separate simulation-side diagnostics
show position error without feeding truth to autonomy. Review #17 before #18.

## Geospatial boundary

Real map flow:

```text
OSM PBF
   |
fleet::osm importer
   |
BaseMap(Graph + MapGeometry)
   |
planner / simulator
```

Output flow:

```text
BaseMap / Trace
      |
fleet::geojson
      |
GeoJSON
      |
GeoLibre / QGIS
```

GIS applications are external tools.

The core has no GIS runtime dependency.

GeoJSON coordinates are `[longitude, latitude]`.

Internal `Wgs84Coordinate` uses named latitude/longitude fields.

---

## External physical simulation

NVIDIA Isaac Sim is part of the long-term validation strategy, not the current
runtime core.

Intended relationship:

```text
FleetSyncSim
    autonomy + distributed-system semantics

Isaac Sim
    physics + robot motion + simulated sensors
```

Planned direction after the current M3 localization semantics are established:

```text
FleetSyncSim adapters
        |
        +---- Isaac ground-truth pose
        +---- IMU / odometry
        +---- later perception sensors
```

Isaac must remain behind explicit adapters.

Do not introduce Isaac, ROS 2, or simulator-specific types into FleetSyncSim
domain modules until a future milestone promotes that work.

The deterministic simulator remains the behavioral reference even after a
higher-fidelity physical backend exists.

---

## Module orientation

Use source code as the final reference, but conceptually:

```text
fleet::common
    strong/common value types

fleet::map
    Graph, BaseMap, MapGeometry, overlays, reconciliation

fleet::planning
    deterministic A* routing

fleet::simulation
    logical clock, event queue, deterministic RNG

fleet::network
    deterministic unreliable transport

fleet::robot
    robot-local autonomy

fleet::world
    simulation ground truth, observation boundary, truth-pose derivation

fleet::scenario
    scenario loading, scheduling and orchestration

fleet::osm
    deterministic OSM import

fleet::geojson
    outward-only debug/export adapters

fleet::localization
    pose, GNSS models, estimate retention, reacquisition observability,
    localization boundary
```

Apps are adapters around these modules rather than owners of domain logic.

---

## ADR lookup

ADRs under `docs/design_decisions/` are normative accepted contracts.

Do not read every ADR by default.

Discover the available decisions from the repository:

```sh
find docs/design_decisions -maxdepth 1 -name 'ADR-*.md' -printf '%f\n' | sort
```

When looking for a decision by topic, search filenames/content first:
```sh
rg -n "localization|GNSS|movement|network|map|scenario" \
    docs/design_decisions/ADR-*.md
```
Then read only the ADRs relevant to the current subsystem.

docs/design_decisions/README.md explains how ADRs should be consumed.

## Standard validation

Typical feature validation includes:

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug

cmake --preset asan
cmake --build --preset asan
ctest --preset asan

# TSan uses the repository's documented setarch -R workaround.
```

Also verify:

```sh
git diff --check
```

and preserve founding-trace equality where the change is not intended to alter
that scenario.

---

## What not to infer

Unless promoted into the active milestone, do not assume the project currently
has or needs:

* threads;
* ROS 2;
* Isaac integration;
* SLAM;
* an EKF;
* visual odometry;
* LiDAR odometry;
* cooperative localization;
* high-fidelity RF simulation;
* distributed consensus;
* cloud services.

Future vision is not current scope.

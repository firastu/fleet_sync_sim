# FleetSyncSim Project Vision

## North star

FleetSyncSim explores software architectures for:

> **Sovereign positioning, mapping, navigation, and cooperative autonomy for
> robots.**

"Sovereign" does not mean avoiding external systems.

It means essential autonomous capabilities should not contain unnecessary hard
runtime dependencies on infrastructure outside the robot or fleet.

External positioning, maps, control stations, communication infrastructure,
physical simulators and development tools may improve capability.

Their temporary loss should produce controlled degradation rather than
immediate system collapse.

---

## Capability pillars

### 1. Locally controlled maps and navigation

Robots should be able to operate from locally available map data and locally
executed planning algorithms.

Conceptually:

```text
source geographic data
        |
        v
   map pipeline
        |
        v
  local map model
        |
        v
 routing / planning
        |
        v
      robot
```

Runtime navigation should not inherently depend on a cloud map or remote
routing provider.

---

### 2. Resilient positioning and navigation

GNSS is one positioning source, not an assumed permanent dependency.

Long-term experiments may combine:

```text
GNSS
IMU
wheel odometry
visual odometry
LiDAR odometry
environmental / map matching
        |
        v
 state estimation
        |
        v
pose + velocity + uncertainty
```

When an absolute positioning source disappears, the system should model
degrading knowledge rather than instantaneous loss of all positional state.

---

### 3. Robot-local world models

Every robot maintains its own knowledge.

The architecture distinguishes:

```text
simulation ground truth
```

from:

```text
robot-local belief
```

Robot-local state may eventually contain:

* static map information;
* dynamic map changes;
* observations;
* provenance;
* information age;
* pose estimates;
* uncertainty.

Temporary disagreement between robots is expected in a distributed system.

Shared truth is not assumed.

---

### 4. Peer-to-peer cooperation

Robots should eventually exchange useful state directly.

Possible information includes:

* map deltas;
* obstacle observations;
* pose estimates and uncertainty;
* landmarks;
* route or mission intent;
* health and status.

Local cooperation must not fundamentally require:

```text
Robot -> Cloud -> Robot
```

A control station may assist, coordinate, persist or aggregate information, but
it should not be a mandatory intermediary for useful robot-to-robot
cooperation.

---

### 5. Graceful degradation

The system should explicitly study degraded modes.

Positioning example:

```text
GNSS unavailable
      |
      v
dead reckoning / local sensing / map matching
      |
      v
uncertainty or information age grows
      |
      v
autonomy adapts
```

Communication example:

```text
V2V unavailable
      |
      v
robots continue independently
      |
      v
knowledge diverges
      |
      v
connectivity returns
      |
      v
knowledge reconciles
```

Loss of an external capability should be visible in system state.

---

## Long-term conceptual stack

```text
                    Mission / Autonomy
                           |
                    Route + Planning
                           |
              +------------+------------+
              |                         |
          Map System              Localization / PNT
              |                         |
        offline maps                   GNSS
        dynamic state                  IMU
        map matching                   odometry
        map merging                    perception
              |                         |
              +------------+------------+
                           |
                   Local World Model
                           |
                    V2V Cooperation
                   /       |       \
            map deltas    pose    observations
                           |
                  Communication Layer
                           |
                         Robots
```

This is directional architecture, not a claim that every layer is currently
implemented.

---

## Simulation and validation environments

FleetSyncSim deliberately separates autonomy semantics from physical
simulation.

The deterministic FleetSyncSim simulator remains the behavioral reference for:

* distributed knowledge;
* communication faults;
* degraded autonomy;
* localization semantics;
* scenario reproducibility.

Higher-fidelity environments may provide physical motion and simulated sensor
data through explicit adapters.

Conceptually:

```text
                  FleetSyncSim autonomy core
                           |
                     adapter boundary
                    /                 \
                   /                   \
                  v                     v
     deterministic simulation     physical simulation
                                     |
                                NVIDIA Isaac Sim
                                     |
                           physics + robot motion
                           IMU / odometry / LiDAR
                           cameras / other sensors
```

The intended responsibility split is:

```text
FleetSyncSim
    autonomy and distributed-system semantics

Isaac Sim
    physical world, robot motion and simulated sensors
```

Isaac Sim is therefore a development, validation and experimentation backend.

It is **not** part of the sovereign operational dependency chain.

Simulator-specific types must not leak into the FleetSyncSim domain model.

The long-term target is that the same autonomy boundaries can eventually be
exercised against:

```text
deterministic FleetSyncSim simulation
high-fidelity physical simulation
real robot hardware
```

without rewriting the autonomy core around a particular simulator.

The deterministic simulator remains valuable even after higher-fidelity
simulation exists because it provides fast, controlled and reproducible
behavioral experiments.

---

## External systems are adapters, not owners

The project may eventually integrate technologies such as:

* NVIDIA Isaac Sim;
* ROS 2;
* physical robot drivers;
* external GIS workstations;
* offline map preparation tools.

These systems should connect through explicit boundaries.

They should not redefine core domain concepts merely because their APIs use a
different representation.

Conceptually:

```text
                        FleetSyncSim Core
                         /            \
                        /              \
                       v                v
                 Isaac Sim         Real robot
                    |
                  ROS 2
                 adapters
```

The exact integration architecture is decided only when promoted into an active
milestone and, where durable, recorded in an ADR.

---

## What FleetSyncSim is today

FleetSyncSim began with deterministic distributed robot-local map knowledge
under unreliable communication.

That foundation now includes:

* movement and autonomous replanning;
* world truth separated from robot-local perception;
* real geographic road networks;
* deterministic scenario execution;
* peer knowledge reconciliation;
* geospatial observability;
* an emerging resilient-localization layer.

This directional document intentionally does not track every current feature or
commit.

For current implementation scope, read:

```text
CURRENT_MILESTONE.md
```

For compact architecture orientation, read:

```text
AI_CONTEXT.md
```

For accepted architecture contracts, read the relevant:

```text
design_decisions/ADR-*.md
```

---

## Guiding principle

> A capability should continue locally where reasonably possible when an
> external service disappears, and the system should make degradation
> observable rather than hiding it.

A second principle follows from this:

> External simulation, tooling and infrastructure may strengthen experiments,
> but the autonomous domain model should remain understandable and testable
> without making those systems hidden runtime dependencies.

===== FILE: docs/geospatial.md =====

# Geospatial Development Tooling

|                   |                                                                                                         |
| ----------------- | ------------------------------------------------------------------------------------------------------- |
| **Document type** | Tooling boundary and usage guide                                                                        |
| **Status**        | Accepted boundary — map import and GeoJSON inspection active                                            |
| **Authority**     | Boundary rules below are normative; individual tool choices are directional                             |
| **Scope**         | Geospatial map preparation, inspection, visualization and future scenario authoring around FleetSyncSim |

---

## 1. Purpose

FleetSyncSim studies sovereign cooperative autonomy using locally held maps,
independent world models and local planning.

Real maps are geographic.

This document defines how external geospatial tooling participates without
becoming part of the autonomous core.

The central rule is:

> **Geospatial tools are human-facing workstations. FleetSyncSim is the
> machine-facing autonomy system.**

They communicate through explicit data formats.

Neither depends on the other's internal representation.

---

## 2. Boundary

```text
                    Geospatial workstation
                     GeoLibre / QGIS
                           |
             inspect / visualize / prepare
                           |
                    open data formats
                           |
              GeoJSON / OSM PBF / ...
                           |
                           v
       +-------------------------------------------+
       |            FleetSyncSim core              |
       |                                           |
       | BaseMap -> MapView -> AStarPlanner        |
       |    |                                      |
       |    +-> MapGeometry                        |
       |                                           |
       | Robot-local belief <- MapDelta <- V2V     |
       | ScenarioRunner / World / Localization     |
       +-------------------------------------------+
```

---

## 3. Normative boundary rules

### 3.1 No GIS runtime dependency

Nothing in the FleetSyncSim domain core may require a desktop GIS application
at runtime.

No core type should depend on GeoLibre, QGIS or another workstation.

### 3.2 No representation capture

Core map abstractions must not be reshaped merely to match a GIS application's
internal model.

For example:

```text
BaseMap
Graph
MapGeometry
DynamicMapOverlay
MapView
```

exist for FleetSyncSim domain semantics.

External formats are translations at a boundary.

### 3.3 Formats are the contract

Data exchange uses explicit portable formats.

Current important formats are:

```text
OSM PBF
    map input

GeoJSON
    debug / visualization output
```

Additional formats may be introduced when a concrete use case requires them.

### 3.4 Offline operation matters

Geospatial workflows should remain compatible with local, offline or
air-gapped use.

The project should not require uploading map data or robot traces to a cloud
service merely to inspect them.

---

## 4. Current geospatial flow

### Map input

```text
region.osm.pbf
      |
      v
 fleet::osm
      |
      v
BaseMap
├── Graph
└── MapGeometry
      |
      v
planning / simulation
```

The OSM importer is deterministic and converts external geographic data into
FleetSyncSim domain types.

Parser dependencies remain isolated to the importer target.

---

### Debug map output

```text
BaseMap
   |
   v
fleet::geojson
   |
   v
GeoJSON
   |
   v
GeoLibre / QGIS
```

This allows visual inspection of imported topology and geometry without making
the GIS tool part of simulation behavior.

Example:

```sh
./build/debug/apps/fleet_map_import/fleet_map_import map.osm.pbf \
    --map-geojson map.geojson
```

---

### Trace output

```text
deterministic trace
       |
       v
 fleet::geojson
       |
       v
    GeoJSON
       |
       v
GeoLibre / QGIS
```

Current trace-derived geographic output is intentionally outward-only.

Exporters observe completed simulator state or traces.

They do not influence robot behavior.

---

## 5. Development stages

Stages describe useful geospatial capabilities.

They do not authorize implementation by themselves.

### G1 — map inspection — ACTIVE

FleetSyncSim can import supported OSM PBF data and emit deterministic map
GeoJSON.

This supports questions such as:

* did the importer retain the expected roads?
* are intersections connected correctly?
* is one-way geometry oriented as expected?
* is the network geographically where expected?

A rejected-feature diagnostic layer may be added later if importer debugging
justifies it.

It is not required merely because it appears here.

---

### G2 — trace visualization — ACTIVE, incremental

FleetSyncSim can export deterministic geographic information from simulation
traces.

Current functionality establishes the outward adapter boundary.

Possible future layers include:

* routes per replan;
* observations;
* communication topology;
* robot-local map beliefs;
* world truth;
* localization estimates;
* truth-versus-estimate trajectories.

These are directions, not current implementation requirements.

In particular, visualizing ground truth must never provide oracle knowledge
back to robot autonomy.

---

### G3 — scenario authoring — LATER

A GIS workstation could eventually help author:

* missions;
* blocked regions;
* communication blackout regions;
* landmarks;
* geographic experiment areas.

Such tooling should export into FleetSyncSim's scenario boundary.

The scenario format must remain understandable independently of a GIS plugin.

---

### G4 — field collection — LATER

Field collection may eventually support:

* map validation;
* landmark collection;
* physical experiment reproduction;
* real-world comparison against simulation.

This remains outside current implementation scope.

---

## 6. Preferred exchange formats

### GeoJSON

Default debug and visualization format.

GeoJSON positions use:

```text
[longitude, latitude]
```

FleetSyncSim's internal WGS84 type uses named latitude and longitude fields.

The conversion occurs explicitly at the output boundary.

Serialized coordinate precision is an outward representation choice and does
not replace internal map-domain precision.

### OSM PBF

Primary source format for offline road-network imports.

### Possible later formats

If dataset scale or workflows require them:

* GeoPackage;
* FlatGeobuf;
* PMTiles;
* MBTiles.

Their presence here does not authorize implementation.

---

## 7. GeoLibre and QGIS

GeoLibre and QGIS are external workstation choices.

They are not dependencies of FleetSyncSim.

Either may be used to inspect standard output formats.

The project should prefer portable data contracts over tool-specific
integration so changing workstation software does not affect the autonomy core.

A dedicated plugin may eventually be useful for scenario authoring or
interactive analysis, but it is not required by the architecture.

---

## 8. What this document does not authorize

This document does **not** authorize:

* GIS libraries inside unrelated FleetSyncSim domain modules;
* cloud map-provider dependencies;
* live visualization coupled into simulation semantics;
* a GIS-controlled robot state;
* automatic implementation of every proposed visualization layer;
* scenario-authoring plugins;
* field-collection features.

Promotion into implementation scope follows `CURRENT_MILESTONE.md`.

---

## 9. Relationship to architectural decisions

The authoritative details live in the relevant ADR files under:

```text
docs/design_decisions/
```

Important related areas include:

* immutable base-map ownership;
* deterministic behavior;
* robot autonomy boundaries;
* world truth versus belief;
* map geometry;
* one-way topology;
* OSM import;
* GeoJSON output;
* localization.

Do not duplicate those ADR contracts here.

When exact semantics matter, open the relevant ADR.

===== FILE: docs/design_decisions/README.md =====

# Architecture Decision Records

Files under this directory record **accepted durable architectural contracts**
for FleetSyncSim.

ADRs are normative.

They answer questions such as:

* why a boundary exists;
* which alternatives were rejected;
* what future code may rely on;
* what invariants must remain true.

They are not a chronological project diary.

---

## Reading strategy

Do not read every ADR before ordinary coding work.

Start with:

```text
AGENTS.md
docs/CURRENT_MILESTONE.md
docs/AI_CONTEXT.md
```

Then discover the ADRs that exist:

```sh
find docs/design_decisions -maxdepth 1 -name 'ADR-*.md' -printf '%f\n' | sort
```

Search for the subsystem or concept involved in the task:

```sh
rg -n "localization|GNSS|movement|network|map|scenario|robot" \
    docs/design_decisions/ADR-*.md
```

Read only the decisions relevant to the change.

This keeps agent context small while preserving access to precise contracts.

---

## Authority

An accepted ADR is normative for the architectural contract it defines.

If an ADR appears inconsistent with:

* `AGENTS.md`;
* `CURRENT_MILESTONE.md`;
* another accepted ADR;
* implementation/tests;

do not silently choose one interpretation.

Determine whether:

1. documentation became stale;
2. implementation violates the accepted contract;
3. a later ADR intentionally superseded an earlier decision.

Resolve the inconsistency explicitly before building additional behavior on it.

---

## Numbering

ADRs use sequential filenames:

```text
ADR-NNN-short-description.md
```

Examples:

```text
ADR-001-...
ADR-002-...
ADR-003-...
```

Rules:

* never reserve numbers for future decisions;
* use the next number only when the ADR is actually introduced;
* do not renumber accepted historical ADRs merely for aesthetics.

---

## When to create an ADR

Create an ADR when a decision is durable enough that future work will rely on
it.

Typical examples:

* ownership/lifetime rules;
* determinism contracts;
* subsystem boundaries;
* distributed-state semantics;
* identity or ordering rules;
* timing semantics;
* public representation contracts;
* adapter boundaries that constrain future integrations.

Do **not** create an ADR merely because:

* a commit is large;
* a test was added;
* implementation details changed internally;
* documentation became longer.

---

## ADR contents

A useful ADR should normally make these clear:

```text
Context
Decision
Consequences
Rejected alternatives
Invariants / constraints
```

Use the existing ADR style in this repository rather than forcing every old
record into a new template.

Precise contracts matter more than identical formatting.

---

## Superseding a decision

Do not rewrite historical architectural intent to make it look as if a later
decision always existed.

When a durable contract changes substantially:

1. create the new ADR if warranted;
2. explicitly state what earlier decision it changes or supersedes;
3. update active milestone/context documentation;
4. update code and tests consistently.

Small clarifications may update the existing ADR when they do not change the
decision itself.

---

## Relationship to other documents

```text
PROJECT_VISION.md
    where the project may go

CURRENT_MILESTONE.md
    what may be implemented now

ADR
    durable architecture chosen for implementation

AI_CONTEXT.md
    compact summary pointing toward the relevant ADR

code + tests
    executable realization of the contract
```

Avoid copying complete ADR reasoning into `AI_CONTEXT.md`, `README.md` or
`CURRENT_MILESTONE.md`.

Link or point to the ADR instead.

---

## External frameworks

The appearance of an external technology in the project vision does not create
an architectural dependency.

For example:

```text
NVIDIA Isaac Sim
ROS 2
Gazebo
real robot hardware
```

remain external/future systems until an active milestone introduces a concrete
integration and, where necessary, an ADR defines the adapter boundary.

Future simulator integration must preserve the distinction between:

```text
FleetSyncSim domain semantics
```

and:

```text
external physical/runtime representation
```

The deterministic FleetSyncSim reference simulator remains independently
testable.

# Geospatial Development Tooling

|                   |                                                                                                          |
| ----------------- | -------------------------------------------------------------------------------------------------------- |
| **Document type** | Tooling boundary and usage guide                                                                         |
| **Status**        | Accepted boundary — map import and GeoJSON inspection active                                             |
| **Authority**     | Boundary rules below are normative; individual tool choices are directional                              |
| **Scope**         | Geospatial map preparation, inspection, visualization, and future scenario authoring around FleetSyncSim |

---

## 1. Purpose

FleetSyncSim studies sovereign cooperative autonomy using locally held maps, independent world models, and local planning.

Real maps are geographic.

This document defines how external geospatial tooling participates in that workflow without becoming part of the autonomous core.

The central rule is:

> **Geospatial tools are human-facing workstations. FleetSyncSim is the machine-facing autonomy system.**

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

Nothing in the FleetSyncSim domain core may require a desktop GIS application at runtime.

No core type should depend on GeoLibre, QGIS, or another GIS workstation.

There must be no hidden requirement that a GIS application is present for:

* planning;
* simulation;
* localization;
* robot autonomy;
* distributed reconciliation;
* scenario execution.

---

### 3.2 No representation capture

Core map abstractions must not be reshaped merely to match a GIS application's internal model.

For example:

```text
BaseMap
Graph
MapGeometry
DynamicMapOverlay
MapView
```

exist for FleetSyncSim domain semantics.

External formats are translated at explicit boundaries.

GIS convenience must not redefine core ownership, topology, identity, or determinism contracts.

---

### 3.3 Formats are the contract

Exchange between FleetSyncSim and geospatial tooling uses explicit portable formats.

Current important formats are:

```text
OSM PBF
    map input

GeoJSON
    debug / visualization output
```

Additional formats may be introduced only when a concrete use case justifies them.

---

### 3.4 Offline operation matters

Geospatial workflows should remain compatible with local, offline, or air-gapped operation.

The project should not require uploading map data, mission data, or robot traces to an external cloud service merely to inspect them.

---

## 4. Current geospatial flow

### 4.1 Map input

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

The OSM importer converts supported external geographic data into FleetSyncSim domain types.

Parser dependencies remain isolated to the importer target rather than leaking into the rest of the core.

Exact importer contracts are defined by the relevant ADRs.

---

### 4.2 Debug map output

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

This allows visual inspection of imported topology and geometry without making a GIS tool part of simulation behavior.

Example:

```sh
./build/debug/apps/fleet_map_import/fleet_map_import map.osm.pbf \
    --map-geojson map.geojson
```

The GeoJSON adapter is outward-facing and read-only.

It does not alter `BaseMap`, routing, or robot state.

---

### 4.3 Trace output

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

Trace-derived geographic output is also outward-only.

Exporters observe simulation results.

They do not feed information back into robot autonomy.

---

## 5. Development stages

These stages describe useful geospatial capabilities.

They do **not** authorize implementation by themselves.

Implementation scope still comes from `CURRENT_MILESTONE.md`.

---

### G1 — Map inspection — ACTIVE

FleetSyncSim can import supported OSM PBF data and emit deterministic map GeoJSON.

This supports questions such as:

* did the importer retain the expected roads?
* are intersections connected correctly?
* are one-way edges represented as expected?
* is imported geometry geographically located correctly?
* are unexpected disconnected components visible?

A rejected-feature diagnostic layer may be added later if importer debugging justifies it.

It is not required merely because it appears in this document.

---

### G2 — Trace visualization — ACTIVE, INCREMENTAL

FleetSyncSim can export deterministic geographic information derived from simulation traces.

The important architectural property is already established:

```text
simulation state / trace
        |
        v
pure outward exporter
        |
        v
GeoJSON
```

Possible future visualization layers include:

* planned routes per replan;
* observations;
* communication topology;
* robot-local map beliefs;
* world truth;
* localization estimates;
* truth-versus-estimate trajectories.

These are directions, not current implementation requirements.

In particular, visualizing ground truth must never create an information path back into robot autonomy.

---

### G3 — Scenario authoring — LATER

A GIS workstation could eventually help author geographic scenario elements such as:

* missions;
* blocked regions;
* communication blackout regions;
* landmarks;
* geographic experiment areas.

Such tooling should export into FleetSyncSim's existing scenario boundary.

The scenario format must remain understandable and usable without requiring a GIS plugin.

---

### G4 — Field collection — LATER

Field collection may eventually support:

* map validation;
* landmark collection;
* physical experiment reproduction;
* comparison between simulation and real-world runs.

This remains outside current implementation scope.

---

## 6. Preferred exchange formats

### GeoJSON

Default debug and visualization format.

GeoJSON positions use:

```text
[longitude, latitude]
```

FleetSyncSim's internal WGS84 representation uses named latitude and longitude fields.

The conversion occurs explicitly at the output boundary.

Serialized coordinate precision is an outward representation choice only.

It must never replace or feed back into internal map-domain precision.

---

### OSM PBF

Primary source format for offline road-network import.

It is compact, widely available, and suitable for locally controlled workflows.

---

### Possible later formats

If dataset scale or future workflows justify them:

* GeoPackage;
* FlatGeobuf;
* PMTiles;
* MBTiles.

Their presence here does not authorize implementation.

---

## 7. GeoLibre and QGIS

GeoLibre and QGIS are external workstation choices.

They are not FleetSyncSim dependencies.

Either may be used to inspect standard output formats.

The project should prefer portable data contracts over tool-specific integration so that changing workstation software does not affect the autonomy core.

A dedicated plugin may eventually become useful for scenario authoring or interactive analysis, but it is not required by the architecture.

Tool preference may change over time without changing FleetSyncSim's domain model.

---

## 8. What this document does not authorize

This document does **not** authorize:

* GIS libraries inside unrelated FleetSyncSim domain modules;
* cloud map-provider runtime dependencies;
* live visualization coupled into simulation semantics;
* a GIS-controlled robot state;
* automatic implementation of every proposed visualization layer;
* scenario-authoring plugins;
* field-collection features;
* changes to current milestone scope.

Promotion into implementation scope follows `CURRENT_MILESTONE.md`.

---

## 9. Relationship to ADRs

The authoritative architecture details live in the relevant ADR files under:

```text
docs/design_decisions/
```

Geospatial work currently intersects decisions concerning:

* immutable base-map ownership;
* deterministic behavior;
* robot autonomy boundaries;
* world truth versus belief;
* WGS84 map geometry;
* one-way topology;
* OSM import;
* GeoJSON export;
* localization.

Do not duplicate the complete reasoning or exact contracts of those ADRs here.

When exact semantics matter, discover and read the relevant ADR:

```sh
find docs/design_decisions -maxdepth 1 -name 'ADR-*.md' -printf '%f\n' | sort
```

or search by topic:

```sh
rg -n "MapGeometry|OSM|GeoJSON|one-way|WGS84|localization" \
    docs/design_decisions/ADR-*.md
```

This document describes the **tooling boundary**.

The ADRs define the **architecture contracts**.

---

## 10. Relationship to external physical simulation

Geospatial tooling and physical simulation are separate concerns.

```text
GeoLibre / QGIS
    map inspection and geographic visualization

NVIDIA Isaac Sim
    physical world, robot motion, and simulated sensors
```

Isaac Sim does not belong inside the GIS tooling boundary defined by this document.

Its long-term architectural placement is described directionally in `PROJECT_VISION.md`.

Any future Isaac integration becomes implementation scope only when promoted into `CURRENT_MILESTONE.md` and, if it creates a durable architecture boundary, recorded in an ADR.

# Geospatial Development Tooling

| | |
|---|---|
| **Document type** | Tooling boundary and usage guide |
| **Status** | Accepted boundary — tools remain optional |
| **Authority** | Boundary rules below are normative; tool choices are directional |
| **Scope** | Everything *around* FleetSyncSim that is geospatial: map preparation, inspection, visualization, scenario authoring |

---

## 1. Purpose

FleetSyncSim studies sovereign cooperative autonomy: robots that navigate from
**locally held maps**, maintain **independent world models**, and keep operating
when infrastructure disappears. Real maps are geographic. This document defines
how geospatial tooling participates in that goal — and, just as importantly,
where it must stop.

The core principle:

> **Geospatial tools are the human-facing workstation. FleetSyncSim is the
> machine-facing autonomous system.** They communicate only through open data
> formats. Neither ever depends on the other's internals.

---

## 2. The boundary

```text
                      FleetSync ecosystem

                  ┌───────────────────────┐
                  │  GeoLibre / QGIS      │
                  │                       │
                  │  inspect maps         │
                  │  visualize traces     │
                  │  prepare layers       │
                  │  author scenarios     │
                  │  field collection     │
                  └──────────┬────────────┘
                             │
                  open data formats only
                             │
           GeoJSON / OSM PBF / GeoPackage / PMTiles / FlatGeobuf ...
                             │
                             ▼

  ┌──────────────────────────────────────────────────────────┐
  │  FleetSyncSim core (deterministic, C++20, no GIS deps)   │
  │                                                          │
  │  BaseMap → Graph/CSR → MapView → AStarPlanner            │
  │                        ▲                                 │
  │  Robot (overlay + reconciler + route) ← MapDelta ← V2V   │
  │  ControlStation · EventQueue · NetworkSimulator          │
  │                                                          │
  │  later: pose estimation, map matching, PNT degradation   │
  └──────────────────────────────────────────────────────────┘
```

### Normative boundary rules

1. **No runtime dependency.** Nothing in `fleet/` may include, link against,
   call into, or require any GIS application. There is no
   `#include <geolibre/...>` and never will be.
2. **No representation capture.** Core map and planning abstractions
   (`BaseMap`, `Graph`, `DynamicMapOverlay`, `MapView`) must never be reshaped
   to match a GIS application's internal model. Our internal representation
   exists for deterministic planning and simulation, not for tool convenience.
3. **Formats are the only contract.** Every exchange between FleetSyncSim and
   geospatial tooling goes through open, file-based formats (GeoJSON first;
   see section 6).
4. **Sovereignty is preserved on both sides.** Tools must be usable offline,
   self-hosted or air-gapped. A workflow that leaks map data or robot traces
   to a cloud service by default is not acceptable for this project.

A GIS tool that violates rule 4, or a workflow that tempts us to break rules
1-3, is rejected regardless of convenience.

---

## 3. Why this matters for the project's goals

The project vision (`PROJECT_VISION.md`) requires robots to operate from
**locally available map data** with **locally executed planning**. That means
at some point the input to `BaseMap` stops being a hand-written 12-node grid
and becomes a real place:

```text
region.osm.pbf                       (sovereign, offline source data)
        |
map preparation pipeline             (filter, project, simplify)
        |
        +---------------------------> debug_roads.geojson
        |                             opened in GeoLibre / QGIS:
        |                             "did we keep the right roads?"
        v
filtered road graph
        |
BaseMap (+ MapVersion)               (immutable, deterministic)
        |
FleetSyncSim robots                  (local knowledge, local planning)
```

The inspection step is where geospatial tooling first earns its place: a road
graph importer that cannot be **visually verified** is a debugging nightmare.
Terminal output cannot answer "are intersections connected?", "did we drop
one-way streets?", "is this component disconnected?". A road network rendered
on real geography can.

---

## 4. Tool roles: GeoLibre and QGIS

Both are open-source desktop GIS workstations that can run fully locally.
They are complements, not replacements:

| Need | Preferred tool |
|---|---|
| Lightweight, fast map inspection | GeoLibre |
| Sovereign/private workflows, air-gapped deployment | both (GeoLibre documents offline setups with mirrored dependencies) |
| Widest format support, mature specialist plugins, heavy cartography | QGIS |
| FleetSync scenario/debug viewing as it grows | GeoLibre (young, fast-moving, plugin-friendly) |

GeoLibre is a **young project**; treat it as promising tooling, not
infrastructure. If it disappears or stalls, QGIS covers the same boundary —
and the format-only contract (section 2) guarantees the core is unaffected
either way. That robustness is the point of the boundary.

---

## 5. Integration by stage

Only the formats are decided now; each stage is activated when a milestone
needs it, not before.

### Stage G1 — map-import inspection

Activated with the future map-importer milestone. The importer emits,
alongside every built `BaseMap`:

```text
debug_roads.geojson        — the filtered road network it retained
debug_rejected.geojson     — what it dropped and why (categorized)
```

Opened in GeoLibre/QGIS, import correctness becomes a visual question.

### Stage G2 — simulation trace export (the first big payoff)

Pairs with the scenario-runner milestone: the simulator's primary
observability artifact is a deterministic structured trace, and a small
exporter renders it as:

```text
trace_routes.geojson           — planned route per robot, per replan
trace_positions.geojson        — robot positions over time
trace_observations.geojson     — who observed what, where, when
trace_links.geojson            — communication topology + partitions over time
trace_belief_a.geojson         — robot A's dynamic knowledge over time
trace_belief_b.geojson         — robot B's belief (deliberately a separate layer)
```

The belief-per-robot layers are the interesting ones: they make **distributed
disagreement visible** — the exact phenomenon this project exists to study.
Ground truth versus each robot's belief, side by side on a real map, turns
"distributed map synchronization" from a log file into something a human can
see and reason about.

### Stage G3 — scenario authoring (later)

Draw missions, blocked regions and blackout boundaries on the map, export to
the scenario format. Possibly a dedicated plugin eventually. **Not now** —
the scenario format must first prove itself hand-written.

### Stage G4 — field collection (much later)

Ground-truthing real environments for map preparation and experiment
reproduction. Depends on field activity that does not exist yet.

---

## 6. Preferred exchange formats

- **GeoJSON** — default for debug/trace export; trivially consumable everywhere.
- **OSM PBF** — source road data (sovereign, compact, offline).
- **GeoPackage / FlatGeobuf** — larger local datasets.
- **PMTiles / MBTiles** — basemap tiles for fully offline viewing.

Internal FleetSyncSim types are **never** exposed through these formats
directly; exporters translate at the boundary, and importers translate in.

---

## 7. What this document does not authorize

- No GIS dependency of any kind in `fleet/` (see section 2).
- No live UI requirement: FleetSyncSim's observability artifact is the
  deterministic trace; rendering it is an offline, after-the-fact act. A
  "follow" mode over a trace file may come later; it changes nothing
  architecturally.
- No map-provider services: the long-term pipeline consumes local files
  (e.g. a regional OSM extract), not an online map or routing API.
- Nothing in this document is an implementation request for the current
  milestone. Promotion follows the normal path defined in `docs/README.md`.

---

## 8. Relationship to existing decisions

- `ADR-001` — immutable BaseMap: importers produce new `BaseMap` revisions;
  tools never mutate them.
- `ADR-002` — determinism: trace export must be a pure function of the
  simulation run (same seed, identical trace, identical GeoJSON).
- `ADR-007` — robot autonomy boundary: visualization observes robots; it never
  feeds them. A robot may not consume rendered state (that would be oracle
  knowledge).
- `PROJECT_VISION.md` pillars 1 and 3 — sovereign maps and local world models:
  this tooling exists to build and inspect them, not to host them.

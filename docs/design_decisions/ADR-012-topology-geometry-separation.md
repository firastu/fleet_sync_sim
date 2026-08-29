# ADR-012: Topology and geometry are separate map concerns

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::map` (geometry side of BaseMap), the future OSM importer
  (#12A), output adapters (#12B)

## Context

The planning map is graph-oriented: `Graph` (CSR) with `NodePosition`
used only to derive builder-default edge costs and the admissible
Euclidean heuristic. Importing real-world maps (OSM, geospatial doc
stage G1) introduces per-node geographic coordinates (WGS84 latitude /
longitude) and per-edge polylines (the actual road geometry between
intersections), plus coordinate-reference-system metadata. The decision
needed BEFORE the importer lands: does geography become part of the
planning graph, or a separate side of the map?

## Decision

**Topology and geometry stay separated.**

```
                 BaseMap
                    |
          +---------+---------+
          |                   |
       Topology            Geometry
       Graph (CSR)         MapGeometry
       adjacency           node coordinates (NodeId -> WGS84)
       traversal costs     edge polylines (EdgeId -> points)
       NodePosition        CRS metadata
       (cost derivation,
        heuristic only)
```

- `Graph` remains the planning topology and gains nothing geographic.
  The planner (A*), overlays, reconciliation, sensing and movement are
  unchanged and remain unaware of where coordinates came from. Node
  positions stay what they are: inputs for default cost derivation and
  the admissible heuristic — no CRS, no degrees, no geodesy.
- `MapGeometry` is a separate, immutable value owned (optionally) by
  `BaseMap`: node coordinates keyed by NodeId, edge polylines keyed by
  EdgeId, and the CRS identity (initially WGS84 only). Absent geometry
  is legal: every existing test, scenario and fixture stays
  geometry-free, and topology-only maps remain first-class.
- Importers (#12A: OSM PBF) produce `BaseMap = Graph + MapGeometry`:
  the OSM node graph becomes topology; OSM node coordinates and way
  geometries become the geometry side. Import policy (which highways
  become edges, how way tags map to costs) is importer configuration,
  not map semantics.
- Consumers of geometry are the OUTPUT and later PNT sides only: debug
  GeoJSON (#12B), trace→GeoJSON export, and later localization /
  map-matching / pose work. Planning never reads geometry.
- Determinism: geometry is immutable data with dense node coordinates
  and ascending-EdgeId polyline access; no iteration over geometry may
  influence simulation behavior (only exports read it, and exports are
  pure functions of the map + trace).

## Alternatives considered

- **Geographic coordinates inside Graph nodes (lat/lon in Node).**
  Rejected: makes every planner consumer transitively geographic,
  couples cost derivation to a CRS, and forces synthetic maps (unit
  grids, test fixtures) to pretend to be on Earth.
- **Geometry as a separate top-level object owned by the runner/app.**
  Rejected for now: geometry's lifecycle is the BaseMap's (same
  immutability, same version); keeping it beside the graph inside
  BaseMap gives importers one product type and preserves the "all
  participants share the same map revision" invariant.
- **Full geometry library integration (GEOS/Boost.Geometry).**
  Rejected: M2 needs storage and export, not geometric predicates;
  a dependency is added when a demonstrated need survives review.

## Consequences and limitations

- Edge polylines are optional per edge (straight-line rendering is the
  documented fallback when a polyline is absent).
- WGS84 is the only CRS in M2; a CRS field exists so later imports can
  be explicit rather than implicit. Projected/local metric frames
  arrive with the work that needs them.
- Costs imported from OSM (way length-based) are materialized as plain
  `base_cost` values at import time — the graph never recomputes them
  from geometry.

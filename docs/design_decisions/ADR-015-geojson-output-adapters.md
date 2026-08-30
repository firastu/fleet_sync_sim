# ADR-015: GeoJSON output adapters

- Status: Accepted
- Date: 2026-08-30
- Scope: `fleet::geojson` (fleet_geojson target), `apps/fleet_map_import`
  (--map-geojson); activates docs/geospatial.md stage G2

## Context

M2 can ingest real geographic data (#12A). The reverse direction —
internal state leaving the simulator for humans and tools — must be an
open interchange format without coupling any GIS tooling into the core.
GeoJSON (RFC 7946) is that format: GeoLibre/QGIS consume it; FleetSyncSim
never consumes anything from them.

## Decision

### Strictly outward-facing pure functions

- `write_base_map_geojson(map, ostream)` and
  `write_trace_geojson(map, events, ostream)`: read-only, no simulator
  state, no mutation, return the feature count. No GeoJSON types inside
  `BaseMap`, `Robot`, planner, reconciler or simulation; no GeoLibre
  dependency anywhere in C++.
- Both throw `std::invalid_argument` on a map without a geographic side
  (there is nothing legitimate to place) and on trace references to
  unknown/coordinate-less nodes (a wiring bug, not data).

### The coordinate-order boundary

- FleetSyncSim stores `Wgs84Coordinate{latitude_deg, longitude_deg}`;
  GeoJSON positions are `[longitude, latitude]`. The conversion lives in
  exactly ONE function (`position_text`), tested to lock lon-before-lat,
  with fixed 7-decimal formatting (~1 cm) for deterministic output. No
  generic pair type carries coordinates anywhere in the pipeline.

### Deterministic feature ordering and stable properties

- Map export: node Point features in ascending NodeId order, then edge
  LineString features in ascending EdgeId order. Nodes without
  coordinates and edges without both endpoint coordinates are skipped
  (geometry is per-node/per-edge optional, ADR-012).
- Edge geometry: the edge's polyline when present, else the straight
  segment between endpoint coordinates (the documented fallback).
- Properties are stable for tooling correlation: nodes `{name}`; edges
  `{edge, from, to, direction, cost}` (direction from ADR-013).
- Trace export: robots in first-appearance order; one LineString per
  robot reconstructed from departure events (consecutive duplicate
  points merged), one Point per completed mission. Non-geographic
  events (link_state, sends, reconciles, ...) are not exported — the
  JSONL trace (ADR-009) remains the complete record; GeoJSON is the
  geographic projection of it.
- Same inputs => byte-identical output (tested).

### CLI

- `fleet_map_import <map.osm.pbf> [--map-geojson <out.geojson>]` — the
  thin CLI gains one option; all logic stays in the libraries.

## Alternatives considered

- **GeoJSON writing via a JSON library.** Rejected: output needs a
  fixed shape, not general JSON construction; hand-written emission
  (same policy as the trace writer) keeps the module dependency-free.
- **Streaming TraceSink that emits GeoJSON live.** Deferred: trajectories
  need consecutive-event context; the post-run exporter over the
  captured trace is simpler and sufficient for G2. A sink can wrap it
  later without contract change.
- **Exporting every trace event as a feature.** Rejected: noise; the
  JSONL trace is the complete record, GeoJSON is its geographic
  projection.

## Consequences and limitations

- Trace GeoJSON requires the map the trace was produced against (node
  names resolve via the graph); callers hold both — the runner does.
- Only WGS84 (RFC 7946 mandates it); projected CRS output would be a
  future exporter.
- Event-timestamped animation (timestamped feature properties) is future
  work; trajectories are currently un-timed polylines plus completion
  markers.

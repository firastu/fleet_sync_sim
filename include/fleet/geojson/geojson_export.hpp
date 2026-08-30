#pragma once

#include <cstddef>
#include <ostream>
#include <span>

#include <string>

#include "fleet/map/base_map.hpp"
#include "fleet/scenario/trace.hpp"

namespace fleet::geojson {

// Deterministic GeoJSON export adapters (ADR-015, geospatial doc G2).
// Strictly OUTWARD-FACING pure functions: they read the map / the
// captured trace and write text — no simulator, robot, planner or
// exporter state, no mutation, no GeoLibre dependency in C++.
//
// Coordinate-order boundary: FleetSyncSim stores
// Wgs84Coordinate{latitude_deg, longitude_deg}; GeoJSON positions are
// [longitude, latitude]. The conversion happens in exactly one place
// here and is covered by dedicated tests. Coordinates are formatted
// with 7 decimals (~1 cm at mid latitudes) — deterministic output.
//
// Deterministic feature ordering: map export emits node features in
// ascending NodeId order, then edge features in ascending EdgeId
// order; trace export emits robots in first-appearance order in the
// trace. Same inputs => byte-identical output.

// Debug export of a BaseMap's geographic side: one Point feature per
// node that has a coordinate, then one LineString feature per edge
// that has both endpoint coordinates (its polyline when present, the
// straight segment between endpoint coordinates otherwise — the
// documented fallback). Nodes/edges without the needed coordinates are
// skipped; the return value is the number of features written. Throws
// std::invalid_argument when the map carries no geographic side.
// Properties are stable for tooling correlation: nodes {name}; edges
// {edge, from, to, direction, cost}.
[[nodiscard]] std::size_t write_base_map_geojson(const map::BaseMap& map,
                                                 std::ostream& output);

// Trace export: reconstructs each robot's trajectory from departure /
// mission_complete events (arrival nodes chain implicitly through
// consecutive departures) and writes one LineString feature per robot
// plus one Point feature per completed mission. Node names in trace
// events are resolved against the map's graph; an unknown name throws
// std::invalid_argument naming the event (traces are generated against
// the same map, so this only fires on a wiring bug). Events without
// geographic meaning (link_state, sends, reconciles, ...) are not
// exported. Returns the number of features written.
[[nodiscard]] std::size_t write_trace_geojson(const map::BaseMap& map,
                                              std::span<const scenario::TraceEvent> events,
                                              std::ostream& output);

// The one coordinate-order conversion ([lon, lat] position text) —
// exposed for tests that lock the lon-before-lat contract.
[[nodiscard]] std::string position_text(const map::Wgs84Coordinate& coordinate);

}  // namespace fleet::geojson

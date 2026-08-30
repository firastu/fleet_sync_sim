#pragma once

#include <cstddef>
#include <filesystem>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"

namespace fleet::osm {

// Import diagnostics (ADR-014). "directed_edges" counts CREATED GRAPH
// EDGES: a bidirectional segment produces one edge (traversable both
// ways), a one-way segment produces one edge (traversable one way).
struct OsmImportStats {
    std::size_t ways_seen = 0;       // every OSM way object in the file
    std::size_t ways_imported = 0;   // eligible under the way policy
    std::size_t topology_nodes = 0;  // retained decision nodes
    std::size_t directed_edges = 0;  // graph edges created
};

struct OsmImportResult {
    map::BaseMap map;
    OsmImportStats stats;
};

struct OsmImportOptions {
    common::MapVersion map_version{1};
};

// Deterministic OSM PBF importer: real geographic data becomes a
// BaseMap (Graph + MapGeometry) without any OSM concept leaking into
// the map core (ADR-012/013/014). This public header exposes only
// FleetSyncSim types; libosmium/protozero are PRIVATE implementation
// details of the fleet_osm_import target — nothing outside it includes
// <osmium/...> or <protozero/...>.
//
// Contracts (all enforced, all tested):
//   - Way eligibility policy is separate from PBF decoding: ways with a
//     highway tag from the documented whitelist (motorway, trunk,
//     primary, secondary, tertiary, unclassified, residential, service,
//     living_street, road, track) and >= 2 node refs are imported;
//     everything else is skipped and counted in ways_seen only.
//   - Retained topology nodes: way endpoints and nodes referenced >= 2
//     times across eligible way node refs (self-intersections count).
//     Ways are split into segments between consecutive retained nodes;
//     intermediate way nodes are NOT graph nodes — their coordinates
//     live on in the edge polylines (ADR-012).
//   - NodeIds: retained OSM node ids sorted ascending -> dense ids.
//   - EdgeIds: segments sorted by the canonical key
//     (osm_way_id, segment_index_within_way, direction) -> creation
//     order. Independent of PBF encounter order; no unordered-container
//     iteration influences any assignment.
//   - oneway: absent/no/false/0 -> bidirectional; yes/true/1 -> forward
//     (way direction only); -1 -> reverse (against way direction only).
//     Any other value fails the import loudly (way id and value named).
//     junction=roundabout with an ABSENT oneway tag carries an implicit
//     direction in OSM and also fails the import loudly (importing it as
//     bidirectional would confidently create wrong topology); explicit
//     oneway values on roundabouts go through the normal parser.
//   - Costs: geodesic (haversine, mean earth radius 6371008.8 m) length
//     of the segment polyline in meters, computed in the importer and
//     handed to the graph as a plain positive cost. A non-positive or
//     non-finite computed length fails the import deterministically
//     (way and segment named) — no manufactured costs.
//   - Polylines reuse the exact canonical node-coordinate VALUES at
//     their endpoints (MapGeometry's exact-equality identity contract).
//   - Anti-parallel/multi-edge topology between the same retained node
//     pair fails the import explicitly (ADR-013) — never silently
//     merged or dropped.
//   - Import-twice determinism: same PBF + same options produce the
//     identical map (ids, adjacency, costs, coordinates, polylines) and
//     identical stats.
//
// Errors are std::invalid_argument / std::runtime_error with
// deterministic messages naming the offending OSM object.
class OsmPbfImporter {
public:
    [[nodiscard]] OsmImportOptions options() const noexcept { return options_; }

    explicit OsmPbfImporter(OsmImportOptions options = {});

    [[nodiscard]] OsmImportResult import(const std::filesystem::path& pbf_file) const;

private:
    OsmImportOptions options_;
};

}  // namespace fleet::osm

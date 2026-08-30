#pragma once

#include "fleet/common/ids.hpp"
#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::map {

// The immutable baseline topology every participant starts from. All
// participants of a simulation run share the same MapVersion; a revision of
// the road network is a new BaseMap object, never in-place mutation
// (see ADR-001).
//
// The map has two sides (ADR-012): the required planning topology
// (Graph) and an optional geographic side (MapGeometry: node
// coordinates, edge polylines, CRS). Planning never reads geometry;
// geometry exists for import (OSM), export (GeoJSON) and later
// PNT/map-matching work. A BaseMap without geometry is fully valid.
//
// Thread-safety: immutable after construction; safe to share without
// synchronization.
class BaseMap {
public:
    // Topology-only map.
    BaseMap(Graph graph, common::MapVersion version) noexcept;

    // Topology + geographic side. Throws std::invalid_argument when the
    // geometry's sizes do not match the graph, or when an edge polyline
    // contradicts its graph edge's orientation (front must be the `from`
    // node's coordinate, back the `to` node's — checked whenever both
    // endpoint coordinates exist).
    BaseMap(Graph graph, common::MapVersion version, MapGeometry geometry);

    [[nodiscard]] const Graph& graph() const noexcept { return graph_; }
    [[nodiscard]] common::MapVersion version() const noexcept { return version_; }

    // nullptr when this map carries no geographic side.
    //
    // Ownership note: const pointer into this BaseMap's own copy.
    // MapGeometry offers NO mutation API after build() (its Builder is a
    // separate, single-shot type), so no alias obtained here can mutate
    // the geographic side behind an existing BaseMap — effectively
    // immutable shared ownership.
    [[nodiscard]] const MapGeometry* geometry() const noexcept { return geometry_.has_value()
                                                                     ? &*geometry_
                                                                     : nullptr; }

private:
    Graph graph_;
    common::MapVersion version_{0};
    std::optional<MapGeometry> geometry_;
};

}  // namespace fleet::map

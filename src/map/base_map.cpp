#include "fleet/map/base_map.hpp"

#include <optional>
#include <stdexcept>
#include <utility>

namespace fleet::map {

BaseMap::BaseMap(Graph graph, common::MapVersion version) noexcept
    : graph_{std::move(graph)}, version_{version} {}

BaseMap::BaseMap(Graph graph, common::MapVersion version, MapGeometry geometry)
    : graph_{std::move(graph)}, version_{version} {
    if (geometry.node_count() != graph_.node_count() ||
        geometry.edge_count() != graph_.edge_count()) {
        throw std::invalid_argument(
            "BaseMap: geometry sizes must match the graph's node and edge counts");
    }
    // Orientation contract (ADR-012): a polyline follows its edge from ->
    // to, verified with EXACT coordinate equality — an identity/attachment
    // check, not geometric proximity. Importers reuse the canonical node
    // coordinate values rather than recomputing them. Checkable whenever
    // both endpoint coordinates exist; endpoints without coordinates keep
    // the documented straight-line fallback and are not checked.
    for (std::size_t index = 0; index < graph_.edge_count(); ++index) {
        const common::EdgeId edge_id{static_cast<std::uint32_t>(index)};
        const std::vector<Wgs84Coordinate>* polyline = geometry.edge_polyline(edge_id);
        if (polyline == nullptr) {
            continue;
        }
        const Edge& edge = graph_.edge(edge_id);
        const Wgs84Coordinate* from_position = geometry.node_position(edge.a);
        const Wgs84Coordinate* to_position = geometry.node_position(edge.b);
        if (from_position != nullptr && polyline->front() != *from_position) {
            throw std::invalid_argument(
                "BaseMap: edge polyline must start at the edge's from-node coordinate");
        }
        if (to_position != nullptr && polyline->back() != *to_position) {
            throw std::invalid_argument(
                "BaseMap: edge polyline must end at the edge's to-node coordinate");
        }
    }
    geometry_ = std::move(geometry);
}

}  // namespace fleet::map

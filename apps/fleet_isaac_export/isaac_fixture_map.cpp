#include "isaac_fixture_map.hpp"

#include <utility>
#include <vector>

#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::isaac {

FixtureMap build_fixture_map() {
    using map::NodePosition;
    map::Graph::Builder builder;

    std::map<std::string, common::NodeId> node_ids;
    const std::vector<std::pair<std::string, NodePosition>> layout{
        {"A", NodePosition{0.0, 0.0}},
        {"B", NodePosition{0.0, 1.0}},
        {"C", NodePosition{1.0, 1.0}}};
    for (const auto& [name, position] : layout) {
        node_ids.emplace(name, builder.add_node(name, position));
    }
    builder.connect(node_ids.at("A"), node_ids.at("B"));
    builder.connect(node_ids.at("B"), node_ids.at("C"));

    const map::Graph graph = builder.build();

    // Geographic side (ADR-020): A anchors at the pinned experiment origin;
    // 0.0002 deg latitude ~ 22.2 m north to B; 0.0003 deg longitude
    // ~ 20.4 m east to C. No edge polylines: node-coordinate geometry with
    // the straight-segment interpolation fallback (ADR-012).
    map::MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    geometry.set_node_position(
        node_ids.at("A"),
        map::Wgs84Coordinate{kFixtureOriginLatitude, kFixtureOriginLongitude});
    geometry.set_node_position(
        node_ids.at("B"),
        map::Wgs84Coordinate{kFixtureOriginLatitude + 0.0002, kFixtureOriginLongitude});
    geometry.set_node_position(
        node_ids.at("C"),
        map::Wgs84Coordinate{kFixtureOriginLatitude + 0.0002, kFixtureOriginLongitude + 0.0003});

    return FixtureMap{map::BaseMap{graph, common::MapVersion{1}, geometry.build()},
                      std::move(node_ids)};
}

}  // namespace fleet::isaac

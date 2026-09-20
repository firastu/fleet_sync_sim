#include "demo_map.hpp"

#include <map>
#include <utility>
#include <vector>

#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::app {

DemoMap build_demo_map() {
    using fleet::map::NodePosition;
    fleet::map::Graph::Builder builder;

    std::map<std::string, fleet::common::NodeId> node_ids;
    const std::vector<std::pair<std::string, NodePosition>> layout{
        {"A", NodePosition{0, 0}}, {"B", NodePosition{1, 0}}, {"C", NodePosition{2, 0}},
        {"D", NodePosition{3, 0}}, {"E", NodePosition{0, 1}}, {"F", NodePosition{1, 1}},
        {"G", NodePosition{2, 1}}, {"H", NodePosition{3, 1}}, {"I", NodePosition{0, 2}},
        {"J", NodePosition{1, 2}}, {"K", NodePosition{2, 2}}, {"L", NodePosition{3, 2}}};
    for (const auto& [name, position] : layout) {
        node_ids.emplace(name, builder.add_node(name, position));
    }

    const auto connect = [&builder, &node_ids](const char* a, const char* b) {
        builder.connect(node_ids.at(a), node_ids.at(b));
    };
    // Rows, then columns: insertion order defines adjacency order.
    connect("A", "B");
    connect("B", "C");
    connect("C", "D");
    connect("E", "F");
    connect("F", "G");
    connect("G", "H");
    connect("I", "J");
    connect("J", "K");
    connect("K", "L");
    connect("A", "E");
    connect("E", "I");
    connect("B", "F");
    connect("F", "J");
    connect("C", "G");
    connect("G", "K");
    connect("D", "H");
    connect("H", "L");

    const fleet::map::Graph graph = builder.build();

    // Geographic side (#16, ADR-018): canonical WGS84 coordinates for the
    // same grid — A anchors at (52.370, 9.730), one grid unit is 0.002
    // degrees (columns east, rows north). Node positions only (no edge
    // polylines): edges render — and their truth pose interpolates — as
    // straight segments, the documented fallback (ADR-012). Planning
    // never reads this; pre-#16 scenarios are unaffected by it.
    fleet::map::MapGeometry::Builder geometry_builder{graph.node_count(),
                                                      graph.edge_count()};
    for (const auto& [name, position] : layout) {
        geometry_builder.set_node_position(
            node_ids.at(name),
            fleet::map::Wgs84Coordinate{52.370 + 0.002 * position.y,
                                        9.730 + 0.002 * position.x});
    }

    return DemoMap{fleet::map::BaseMap{graph, fleet::common::MapVersion{1},
                                       geometry_builder.build()},
                   std::move(node_ids)};
}

}  // namespace fleet::app

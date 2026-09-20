#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::testsupport {

// The 3x4 road network used across unit tests and (later) scenarios:
//
//   A -- B -- C -- D
//   |    |    |    |
//   E -- F -- G -- H
//   |    |    |    |
//   I -- J -- K -- L
//
// Unit-spaced grid; every edge defaults to cost 1.0 (Euclidean distance).
struct GridMap {
    map::BaseMap base;
    std::map<std::string, common::NodeId> node_ids;  // ordered map: deterministic lookups

    [[nodiscard]] common::NodeId node(const std::string& name) const {
        return node_ids.at(name);
    }
};

[[nodiscard]] inline GridMap make_grid_map(common::MapVersion version = common::MapVersion{1}) {
    using map::NodePosition;
    map::Graph::Builder builder;

    std::map<std::string, common::NodeId> node_ids;
    for (const auto& [name, position] :
         std::vector<std::pair<std::string, NodePosition>>{
             {"A", NodePosition{0, 0}}, {"B", NodePosition{1, 0}}, {"C", NodePosition{2, 0}},
             {"D", NodePosition{3, 0}}, {"E", NodePosition{0, 1}}, {"F", NodePosition{1, 1}},
             {"G", NodePosition{2, 1}}, {"H", NodePosition{3, 1}}, {"I", NodePosition{0, 2}},
             {"J", NodePosition{1, 2}}, {"K", NodePosition{2, 2}}, {"L", NodePosition{3, 2}}}) {
        node_ids.emplace(name, builder.add_node(name, position));
    }

    const auto connect = [&builder, &node_ids](const char* a, const char* b) {
        builder.connect(node_ids.at(a), node_ids.at(b));
    };
    // Rows, then columns: insertion order matters for adjacency-order tests.
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

    return GridMap{map::BaseMap{builder.build(), version}, std::move(node_ids)};
}

// The same 3x4 grid WITH the canonical WGS84 geographic side (#16,
// ADR-018) — the same coordinate scheme as the apps' demo map: A anchors
// at (52.370, 9.730), one grid unit is 0.002 degrees (columns east, rows
// north). Node positions only; edge truth pose interpolates straight
// segments (the ADR-012 fallback). For localization-enabled tests.
[[nodiscard]] inline GridMap make_grid_map_with_geometry(
    common::MapVersion version = common::MapVersion{1}) {
    using map::NodePosition;
    map::Graph::Builder builder;

    std::map<std::string, common::NodeId> node_ids;
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

    const map::Graph graph = builder.build();
    map::MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    for (const auto& [name, position] : layout) {
        geometry.set_node_position(
            node_ids.at(name),
            map::Wgs84Coordinate{52.370 + 0.002 * position.y, 9.730 + 0.002 * position.x});
    }
    return GridMap{map::BaseMap{graph, version, geometry.build()}, std::move(node_ids)};
}


}  // namespace fleet::testsupport

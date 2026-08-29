#include "demo_map.hpp"

#include <utility>
#include <vector>

#include "fleet/map/graph.hpp"

namespace fleet::app {

DemoMap build_demo_map() {
    using fleet::map::NodePosition;
    fleet::map::Graph::Builder builder;

    std::map<std::string, fleet::common::NodeId> node_ids;
    for (const auto& [name, position] :
         std::vector<std::pair<std::string, fleet::map::NodePosition>>{
             {"A", NodePosition{0, 0}}, {"B", NodePosition{1, 0}}, {"C", NodePosition{2, 0}},
             {"D", NodePosition{3, 0}}, {"E", NodePosition{0, 1}}, {"F", NodePosition{1, 1}},
             {"G", NodePosition{2, 1}}, {"H", NodePosition{3, 1}}, {"I", NodePosition{0, 2}},
             {"J", NodePosition{1, 2}}, {"K", NodePosition{2, 2}}, {"L", NodePosition{3, 2}}}) {
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

    return DemoMap{fleet::map::BaseMap{builder.build(), fleet::common::MapVersion{1}},
                   std::move(node_ids)};
}

}  // namespace fleet::app

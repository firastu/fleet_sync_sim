// Minimal developer entry point for the map core: builds the demo road
// network, applies simulated observations directly to an overlay, and
// prints the composed state through a MapView.
//
// Robots, planning, networking and the event-driven scenario runner arrive
// in later commits; this executable exists so the project is runnable from
// the first commit that has behavior worth running. The --scenario/--seed
// CLI arrives with the scenario runner.

#include <cstdlib>
#include <format>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/map/map_view.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::MapVersion;
using fleet::common::NodeId;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::BaseMap;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::map::Graph;
using fleet::map::MapView;
using fleet::map::NodePosition;

// Demo road network (same topology as the unit-test fixture):
//
//   A -- B -- C -- D
//   |    |    |    |
//   E -- F -- G -- H
//   |    |    |    |
//   I -- J -- K -- L
//
// Built here instead of sharing tests/support/: applications must not
// depend on test-support code, and the scenario loader of a later commit
// will replace hand-built maps in any case.
struct DemoMap {
    BaseMap base;
    std::map<std::string, NodeId> node_ids;  // ordered map: deterministic lookups

    [[nodiscard]] NodeId node(std::string_view name) const {
        return node_ids.at(std::string{name});
    }
};

DemoMap build_demo_map() {
    Graph::Builder builder;
    std::map<std::string, NodeId> node_ids;
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

    return DemoMap{BaseMap{builder.build(), MapVersion{1}}, std::move(node_ids)};
}

EdgeId edge_for(const DemoMap& demo, std::string_view a, std::string_view b) {
    const std::optional<EdgeId> edge =
        demo.base.graph().edge_between(demo.node(a), demo.node(b));
    if (!edge.has_value()) {
        throw std::runtime_error(std::format("demo map has no edge {}-{}", a, b));
    }
    return *edge;
}

std::string_view status_name(EdgeStatus status) {
    return status == EdgeStatus::Open ? "OPEN" : "BLOCKED";
}

void run() {
    const DemoMap demo = build_demo_map();
    const Graph& graph = demo.base.graph();

    std::cout << "FleetSyncSim\n";
    std::cout << "milestone 1: deterministic reference simulator (map core)\n\n";

    std::cout << std::format("base map: version={} nodes={} edges={}\n",
                             demo.base.version().value(), graph.node_count(),
                             graph.edge_count());
    std::cout << "nodes:";
    for (const auto& node : graph.nodes()) {
        std::cout << ' ' << node.name;
    }
    std::cout << '\n';

    std::cout << "edges:\n";
    for (const auto& edge : graph.edges()) {
        std::cout << std::format("  [{:>2}] {}-{} cost={:.2f}\n", edge.id.value(),
                                 graph.node(edge.a).name, graph.node(edge.b).name,
                                 edge.base_cost);
    }

    DynamicMapOverlay overlay{graph.edge_count()};

    // Two simulated observations. The observation pipeline and network
    // transport arrive in later commits; these states are applied directly
    // to demonstrate overlay storage and view composition. Applying to an
    // empty overlay must change state, so a no-op return is a demo bug.
    const auto apply_observation = [&overlay](EdgeId edge, EdgeDynamicState state) {
        if (!overlay.apply(edge, state)) {
            throw std::runtime_error(
                std::format("observation on edge {} was a no-op", edge.value()));
        }
    };

    const EdgeId fg = edge_for(demo, "F", "G");
    const EdgeId kl = edge_for(demo, "K", "L");
    apply_observation(fg, EdgeDynamicState{
                              .status = EdgeStatus::Blocked,
                              .observed_at = Tick{5000},
                              .source = RobotId{1},
                              .source_sequence = 7,
                              .confidence = 0.9,
                          });
    apply_observation(kl, EdgeDynamicState{
                              .status = EdgeStatus::Blocked,
                              .observed_at = Tick{6200},
                              .source = RobotId{2},
                              .source_sequence = 12,
                              .confidence = 0.75,
                          });

    const MapView view{demo.base, overlay};

    std::cout << std::format("\ndynamic overlay: version={} tracked={}\n",
                             overlay.version().value(), overlay.tracked_count());
    for (const EdgeId edge : overlay.tracked_edges()) {
        const EdgeDynamicState& state = *view.dynamic_state(edge);
        const auto& base_edge = graph.edge(edge);
        std::cout << std::format(
            "  edge={:>2} ({}-{}) status={} observed_at_tick={} source={} sequence={} "
            "confidence={:.2}\n",
            edge.value(), graph.node(base_edge.a).name, graph.node(base_edge.b).name,
            status_name(state.status), state.observed_at.value, state.source.value(),
            state.source_sequence, state.confidence);
    }

    std::cout << "\nmap view checks:\n";
    for (const auto& [edge, label] :
         std::vector<std::pair<EdgeId, const char*>>{{edge_for(demo, "A", "B"), "A-B"},
                                                     {edge_for(demo, "E", "F"), "E-F"},
                                                     {fg, "F-G"},
                                                     {edge_for(demo, "G", "H"), "G-H"},
                                                     {kl, "K-L"}}) {
        const std::optional<double> cost = view.traversal_cost(edge);
        std::cout << std::format(
            "  {}: {}\n", label,
            cost.has_value() ? std::format("cost={:.2f}", *cost) : std::string{"BLOCKED"});
    }
}

}  // namespace

int main() {
    try {
        run();
    } catch (const std::exception& error) {
        std::cerr << std::format("fleet_sim: fatal: {}\n", error.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

// Minimal developer entry point: build the demo network, plan a mission,
// inject an observation that invalidates the route, and replan around it.
//
// Robots, networking and the event-driven scenario runner arrive in later
// commits; the --scenario/--seed CLI arrives with the scenario runner.

#include <cstdlib>
#include <format>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "demo_map.hpp"

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/map/map_reconciler.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::planning::AStarPlanner;
using fleet::planning::Route;

std::string format_route(const Route& route, const fleet::map::Graph& graph) {
    if (!route.found) {
        return "<no route>";
    }
    std::string text;
    for (std::size_t i = 0; i < route.nodes.size(); ++i) {
        if (i > 0) {
            text += " -> ";
        }
        text += graph.node(route.nodes[i]).name;
    }
    return text + std::format(" cost={:.2f}", route.cost);
}

EdgeId edge_for(const fleet::app::DemoMap& demo, const char* a, const char* b) {
    const std::optional<EdgeId> edge =
        demo.base.graph().edge_between(demo.node(a), demo.node(b));
    if (!edge.has_value()) {
        throw std::runtime_error(std::format("demo map has no edge {}-{}", a, b));
    }
    return *edge;
}

std::string_view status_name(fleet::map::EdgeStatus status) {
    return status == fleet::map::EdgeStatus::Open ? "OPEN" : "BLOCKED";
}

std::string_view decision_name(fleet::map::ReconcileDecision decision) {
    if (decision == fleet::map::ReconcileDecision::Applied) {
        return "APPLIED";
    }
    if (decision == fleet::map::ReconcileDecision::Duplicate) {
        return "DUPLICATE";
    }
    if (decision == fleet::map::ReconcileDecision::Stale) {
        return "STALE";
    }
    if (decision == fleet::map::ReconcileDecision::Dominated) {
        return "DOMINATED";
    }
    return "REJECTED_CONFLICT";
}

void run() {
    const fleet::app::DemoMap demo = fleet::app::build_demo_map();
    const fleet::map::Graph& graph = demo.base.graph();
    fleet::map::DynamicMapOverlay overlay{graph.edge_count()};
    const fleet::map::MapView view{demo.base, overlay};

    std::cout << "FleetSyncSim\n";
    std::cout << "milestone 1: deterministic reference simulator (map core + planning)\n\n";

    std::cout << std::format("base map: version={} nodes={} edges={}\n",
                             demo.base.version().value(), graph.node_count(),
                             graph.edge_count());
    std::cout << "nodes:";
    for (const auto& node : graph.nodes()) {
        std::cout << ' ' << node.name;
    }
    std::cout << "\n\n";

    // Default zero heuristic: Dijkstra-equivalent and always optimal.
    AStarPlanner planner;

    const Route initial = planner.plan(view, demo.node("A"), demo.node("D"));
    if (!initial.found) {
        throw std::runtime_error("demo invariant violated: A->D must be plannable");
    }
    std::cout << std::format("mission: A -> D\ninitial route: {}\n\n",
                             format_route(initial, graph));

    // Observation at tick 5000 blocks an edge that is on the initial
    // route. It is turned into a MapDelta and goes through the same
    // reconciliation path a remotely received delta would (ADR-004);
    // overlay.apply() remains the low-level storage primitive beneath it.
    const EdgeId blocked = edge_for(demo, "B", "C");
    const fleet::map::MapDelta observation{
        blocked,
        fleet::map::EdgeDynamicState{
            .status = fleet::map::EdgeStatus::Blocked,
            .observed_at = Tick{5000},
            .source = RobotId{1},
            .source_sequence = fleet::common::SequenceNumber{7},
            .confidence = 0.9,
        },
    };
    fleet::map::MapReconciler reconciler{graph.edge_count()};

    std::cout << "[5000] local observation: edge B-C BLOCKED"
                 " (delta source=1 seq=7)\n";
    std::cout << std::format("reconcile: decision={}\n",
                             decision_name(reconciler.reconcile(observation, overlay)));

    // Re-delivery hazards. The unreliable network arrives in a later
    // commit; these exercise reconciliation determinism directly.
    std::cout << std::format("duplicate re-delivery: decision={}\n",
                             decision_name(reconciler.reconcile(observation, overlay)));

    const fleet::map::MapDelta stale{
        blocked,
        fleet::map::EdgeDynamicState{
            .status = fleet::map::EdgeStatus::Blocked,
            .observed_at = Tick{4990},
            .source = RobotId{1},
            .source_sequence = fleet::common::SequenceNumber{6},
            .confidence = 0.9,
        },
    };
    std::cout << std::format("stale re-delivery (seq=6): decision={}\n",
                             decision_name(reconciler.reconcile(stale, overlay)));

    // New valid event from another source whose observation is older than
    // the winner's: processed (progression advances) but dominated.
    const fleet::map::MapDelta dominated{
        blocked,
        fleet::map::EdgeDynamicState{
            .status = fleet::map::EdgeStatus::Open,
            .observed_at = Tick{4500},
            .source = RobotId{2},
            .source_sequence = fleet::common::SequenceNumber{1},
            .confidence = 0.8,
        },
    };
    std::cout << std::format("older cross-source observation (source=2 tick=4500): decision={}\n",
                             decision_name(reconciler.reconcile(dominated, overlay)));

    std::cout << std::format("route invalidated: current route uses edge B-C = {}\n",
                             initial.uses_edge(blocked) ? "true" : "false");

    const Route rerouted = planner.plan(view, demo.node("A"), demo.node("D"));
    if (!rerouted.found) {
        throw std::runtime_error("demo invariant violated: A->D must remain plannable");
    }
    std::cout << std::format("rerouted route: {}\n\n", format_route(rerouted, graph));

    std::cout << std::format("dynamic overlay: version={} tracked={}\n",
                             overlay.version().value(), overlay.tracked_count());
    for (const EdgeId edge : overlay.tracked_edges()) {
        const fleet::map::EdgeDynamicState& state = *view.dynamic_state(edge);
        const auto& base_edge = graph.edge(edge);
        std::cout << std::format(
            "  edge={:>2} ({}-{}) status={} observed_at_tick={} source={} sequence={} "
            "confidence={:.2}\n",
            edge.value(), graph.node(base_edge.a).name, graph.node(base_edge.b).name,
            status_name(state.status), state.observed_at.value, state.source.value(),
            state.source_sequence.value(), state.confidence);
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

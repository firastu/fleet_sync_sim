// Minimal developer entry point: build the demo world, plan missions for
// two participants, let one observe a blocked edge, share the delta over
// the simulated network, and reroute both participants.
//
// No Robot abstraction yet (commit #7); participants here are just an
// overlay + reconciler each, connected through NetworkSimulator. The
// --scenario/--seed CLI arrives with the scenario runner.

#include <cstdint>
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
#include "fleet/network/network_simulator.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"
#include "fleet/simulation/event_queue.hpp"

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

std::string edge_label(const fleet::map::Graph& graph, EdgeId edge) {
    const fleet::map::Edge& base_edge = graph.edge(edge);
    return std::format("{}-{}", graph.node(base_edge.a).name, graph.node(base_edge.b).name);
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

    // Two participants, A and B, each with their own overlay and
    // reconciler, connected by a simulated network link (fixed 80 ms
    // latency, ideal link). No Robot abstraction yet.
    fleet::map::DynamicMapOverlay overlay_a{graph.edge_count()};
    fleet::map::DynamicMapOverlay overlay_b{graph.edge_count()};
    fleet::map::MapReconciler reconciler_a{graph.edge_count()};
    fleet::map::MapReconciler reconciler_b{graph.edge_count()};
    const fleet::map::MapView view_a{demo.base, overlay_a};
    const fleet::map::MapView view_b{demo.base, overlay_b};

    fleet::simulation::EventQueue queue;
    fleet::network::NetworkSimulator network{
        queue, fleet::network::NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}},
        /*seed=*/1};
    const fleet::network::EndpointId endpoint_a{1};
    const fleet::network::EndpointId endpoint_b{2};

    // Default zero heuristic: Dijkstra-equivalent and always optimal.
    const AStarPlanner planner;
    const auto mission_from = demo.node("A");
    const auto mission_to = demo.node("D");

    const Route initial_a = planner.plan(view_a, mission_from, mission_to);
    const Route initial_b = planner.plan(view_b, mission_from, mission_to);
    if (!initial_a.found || !initial_b.found) {
        throw std::runtime_error("demo invariant violated: A->D must be plannable");
    }

    // B's receive path: reconcile, then replan if the route is hit.
    network.add_endpoint(
        endpoint_b, [&](fleet::network::EndpointId from, const fleet::map::MapDelta& delta) {
            const std::uint64_t now = queue.clock().now().value;
            std::cout << std::format(
                "[{}][NETWORK] endpoint={} received delta (source={} seq={} edge={})\n", now,
                from.value(), delta.state.source.value(), delta.state.source_sequence.value(),
                edge_label(graph, delta.edge));
            std::cout << std::format("[{}][ROBOT_B] reconcile: decision={}\n", now,
                                     decision_name(reconciler_b.reconcile(delta, overlay_b)));
            if (initial_b.uses_edge(delta.edge)) {
                const Route rerouted = planner.plan(view_b, mission_from, mission_to);
                if (!rerouted.found) {
                    throw std::runtime_error("demo invariant violated: B must reroute");
                }
                std::cout << std::format("[{}][ROBOT_B] route invalidated (uses {}): rerouted {}\n",
                                         now, edge_label(graph, delta.edge),
                                         format_route(rerouted, graph));
            }
        });

    std::cout << "FleetSyncSim\n";
    std::cout << "milestone 1: deterministic reference simulator"
                 " (map + planning + reconciliation + network)\n\n";
    std::cout << std::format("base map: version={} nodes={} edges={}\n",
                             demo.base.version().value(), graph.node_count(),
                             graph.edge_count());
    std::cout << "nodes:";
    for (const auto& node : graph.nodes()) {
        std::cout << ' ' << node.name;
    }
    std::cout << "\n\n";
    std::cout << std::format("mission A: A -> D\ninitial route A: {}\n",
                             format_route(initial_a, graph));
    std::cout << std::format("mission B: A -> D\ninitial route B: {}\n\n",
                             format_route(initial_b, graph));

    // t = 5000: A observes an edge on its own route and shares the delta.
    queue.run_until(Tick{5000});
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

    std::cout << "[5000][ROBOT_A] observation: edge B-C BLOCKED (delta source=1 seq=7)\n";
    std::cout << std::format("[5000][ROBOT_A] reconcile: decision={}\n",
                             decision_name(reconciler_a.reconcile(observation, overlay_a)));
    if (initial_a.uses_edge(blocked)) {
        const Route rerouted = planner.plan(view_a, mission_from, mission_to);
        std::cout << std::format("[5000][ROBOT_A] route invalidated (uses B-C): rerouted {}\n",
                                 format_route(rerouted, graph));
    }

    const fleet::network::SendResult sent = network.send(endpoint_a, endpoint_b, observation);
    if (sent.scheduled_deliveries() != 1U) {
        throw std::runtime_error("demo invariant violated: ideal link schedules one delivery");
    }
    std::cout << std::format(
        "[5000][NETWORK] send endpoint=1 -> endpoint=2 (delta source=1 seq=7 edge=B-C);"
        " delivery scheduled @{}\n",
        sent.delivery_ticks.front().value);

    // Re-delivery hazards against A's own knowledge (lossy-link scenarios
    // arrive with the scenario runner).
    std::cout << std::format("[5000][ROBOT_A] duplicate re-delivery: decision={}\n",
                             decision_name(reconciler_a.reconcile(observation, overlay_a)));

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
    std::cout << std::format("[5000][ROBOT_A] stale re-delivery (seq=6): decision={}\n",
                             decision_name(reconciler_a.reconcile(stale, overlay_a)));

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
    std::cout << std::format(
        "[5000][ROBOT_A] older cross-source observation (source=2 tick=4500): decision={}\n",
        decision_name(reconciler_a.reconcile(dominated, overlay_a)));

    // Advance simulated time: B receives the delta at 5080 and reroutes.
    queue.run_until(Tick{6000});

    std::cout << std::format("\nrobot B overlay: version={} tracked={}\n",
                             overlay_b.version().value(), overlay_b.tracked_count());
    for (const EdgeId edge : overlay_b.tracked_edges()) {
        const fleet::map::EdgeDynamicState& state = *view_b.dynamic_state(edge);
        std::cout << std::format(
            "  edge={:>2} ({}) status={} observed_at_tick={} source={} sequence={} "
            "confidence={:.2}\n",
            edge.value(), edge_label(graph, edge), status_name(state.status),
            state.observed_at.value, state.source.value(), state.source_sequence.value(),
            state.confidence);
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

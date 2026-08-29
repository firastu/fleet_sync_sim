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
#include "fleet/robot/robot.hpp"
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

    fleet::simulation::EventQueue queue;
    fleet::network::NetworkSimulator network{
        queue, fleet::network::NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}},
        /*seed=*/1};
    const fleet::network::EndpointId endpoint_a{1};
    const fleet::network::EndpointId endpoint_b{2};

    // Autonomous participants: each robot owns its mission, overlay,
    // reconciler and current route, and shares observations through a
    // sink wired to the simulated link. The app coordinates wiring only.
    fleet::robot::Robot robot_a{
        RobotId{1},
        fleet::robot::Mission{demo.node("A"), demo.node("D")},
        demo.base,
        [&](const fleet::map::MapDelta& delta) {
            const auto result = network.send(endpoint_a, endpoint_b, delta);
            for (const Tick at : result.delivery_ticks) {
                std::cout << std::format(
                    "[{}][NETWORK] send endpoint=1 -> endpoint=2 (delta source={} seq={});"
                    " delivery scheduled @{}\n",
                    queue.clock().now().value, delta.state.source.value(),
                    delta.state.source_sequence.value(), at.value);
            }
        }};
    fleet::robot::Robot robot_b{
        RobotId{2},
        fleet::robot::Mission{demo.node("I"), demo.node("H")},
        demo.base,
        [&](const fleet::map::MapDelta& delta) {
            const auto result = network.send(endpoint_b, endpoint_a, delta);
            for (const Tick at : result.delivery_ticks) {
                std::cout << std::format(
                    "[{}][NETWORK] send endpoint=2 -> endpoint=1 (delta source={} seq={});"
                    " delivery scheduled @{}\n",
                    queue.clock().now().value, delta.state.source.value(),
                    delta.state.source_sequence.value(), at.value);
            }
        }};

    network.add_endpoint(
        endpoint_b, [&](fleet::network::EndpointId from, const fleet::map::MapDelta& delta) {
            const std::uint64_t now = queue.clock().now().value;
            const bool route_hit = robot_b.current_route().uses_edge(delta.edge);
            const fleet::map::ReconcileDecision decision = robot_b.receive(delta);
            std::cout << std::format(
                "[{}][NETWORK] endpoint={} received delta (source={} seq={} edge={})\n", now,
                from.value(), delta.state.source.value(), delta.state.source_sequence.value(),
                edge_label(graph, delta.edge));
            std::cout << std::format("[{}][ROBOT_B] reconcile: decision={}\n", now,
                                     decision_name(decision));
            if (route_hit) {
                std::cout << std::format("[{}][ROBOT_B] route invalidated (uses {}): rerouted {}\n",
                                         now, edge_label(graph, delta.edge),
                                         format_route(robot_b.current_route(), graph));
            }
        });
    network.add_endpoint(
        endpoint_a, [&](fleet::network::EndpointId, const fleet::map::MapDelta&) {
            // No traffic toward A in this demo; registration keeps A a
            // reachable participant.
        });

    std::cout << "FleetSyncSim\n";
    std::cout << "milestone 1: deterministic reference simulator"
                 " (map + planning + reconciliation + network + robots)\n\n";
    std::cout << std::format("base map: version={} nodes={} edges={}\n",
                             demo.base.version().value(), graph.node_count(),
                             graph.edge_count());
    std::cout << "nodes:";
    for (const auto& node : graph.nodes()) {
        std::cout << ' ' << node.name;
    }
    std::cout << "\n\n";
    std::cout << std::format("mission A: A -> D\ninitial route A: {}\n",
                             format_route(robot_a.current_route(), graph));
    std::cout << std::format("mission B: I -> H\ninitial route B: {}\n\n",
                             format_route(robot_b.current_route(), graph));

    // t = 5000: the world changes edge F-G; robot A is the observer.
    // F-G is not on A's own route (A does not replan) but it is on B's.
    queue.run_until(Tick{5000});
    const EdgeId observed = edge_for(demo, "F", "G");
    const auto observation = robot_a.observe(observed, fleet::map::EdgeStatus::Blocked,
                                             Tick{5000}, 0.9);
    std::cout << std::format(
        "[5000][WORLD] edge F-G becomes BLOCKED\n"
        "[5000][ROBOT_A] observation: decision={} replanned={} (edge not on own route)\n",
        decision_name(observation.local_decision), observation.replanned ? "true" : "false");

    // Advance simulated time: B receives the delta at 5080 and reroutes.
    queue.run_until(Tick{6000});

    for (const fleet::robot::Robot* robot : {&robot_a, &robot_b}) {
        std::cout << std::format("\nrobot {} overlay: version={} tracked={}\n",
                                 robot->id().value(), robot->overlay().version().value(),
                                 robot->overlay().tracked_count());
        const fleet::map::MapView view{demo.base, robot->overlay()};
        for (const EdgeId edge : robot->overlay().tracked_edges()) {
            const fleet::map::EdgeDynamicState& state = *view.dynamic_state(edge);
            std::cout << std::format(
                "  edge={:>2} ({}) status={} observed_at_tick={} source={} sequence={} "
                "confidence={:.2}\n",
                edge.value(), edge_label(graph, edge), status_name(state.status),
                state.observed_at.value, state.source.value(), state.source_sequence.value(),
                state.confidence);
        }
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

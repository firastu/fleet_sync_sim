#include "fleet/robot/robot.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/map/map_reconciler.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/network/network_simulator.hpp"
#include "fleet/planning/route.hpp"
#include "fleet/simulation/event_queue.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::RobotId;
using fleet::common::SequenceNumber;
using fleet::common::Tick;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::map::MapDelta;
using fleet::map::ReconcileDecision;
using fleet::network::EndpointId;
using fleet::network::NetworkConfig;
using fleet::network::NetworkSimulator;
using fleet::planning::Route;
using fleet::robot::DeltaSink;
using fleet::robot::Mission;
using fleet::robot::Robot;
using fleet::simulation::EventQueue;
using fleet::testsupport::make_grid_map;

class RobotTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    std::vector<MapDelta> emitted_;

    [[nodiscard]] EdgeId edge_id(const char* a, const char* b) const {
        const std::optional<EdgeId> edge =
            grid_.base.graph().edge_between(grid_.node(a), grid_.node(b));
        EXPECT_TRUE(edge.has_value());
        return edge.value_or(EdgeId{});
    }

    [[nodiscard]] Robot make_robot_a_mission(char start, char goal) {
        return Robot{RobotId{1},
                     Mission{grid_.node(std::string{start}), grid_.node(std::string{goal})},
                     grid_.base,
                     [this](const MapDelta& delta) { emitted_.push_back(delta); }};
    }

    [[nodiscard]] static MapDelta remote_delta(RobotId source, SequenceNumber sequence,
                                               EdgeId edge, Tick at, EdgeStatus status) {
        return MapDelta{
            edge,
            EdgeDynamicState{
                .status = status,
                .observed_at = at,
                .source = source,
                .source_sequence = sequence,
                .confidence = 0.9,
            },
        };
    }
};

TEST_F(RobotTest, PlansInitialRouteFromMissionOnEmptyKnowledge) {
    const Robot robot = make_robot_a_mission('A', 'D');

    const Route& route = robot.current_route();
    EXPECT_TRUE(route.found);
    EXPECT_DOUBLE_EQ(route.cost, 3.0);
    EXPECT_EQ(robot.overlay().version(), fleet::common::OverlayVersion{0});
    EXPECT_TRUE(emitted_.empty());
}

TEST_F(RobotTest, ObservationAppliesEmitsAndAllocatesPerEdgeSequences) {
    Robot robot = make_robot_a_mission('A', 'D');

    const auto first = robot.observe(edge_id("F", "G"), EdgeStatus::Blocked, Tick{5000}, 0.9);
    EXPECT_EQ(first.local_decision, ReconcileDecision::Applied);
    EXPECT_EQ(first.delta.state.source, RobotId{1});
    EXPECT_EQ(first.delta.state.source_sequence, SequenceNumber{1});
    ASSERT_EQ(emitted_.size(), 1U);
    EXPECT_EQ(emitted_.front(), first.delta);
    ASSERT_NE(robot.overlay().find(edge_id("F", "G")), nullptr);
    EXPECT_EQ(robot.overlay().find(edge_id("F", "G"))->status, EdgeStatus::Blocked);

    // Same stream again: sequence advances.
    const auto second = robot.observe(edge_id("F", "G"), EdgeStatus::Open, Tick{6000});
    EXPECT_EQ(second.delta.state.source_sequence, SequenceNumber{2});

    // A different edge is an independent stream (identity is
    // (source, edge, sequence), ADR-004).
    const auto other = robot.observe(edge_id("K", "L"), EdgeStatus::Blocked, Tick{6000});
    EXPECT_EQ(other.delta.state.source_sequence, SequenceNumber{1});
}

TEST_F(RobotTest, ObservationOnRouteEdgeReplansAroundIt) {
    Robot robot = make_robot_a_mission('A', 'D');
    ASSERT_TRUE(robot.current_route().uses_edge(edge_id("B", "C")));

    const auto result = robot.observe(edge_id("B", "C"), EdgeStatus::Blocked, Tick{5000}, 0.9);

    EXPECT_TRUE(result.replanned);
    const Route& rerouted = robot.current_route();
    ASSERT_TRUE(rerouted.found);
    EXPECT_FALSE(rerouted.uses_edge(edge_id("B", "C")));
    EXPECT_DOUBLE_EQ(rerouted.cost, 5.0);  // canonical detour (ADR-003)
}

TEST_F(RobotTest, ObservationOffRouteDoesNotReplan) {
    Robot robot = make_robot_a_mission('A', 'D');
    const Route route_before = robot.current_route();

    const auto result = robot.observe(edge_id("K", "L"), EdgeStatus::Blocked, Tick{5000}, 0.9);

    EXPECT_FALSE(result.replanned);
    EXPECT_EQ(robot.current_route(), route_before);  // unchanged
}

TEST_F(RobotTest, LocallyDominatedObservationIsStillEmitted) {
    Robot robot = make_robot_a_mission('A', 'D');

    // Another source already delivered better knowledge for this edge.
    ASSERT_EQ(robot.receive(remote_delta(RobotId{2}, SequenceNumber{3}, edge_id("F", "G"),
                                         Tick{6000}, EdgeStatus::Open)),
              ReconcileDecision::Applied);

    // Robot 1's own observation is new in its stream but older than the
    // stored winner: Dominated locally — yet still emitted.
    const auto result = robot.observe(edge_id("F", "G"), EdgeStatus::Blocked, Tick{5000}, 0.9);
    EXPECT_EQ(result.local_decision, ReconcileDecision::Dominated);
    EXPECT_FALSE(result.replanned);
    ASSERT_EQ(emitted_.size(), 1U);
    ASSERT_NE(robot.overlay().find(edge_id("F", "G")), nullptr);
    EXPECT_EQ(robot.overlay().find(edge_id("F", "G"))->source, RobotId{2});  // winner unchanged
}

TEST_F(RobotTest, ReceiveAppliesAndReplansWhenRouteInvalidated) {
    Robot robot = make_robot_a_mission('A', 'D');

    const auto decision =
        robot.receive(remote_delta(RobotId{2}, SequenceNumber{1}, edge_id("B", "C"), Tick{5000},
                                   EdgeStatus::Blocked));

    EXPECT_EQ(decision, ReconcileDecision::Applied);
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_FALSE(robot.current_route().uses_edge(edge_id("B", "C")));
    EXPECT_DOUBLE_EQ(robot.current_route().cost, 5.0);
}

TEST_F(RobotTest, ReceiveDuplicateIsIdempotentForTheRobot) {
    Robot robot = make_robot_a_mission('A', 'D');
    const MapDelta delta =
        remote_delta(RobotId{2}, SequenceNumber{1}, edge_id("K", "L"), Tick{5000},
                     EdgeStatus::Blocked);
    ASSERT_EQ(robot.receive(delta), ReconcileDecision::Applied);
    const Route route_after_first = robot.current_route();

    EXPECT_EQ(robot.receive(delta), ReconcileDecision::Duplicate);
    EXPECT_EQ(robot.current_route(), route_after_first);
    EXPECT_EQ(robot.overlay().version(), fleet::common::OverlayVersion{1});
}

TEST_F(RobotTest, RobotOperatesWithoutASink) {
    Robot robot{RobotId{1},
                Mission{grid_.node("A"), grid_.node("D")},
                grid_.base,
                DeltaSink{}};

    const auto result = robot.observe(edge_id("B", "C"), EdgeStatus::Blocked, Tick{5000});
    EXPECT_EQ(result.local_decision, ReconcileDecision::Applied);
    ASSERT_TRUE(robot.current_route().found);
}

TEST_F(RobotTest, TwoRobotsConvergeThroughTransport) {
    // The keystone integration: two autonomous robots, one simulated
    // link, no manual coordination. A observes F-G at t=5000 (not on
    // A's own route); B's I->H route uses F-G and must reroute on
    // delivery at t=5080.
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}}, /*seed=*/1};
    const EndpointId endpoint_a{1};
    const EndpointId endpoint_b{2};

    Robot robot_a{RobotId{1},
                  Mission{grid_.node("A"), grid_.node("D")},
                  grid_.base,
                  [&](const MapDelta& delta) { (void)network.send(endpoint_a, endpoint_b, delta); }};
    Robot robot_b{RobotId{2},
                  Mission{grid_.node("I"), grid_.node("H")},
                  grid_.base,
                  [&](const MapDelta& delta) { (void)network.send(endpoint_b, endpoint_a, delta); }};

    network.add_endpoint(endpoint_b, [&robot_b](EndpointId, const MapDelta& delta) {
        (void)robot_b.receive(delta);
    });
    network.add_endpoint(endpoint_a, [&robot_a](EndpointId, const MapDelta& delta) {
        (void)robot_a.receive(delta);
    });

    // Canonical I->H route runs through F-G (ADR-003 tie-break).
    const EdgeId fg = edge_id("F", "G");
    ASSERT_TRUE(robot_b.current_route().uses_edge(fg));

    queue.run_until(Tick{5000});
    const auto observation = robot_a.observe(fg, EdgeStatus::Blocked, Tick{5000}, 0.9);
    EXPECT_EQ(observation.local_decision, ReconcileDecision::Applied);
    EXPECT_FALSE(observation.replanned);  // not on A's route
    EXPECT_TRUE(robot_a.current_route().uses_edge(edge_id("B", "C")));  // A's route untouched

    queue.run_to_completion();  // delivery at 5080 reaches robot B

    // B reconciled and rerouted without using F-G.
    ASSERT_TRUE(robot_b.current_route().found);
    EXPECT_FALSE(robot_b.current_route().uses_edge(fg));
    EXPECT_DOUBLE_EQ(robot_b.current_route().cost, 4.0);

    // Convergence: A and B hold identical knowledge about F-G.
    ASSERT_NE(robot_a.overlay().find(fg), nullptr);
    ASSERT_NE(robot_b.overlay().find(fg), nullptr);
    EXPECT_EQ(*robot_a.overlay().find(fg), *robot_b.overlay().find(fg));
}

TEST_F(RobotTest, ReopeningOffRouteEdgeRestoresShorterRoute) {
    // The routing bug this locks down: a re-opened edge is NOT on the
    // current detour, yet the robot must replan because the effective
    // traversal improved (best-known-route invariant).
    Robot robot = make_robot_a_mission('A', 'D');
    ASSERT_TRUE(robot.current_route().uses_edge(edge_id("B", "C")));

    const auto blocked = robot.observe(edge_id("B", "C"), EdgeStatus::Blocked, Tick{5000});
    ASSERT_TRUE(blocked.replanned);
    ASSERT_DOUBLE_EQ(robot.current_route().cost, 5.0);

    const auto reopened = robot.observe(edge_id("B", "C"), EdgeStatus::Open, Tick{6000});
    EXPECT_EQ(reopened.local_decision, ReconcileDecision::Applied);
    EXPECT_TRUE(reopened.replanned);  // B-C is off the current route, but better

    const Route& restored = robot.current_route();
    ASSERT_TRUE(restored.found);
    EXPECT_TRUE(restored.uses_edge(edge_id("B", "C")));
    EXPECT_DOUBLE_EQ(restored.cost, 3.0);
}

TEST_F(RobotTest, ProvenanceRefreshWithSameEffectiveStateDoesNotReplan) {
    Robot robot = make_robot_a_mission('A', 'D');
    ASSERT_EQ(robot.observe(edge_id("B", "C"), EdgeStatus::Blocked, Tick{5000}).local_decision,
              ReconcileDecision::Applied);
    const Route detour = robot.current_route();
    ASSERT_DOUBLE_EQ(detour.cost, 5.0);

    // Applied cross-source refresh: same BLOCKED status, newer tick —
    // knowledge provenance updates, effective traversal is unchanged.
    const auto decision = robot.receive(remote_delta(RobotId{2}, SequenceNumber{5},
                                                     edge_id("B", "C"), Tick{6000},
                                                     EdgeStatus::Blocked));
    EXPECT_EQ(decision, ReconcileDecision::Applied);
    EXPECT_EQ(robot.overlay().version(), fleet::common::OverlayVersion{2});  // refresh stored
    EXPECT_EQ(robot.current_route(), detour);  // but no unnecessary replan
}

TEST_F(RobotTest, RejectsObservationTimeRegressionWithoutConsumingSequence) {
    Robot robot = make_robot_a_mission('A', 'D');
    const EdgeId edge = edge_id("F", "G");  // off-route edge

    const auto first = robot.observe(edge, EdgeStatus::Blocked, Tick{5000});
    ASSERT_EQ(first.delta.state.source_sequence, SequenceNumber{1});
    ASSERT_EQ(emitted_.size(), 1U);
    const Route route_before = robot.current_route();
    const auto version_before = robot.overlay().version();

    // Tick regression within the same stream: rejected before any state
    // change (release-build behavior, not just an assert).
    EXPECT_THROW(robot.observe(edge, EdgeStatus::Open, Tick{4999}), std::invalid_argument);
    EXPECT_EQ(emitted_.size(), 1U);                              // sink not called
    EXPECT_EQ(robot.overlay().version(), version_before);        // overlay unchanged
    EXPECT_EQ(robot.current_route(), route_before);              // route unchanged

    // Same tick is legal (non-decreasing), and the rejected call did not
    // consume event identity: the next valid observation is seq 2.
    const auto second = robot.observe(edge, EdgeStatus::Open, Tick{5000});
    EXPECT_EQ(second.delta.state.source_sequence, SequenceNumber{2});
    EXPECT_EQ(emitted_.size(), 2U);
}

TEST_F(RobotTest, ThrowingSinkDoesNotRollBackCommittedObservation) {
    bool throw_on_emit = true;
    std::vector<MapDelta> emitted;
    Robot robot{
        RobotId{1},
        Mission{grid_.node("A"), grid_.node("D")},
        grid_.base,
        [&](const MapDelta& delta) {
            if (throw_on_emit) {
                throw std::runtime_error("sink failure");
            }
            emitted.push_back(delta);
        }};

    EXPECT_THROW(robot.observe(edge_id("B", "C"), EdgeStatus::Blocked, Tick{5000}),
                 std::runtime_error);

    // The observation stays committed: knowledge applied, sequence
    // consumed, route replanned around the blocked edge.
    ASSERT_NE(robot.overlay().find(edge_id("B", "C")), nullptr);
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_FALSE(robot.current_route().uses_edge(edge_id("B", "C")));

    // Identity was consumed by the throwing call, not rolled back: the
    // next observation on this stream is seq 2.
    throw_on_emit = false;
    const auto reopen = robot.observe(edge_id("B", "C"), EdgeStatus::Open, Tick{6000});
    EXPECT_EQ(reopen.delta.state.source_sequence, SequenceNumber{2});
    EXPECT_DOUBLE_EQ(robot.current_route().cost, 3.0);
    EXPECT_EQ(emitted.size(), 1U);
}

TEST_F(RobotTest, ResynchronizeEmitsCurrentKnowledgeWithOriginalIdentity) {
    Robot robot = make_robot_a_mission('A', 'D');
    const EdgeId x = edge_id("F", "G");
    const EdgeId y = edge_id("K", "L");
    const auto first = robot.observe(x, EdgeStatus::Blocked, Tick{5000});
    const auto second = robot.observe(y, EdgeStatus::Blocked, Tick{5100});
    ASSERT_EQ(emitted_.size(), 2U);
    emitted_.clear();

    // Re-announcement carries the ORIGINAL identity of each winner.
    ASSERT_EQ(robot.resynchronize(), 2U);
    ASSERT_EQ(emitted_.size(), 2U);
    EXPECT_EQ(emitted_.front(), first.delta);
    EXPECT_EQ(emitted_.back(), second.delta);

    // A receiver that already knows a fact suppresses it as Duplicate
    // (ADR-004 idempotence) — here the emitter itself.
    for (const MapDelta& reannouncement : emitted_) {
        EXPECT_EQ(robot.receive(reannouncement), ReconcileDecision::Duplicate);
    }
    EXPECT_EQ(robot.overlay().version(), fleet::common::OverlayVersion{2});  // unchanged
}

}  // namespace

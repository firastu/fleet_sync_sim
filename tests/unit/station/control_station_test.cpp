#include "fleet/station/control_station.hpp"

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
#include "fleet/robot/robot.hpp"
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
using fleet::robot::Mission;
using fleet::robot::Robot;
using fleet::simulation::EventQueue;
using fleet::station::ControlStation;
using fleet::testsupport::make_grid_map;

MapDelta blocked_delta(EdgeId edge, RobotId source, SequenceNumber sequence, Tick at) {
    return MapDelta{
        edge,
        EdgeDynamicState{
            .status = EdgeStatus::Blocked,
            .observed_at = at,
            .source = source,
            .source_sequence = sequence,
            .confidence = 0.9,
        },
    };
}

class ControlStationTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    ControlStation station_{grid_.base};

    [[nodiscard]] EdgeId edge_id(const char* a, const char* b) const {
        const std::optional<EdgeId> edge =
            grid_.base.graph().edge_between(grid_.node(a), grid_.node(b));
        EXPECT_TRUE(edge.has_value());
        return edge.value_or(EdgeId{});
    }
};

TEST_F(ControlStationTest, ReceivesAndStoresFleetKnowledge) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta delta =
        blocked_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000});

    EXPECT_EQ(station_.receive(delta), ReconcileDecision::Applied);
    ASSERT_NE(station_.overlay().find(edge), nullptr);
    EXPECT_EQ(station_.overlay().find(edge)->status, EdgeStatus::Blocked);
    EXPECT_EQ(station_.overlay().version(), fleet::common::OverlayVersion{1});

    // Re-announcement of the same fact is suppressed (ADR-004).
    EXPECT_EQ(station_.receive(delta), ReconcileDecision::Duplicate);
    EXPECT_EQ(station_.overlay().version(), fleet::common::OverlayVersion{1});
}

TEST_F(ControlStationTest, StartsEmpty) {
    EXPECT_EQ(station_.overlay().version(), fleet::common::OverlayVersion{0});
    EXPECT_EQ(station_.overlay().tracked_count(), 0U);
}

TEST_F(ControlStationTest, PartitionedStationConvergesOnReconnect) {
    // The #8 keystone: while the station is partitioned away, the fleet
    // keeps operating (robot-to-robot delta flows, B reroutes); on
    // reconnect, robots resynchronize and the station converges to the
    // fleet's knowledge purely through reconciliation.
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}}, /*seed=*/1};
    const EndpointId endpoint_a{1};
    const EndpointId endpoint_b{2};
    const EndpointId endpoint_station{3};

    ControlStation station{grid_.base};
    Robot robot_a{RobotId{1},
                  Mission{grid_.node("A"), grid_.node("D")},
                  grid_.base,
                  [&](const MapDelta& delta) {
                      (void)network.send(endpoint_a, endpoint_b, delta);
                      (void)network.send(endpoint_a, endpoint_station, delta);
                  }};
    Robot robot_b{RobotId{2},
                  Mission{grid_.node("I"), grid_.node("H")},
                  grid_.base,
                  [&](const MapDelta& delta) {
                      (void)network.send(endpoint_b, endpoint_a, delta);
                      (void)network.send(endpoint_b, endpoint_station, delta);
                  }};

    network.add_endpoint(endpoint_b, [&robot_b](EndpointId, const MapDelta& delta) {
        (void)robot_b.receive(delta);
    });
    network.add_endpoint(endpoint_a, [&robot_a](EndpointId, const MapDelta& delta) {
        (void)robot_a.receive(delta);
    });
    network.add_endpoint(
        endpoint_station, [&station](EndpointId, const MapDelta& delta) {
            (void)station.receive(delta);
        });

    // t=2000: the station is partitioned away (both directions per robot).
    queue.run_until(Tick{2000});
    for (const auto& [from, to] : {std::pair{endpoint_a, endpoint_station},
                                   std::pair{endpoint_b, endpoint_station},
                                   std::pair{endpoint_station, endpoint_a},
                                   std::pair{endpoint_station, endpoint_b}}) {
        network.set_link_state(from, to, false);
    }

    // t=5000: A observes F-G. Robot-to-robot link still works; the
    // station link does not. B reroutes; the station learns nothing.
    const EdgeId fg = edge_id("F", "G");
    queue.run_until(Tick{5000});
    ASSERT_TRUE(robot_b.current_route().uses_edge(fg));
    (void)robot_a.observe(fg, EdgeStatus::Blocked, Tick{5000}, 0.9);
    queue.run_to_completion();
    ASSERT_FALSE(robot_b.current_route().uses_edge(fg));  // fleet kept working
    ASSERT_EQ(station.overlay().tracked_count(), 0U);     // station isolated
    ASSERT_NE(robot_a.overlay().find(fg), nullptr);

    // t=8000: reconnect and resynchronize the station.
    queue.run_until(Tick{8000});
    for (const auto& [from, to] : {std::pair{endpoint_a, endpoint_station},
                                   std::pair{endpoint_b, endpoint_station},
                                   std::pair{endpoint_station, endpoint_a},
                                   std::pair{endpoint_station, endpoint_b}}) {
        network.set_link_state(from, to, true);
    }
    (void)robot_a.resynchronize();
    (void)robot_b.resynchronize();
    queue.run_to_completion();

    // Convergence: the station holds exactly the fleet's F-G knowledge.
    ASSERT_NE(station.overlay().find(fg), nullptr);
    EXPECT_EQ(*station.overlay().find(fg), *robot_a.overlay().find(fg));
    EXPECT_EQ(*station.overlay().find(fg), *robot_b.overlay().find(fg));
    EXPECT_EQ(station.overlay().tracked_count(), 1U);
}

}  // namespace

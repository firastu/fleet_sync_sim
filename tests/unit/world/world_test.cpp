#include "fleet/world/observation_model.hpp"
#include "fleet/world/world.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/robot/robot_state.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::NodeId;
using fleet::map::EdgeStatus;
using fleet::robot::RobotState;
using fleet::world::EdgeObservation;
using fleet::world::PerfectLocalEdgeSensor;
using fleet::world::World;
using fleet::testsupport::make_grid_map;

class WorldTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    const World world_{grid_.base};

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

TEST_F(WorldTest, InitialTruthMatchesBaseMap) {
    EXPECT_EQ(world_.edge_state(edge("A", "B")), EdgeStatus::Open);
    EXPECT_EQ(world_.edge_state(edge("C", "D")), EdgeStatus::Open);
}

TEST_F(WorldTest, TruthChangesAreQueryable) {
    World world{grid_.base};
    world.set_edge_state(edge("C", "D"), EdgeStatus::Blocked);
    EXPECT_EQ(world.edge_state(edge("C", "D")), EdgeStatus::Blocked);
    world.set_edge_state(edge("C", "D"), EdgeStatus::Open);
    EXPECT_EQ(world.edge_state(edge("C", "D")), EdgeStatus::Open);
}

TEST_F(WorldTest, RejectsUnknownEdge) {
    World world{grid_.base};
    EXPECT_THROW(static_cast<void>(world.set_edge_state(EdgeId{9999}, EdgeStatus::Blocked)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(world.edge_state(EdgeId{9999})), std::invalid_argument);
}

class PerfectLocalEdgeSensorTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    const PerfectLocalEdgeSensor sensor_;

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }

    [[nodiscard]] RobotState at(const char* node) const {
        RobotState state;
        state.position = grid_.node(node);
        return state;
    }
};

TEST_F(PerfectLocalEdgeSensorTest, MeasuresAllObservableEdgesAscending) {
    // Pure perception (ADR-011): the sensor reports the measured truth of
    // every observable edge — open ones included — without any knowledge
    // of belief. Suppression happens downstream.
    World world{grid_.base};
    world.set_edge_state(edge("C", "D"), EdgeStatus::Blocked);
    world.set_edge_state(edge("C", "G"), EdgeStatus::Blocked);

    const std::vector<EdgeObservation> observations = sensor_.sense(world, at("C"));

    ASSERT_EQ(observations.size(), 3U);  // B-C, C-D, C-G incident to C
    EXPECT_EQ(observations[0].edge, edge("B", "C"));
    EXPECT_EQ(observations[0].status, EdgeStatus::Open);
    EXPECT_EQ(observations[1].edge, edge("C", "D"));
    EXPECT_EQ(observations[1].status, EdgeStatus::Blocked);
    EXPECT_EQ(observations[2].edge, edge("C", "G"));
    EXPECT_EQ(observations[2].status, EdgeStatus::Blocked);
}

TEST_F(PerfectLocalEdgeSensorTest, RangeIsTheOccupiedNode) {
    World world{grid_.base};
    world.set_edge_state(edge("C", "D"), EdgeStatus::Blocked);

    // The change is not incident to A: a robot at A measures only A's
    // incident edges (A-B, A-E), all open.
    const std::vector<EdgeObservation> at_a = sensor_.sense(world, at("A"));
    ASSERT_EQ(at_a.size(), 2U);
    EXPECT_EQ(at_a[0].edge, edge("A", "B"));
    EXPECT_EQ(at_a[1].edge, edge("A", "E"));

    // A robot in transit FROM A has the same occupied node (the transit
    // edge is incident to it); the blocked A-E would be measured.
    world.set_edge_state(edge("A", "E"), EdgeStatus::Blocked);
    const RobotState in_transit{
        .position = grid_.node("A"),
        .in_transit = fleet::robot::RobotTransit{edge("A", "B"), grid_.node("A"),
                                                 grid_.node("B"), fleet::common::Tick{1000}}};
    const std::vector<EdgeObservation> observations =
        sensor_.sense(world, in_transit);
    ASSERT_EQ(observations.size(), 2U);
    EXPECT_EQ(observations[0].edge, edge("A", "B"));
    EXPECT_EQ(observations[0].status, EdgeStatus::Open);
    EXPECT_EQ(observations[1].edge, edge("A", "E"));
    EXPECT_EQ(observations[1].status, EdgeStatus::Blocked);
}

}  // namespace

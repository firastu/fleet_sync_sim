#include "fleet/robot/robot.hpp"
#include "fleet/robot/robot_state.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <stdexcept>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/robot/mission.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::EdgeStatus;
using fleet::robot::Mission;
using fleet::robot::Robot;
using fleet::robot::RobotTransit;
using fleet::testsupport::make_grid_map;

class RobotMovementTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] Robot make_robot(const char* start, const char* goal) const {
        return Robot{RobotId{1},
                     Mission{grid_.node(start), grid_.node(goal)},
                     grid_.base,
                     [](const fleet::map::MapDelta&) {}};
    }

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

TEST_F(RobotMovementTest, InitialStatePositionsRobotAtMissionStart) {
    const Robot robot = make_robot("A", "D");
    EXPECT_EQ(robot.state().position, grid_.node("A"));
    EXPECT_FALSE(robot.state().in_transit.has_value());
    EXPECT_FALSE(robot.state().mission_complete);
}

TEST_F(RobotMovementTest, StartEqualsGoalMissionImmediatelyComplete) {
    Robot robot = make_robot("A", "A");
    EXPECT_TRUE(robot.state().mission_complete);
    EXPECT_EQ(robot.state().position, grid_.node("A"));
    // Nothing to traverse.
    EXPECT_FALSE(robot.begin_transit(Tick{0}, 1000).has_value());
}

TEST_F(RobotMovementTest, BeginTransitReturnsFirstRouteEdge) {
    Robot robot = make_robot("A", "D");
    const std::optional<RobotTransit> transit = robot.begin_transit(Tick{0}, 1000);
    ASSERT_TRUE(transit.has_value());
    EXPECT_EQ(transit->edge, edge("A", "B"));
    EXPECT_EQ(transit->from, grid_.node("A"));
    EXPECT_EQ(transit->to, grid_.node("B"));
    EXPECT_EQ(transit->arrival, Tick{1000});
    ASSERT_TRUE(robot.state().in_transit.has_value());
    // Position stays the departure node until the arrival commits.
    EXPECT_EQ(robot.state().position, grid_.node("A"));
    // Only one traversal at a time.
    EXPECT_FALSE(robot.begin_transit(Tick{0}, 1000).has_value());
}

TEST_F(RobotMovementTest, TraversalTimeScalesWithSpeedAndClock) {
    Robot robot = make_robot("A", "B");
    const std::optional<RobotTransit> half_speed = robot.begin_transit(Tick{100}, 500);
    ASSERT_TRUE(half_speed.has_value());
    EXPECT_EQ(half_speed->arrival, Tick{600});
}

TEST_F(RobotMovementTest, BeginTransitRequiresUsableRoute) {
    Robot robot = make_robot("A", "D");
    robot.observe(edge("A", "B"), EdgeStatus::Blocked, Tick{100});
    robot.observe(edge("A", "E"), EdgeStatus::Blocked, Tick{100});
    ASSERT_FALSE(robot.current_route().found);  // A is isolated
    EXPECT_FALSE(robot.begin_transit(Tick{0}, 1000).has_value());
    EXPECT_FALSE(robot.state().in_transit.has_value());
}

TEST_F(RobotMovementTest, CompleteTransitCommitsPositionAndContinues) {
    Robot robot = make_robot("A", "D");
    ASSERT_TRUE(robot.begin_transit(Tick{0}, 1000).has_value());
    EXPECT_FALSE(robot.complete_transit());
    EXPECT_EQ(robot.state().position, grid_.node("B"));
    EXPECT_FALSE(robot.state().in_transit.has_value());
    EXPECT_FALSE(robot.state().mission_complete);
    // Route now plans from B.
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_EQ(robot.current_route().nodes.front(), grid_.node("B"));
    EXPECT_DOUBLE_EQ(robot.current_route().cost, 2.0);
}

TEST_F(RobotMovementTest, CompleteTransitAtGoalCompletesMission) {
    Robot robot = make_robot("A", "B");
    ASSERT_TRUE(robot.begin_transit(Tick{0}, 1000).has_value());
    EXPECT_TRUE(robot.complete_transit());
    EXPECT_EQ(robot.state().position, grid_.node("B"));
    EXPECT_TRUE(robot.state().mission_complete);
    EXPECT_FALSE(robot.begin_transit(Tick{1000}, 1000).has_value());
}

TEST_F(RobotMovementTest, CompleteTransitThrowsWhenNotInTransit) {
    Robot robot = make_robot("A", "D");
    EXPECT_THROW(static_cast<void>(robot.complete_transit()), std::runtime_error);
}

TEST_F(RobotMovementTest, ReplansFromCurrentPositionAfterMoving) {
    Robot robot = make_robot("A", "D");
    ASSERT_TRUE(robot.begin_transit(Tick{0}, 1000).has_value());
    ASSERT_FALSE(robot.complete_transit());  // at B

    robot.observe(edge("C", "D"), EdgeStatus::Blocked, Tick{2000});
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_EQ(robot.current_route().nodes.front(), grid_.node("B"));  // from position
    EXPECT_FALSE(robot.current_route().uses_edge(edge("C", "D")));
    EXPECT_DOUBLE_EQ(robot.current_route().cost, 4.0);  // B -> F -> G -> (C|H) -> D
}

TEST_F(RobotMovementTest, ReplanWhileInTransitStartsFromDestination) {
    Robot robot = make_robot("A", "D");
    ASSERT_TRUE(robot.begin_transit(Tick{0}, 1000).has_value());  // A -> B

    // Knowledge changes mid-transit: the new route must start at B (the
    // committed traversal's destination), never at A.
    robot.observe(edge("B", "C"), EdgeStatus::Blocked, Tick{500});
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_EQ(robot.current_route().nodes.front(), grid_.node("B"));
    EXPECT_FALSE(robot.current_route().uses_edge(edge("B", "C")));
    EXPECT_DOUBLE_EQ(robot.current_route().cost, 4.0);
    EXPECT_EQ(robot.state().position, grid_.node("A"));  // still physically on A-B
}

TEST_F(RobotMovementTest, CommittedTransitSurvivesBlockedKnowledge) {
    Robot robot = make_robot("A", "D");
    ASSERT_TRUE(robot.begin_transit(Tick{0}, 1000).has_value());  // on A-B

    // The robot learns its own edge is blocked while on it: the traversal
    // is physically committed and still completes.
    robot.observe(edge("A", "B"), EdgeStatus::Blocked, Tick{500});
    EXPECT_FALSE(robot.complete_transit());
    EXPECT_EQ(robot.state().position, grid_.node("B"));
    EXPECT_FALSE(robot.state().mission_complete);

    // And it can continue from B on its knowledge.
    const std::optional<RobotTransit> next = robot.begin_transit(Tick{1000}, 1000);
    ASSERT_TRUE(next.has_value());
    EXPECT_EQ(next->from, grid_.node("B"));
}

}  // namespace

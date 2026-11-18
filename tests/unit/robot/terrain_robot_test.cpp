#include "fleet/robot/terrain_robot.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "fleet/map/elevation_grid.hpp"

namespace {

using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::localization::TerrainSensorFrame;
using fleet::map::ElevationGrid;
using fleet::map::ElevationGridLayout;
using fleet::map::ElevationSurface;
using fleet::map::VerticalDatum;
using fleet::map::Wgs84Coordinate;
using fleet::robot::TerrainMission;
using fleet::robot::TerrainRobot;

[[nodiscard]] std::shared_ptr<const ElevationGrid> flat_map() {
    return std::make_shared<const ElevationGrid>(
        ElevationGridLayout{Wgs84Coordinate{1.0, 0.0}, Wgs84Coordinate{0.0, 1.0}, 2, 2},
        std::vector<float>(4, 10.0F), ElevationSurface::BareEarth, VerticalDatum::Synthetic);
}

[[nodiscard]] TerrainSensorFrame frame(std::uint64_t from, std::uint64_t at) {
    TerrainSensorFrame result;
    result.from = Tick{from};
    result.at = Tick{at};
    result.baro_altitude_m = 12.5;
    return result;
}

TEST(TerrainRobotTest, HoldsItsMissionAndItsOwnMap) {
    const auto map = flat_map();
    const TerrainRobot robot{RobotId{3}, TerrainMission{{{0.5, 0.5}}}, map};
    EXPECT_EQ(robot.id(), RobotId{3});
    EXPECT_EQ(robot.mission().waypoints.size(), 1U);
    EXPECT_EQ(&robot.terrain_map(), map.get());
    EXPECT_FALSE(robot.last_frame().has_value());
    EXPECT_EQ(robot.frames_received(), 0U);

    EXPECT_THROW(TerrainRobot(RobotId{1}, TerrainMission{{{0.5, 0.5}}}, nullptr),
                 std::invalid_argument);
    EXPECT_THROW(TerrainRobot(RobotId{1}, TerrainMission{}, map), std::invalid_argument);
    EXPECT_THROW(TerrainRobot(RobotId{1}, TerrainMission{{{95.0, 0.5}}}, map),
                 std::invalid_argument);
}

TEST(TerrainRobotTest, ReceivesFramesInTimeOrder) {
    TerrainRobot robot{RobotId{1}, TerrainMission{{{0.5, 0.5}}}, flat_map()};
    robot.receive(frame(0, 1000));
    robot.receive(frame(1000, 2000));
    EXPECT_EQ(robot.frames_received(), 2U);
    ASSERT_TRUE(robot.last_frame().has_value());
    EXPECT_EQ(robot.last_frame()->at, Tick{2000});
    EXPECT_EQ(robot.last_frame()->baro_altitude_m, 12.5);

    EXPECT_THROW(robot.receive(frame(1500, 2500)), std::invalid_argument);  // overlaps
    EXPECT_THROW(robot.receive(frame(3000, 3000)), std::invalid_argument);  // empty interval
    EXPECT_EQ(robot.frames_received(), 2U);
    EXPECT_EQ(robot.last_frame()->at, Tick{2000});
}

}  // namespace

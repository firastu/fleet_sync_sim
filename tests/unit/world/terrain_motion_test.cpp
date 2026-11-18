#include "fleet/world/terrain_motion.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet/dem/synthetic_terrain.hpp"
#include "fleet/map/elevation_grid.hpp"

namespace {

using fleet::common::Tick;
using fleet::dem::SyntheticHill;
using fleet::dem::SyntheticTerrainSpec;
using fleet::dem::synthesize_terrain;
using fleet::map::ElevationGrid;
using fleet::map::ElevationGridLayout;
using fleet::map::ElevationSurface;
using fleet::map::VerticalDatum;
using fleet::map::Wgs84Coordinate;
using fleet::world::TerrainMotion;
using fleet::world::TerrainMotionConfig;
using fleet::world::TerrainPlatform;
using fleet::world::TerrainStepOutcome;

constexpr double kPi = std::numbers::pi;

// About 22 km x 15 km around (46.0, 7.0).
constexpr ElevationGridLayout kArea{Wgs84Coordinate{46.1, 6.9}, Wgs84Coordinate{45.9, 7.1}, 41,
                                    41};

[[nodiscard]] std::shared_ptr<const ElevationGrid> terrain(std::vector<SyntheticHill> hills = {},
                                                           double base = 100.0) {
    return std::make_shared<const ElevationGrid>(synthesize_terrain(
        SyntheticTerrainSpec{kArea, base, std::move(hills), ElevationSurface::BareEarth}));
}

// A tilted plane: rises 300 m from the west edge to the east edge.
[[nodiscard]] std::shared_ptr<const ElevationGrid> slope() {
    std::vector<float> heights;
    for (std::size_t row = 0; row < 2; ++row) {
        heights.push_back(100.0F);
        heights.push_back(400.0F);
    }
    return std::make_shared<const ElevationGrid>(
        ElevationGridLayout{Wgs84Coordinate{46.1, 6.9}, Wgs84Coordinate{45.9, 7.1}, 2, 2},
        std::move(heights), ElevationSurface::BareEarth, VerticalDatum::Synthetic);
}

[[nodiscard]] TerrainMotionConfig ground(double speed = 10.0) {
    TerrainMotionConfig config;
    config.platform = TerrainPlatform::Ground;
    config.speed_mps = speed;
    config.max_turn_rate_rad_per_s = 0.5;
    config.arrival_radius_m = 50.0;
    config.mount_height_m = 1.5;
    return config;
}

[[nodiscard]] TerrainMotionConfig air(double altitude) {
    TerrainMotionConfig config;
    config.platform = TerrainPlatform::Air;
    config.speed_mps = 10.0;
    config.max_turn_rate_rad_per_s = 0.5;
    config.arrival_radius_m = 50.0;
    config.altitude_m = altitude;
    return config;
}

[[nodiscard]] double haversine_m(Wgs84Coordinate a, Wgs84Coordinate b) {
    const double rad = kPi / 180.0;
    const double h = std::pow(std::sin((b.latitude_deg - a.latitude_deg) * rad / 2.0), 2) +
                     std::cos(a.latitude_deg * rad) * std::cos(b.latitude_deg * rad) *
                         std::pow(std::sin((b.longitude_deg - a.longitude_deg) * rad / 2.0), 2);
    return 2.0 * 6371008.8 * std::asin(std::sqrt(h));
}

constexpr Wgs84Coordinate kStart{46.0, 7.0};

TEST(TerrainMotionConfigTest, RejectsInvalidLimits) {
    const auto rejects = [](TerrainMotionConfig config) {
        EXPECT_THROW(config.validate(), std::invalid_argument);
    };
    TerrainMotionConfig config = ground();
    config.speed_mps = 0.0;
    rejects(config);
    config = ground();
    config.max_turn_rate_rad_per_s = -1.0;
    rejects(config);
    config = ground();
    config.arrival_radius_m = 0.0;
    rejects(config);
    config = ground();
    config.arrival_radius_m = config.turning_diameter_m() - 0.001;  // 40 m diameter
    rejects(config);
    config = ground();
    config.mount_height_m = -0.1;
    rejects(config);
    config = ground();
    config.altitude_m = 500.0;  // ground bodies follow the terrain
    rejects(config);
    config = air(500.0);
    config.mount_height_m = 1.0;  // air bodies fly at altitude_m
    rejects(config);
    config = ground();
    config.speed_mps = std::numeric_limits<double>::quiet_NaN();
    rejects(config);
    EXPECT_NO_THROW(ground().validate());
    EXPECT_NO_THROW(air(500.0).validate());
}

TEST(TerrainMotionTest, ConstructionRejectsInvalidInputs) {
    const auto shared = terrain();
    const std::vector<Wgs84Coordinate> waypoints{{46.0, 7.05}};
    EXPECT_THROW(TerrainMotion(ground(), waypoints, nullptr, kStart, 0.0, Tick{0}),
                 std::invalid_argument);
    EXPECT_THROW(TerrainMotion(ground(), {}, shared, kStart, 0.0, Tick{0}), std::invalid_argument);
    EXPECT_THROW(TerrainMotion(ground(), {{91.0, 7.0}}, shared, kStart, 0.0, Tick{0}),
                 std::invalid_argument);
    EXPECT_THROW(TerrainMotion(ground(), waypoints, shared, {47.0, 7.0}, 0.0, Tick{0}),
                 std::invalid_argument);  // start outside the truth terrain
    EXPECT_THROW(TerrainMotion(air(90.0), waypoints, shared, kStart, 0.0, Tick{0}),
                 std::invalid_argument);  // air start below the 100 m terrain
    EXPECT_THROW(TerrainMotion(ground(), waypoints, shared, kStart,
                               std::numeric_limits<double>::infinity(), Tick{0}),
                 std::invalid_argument);
}

TEST(TerrainMotionTest, StepValidationRejectsBadSteps) {
    TerrainMotion motion{ground(), {{46.0, 7.05}}, terrain(), kStart, kPi / 2.0, Tick{0}};
    EXPECT_THROW((void)motion.step(0), std::invalid_argument);
    EXPECT_THROW((void)motion.step(6000), std::invalid_argument);  // 60 m > 50 m arrival radius

    TerrainMotionConfig fast = ground(60.0);
    fast.max_turn_rate_rad_per_s = 2.0;
    fast.arrival_radius_m = 100.0;
    EXPECT_THROW(TerrainMotion::validate_step(fast, 1000), std::invalid_argument);  // 60 m > 50 m
    EXPECT_NO_THROW(TerrainMotion::validate_step(fast, 500));
}

TEST(TerrainMotionTest, MovesAtConstantSpeedTowardTheWaypoint) {
    TerrainMotion motion{ground(), {{46.0, 7.05}}, terrain(), kStart, kPi / 2.0, Tick{0}};
    const Wgs84Coordinate before = motion.state().position;
    const TerrainStepOutcome outcome = motion.step(1000);
    EXPECT_TRUE(outcome.reached_waypoints.empty());
    EXPECT_FALSE(outcome.mission_completed);
    EXPECT_EQ(motion.state().at, Tick{1000});
    EXPECT_NEAR(haversine_m(before, motion.state().position), 10.0, 1e-6);
    EXPECT_GT(motion.state().position.longitude_deg, before.longitude_deg);
    EXPECT_NEAR(motion.state().heading_rad, kPi / 2.0, 1e-3);
}

TEST(TerrainMotionTest, StepOutcomeReportsTheTrueIncrement) {
    TerrainMotion motion{ground(), {{45.95, 7.0}}, terrain(), kStart, 0.0, Tick{0}};
    const TerrainStepOutcome turning = motion.step(1000);
    EXPECT_EQ(turning.distance_m, 10.0);
    EXPECT_EQ(turning.turn_rad, 0.5);  // the full clockwise budget

    TerrainMotion parked{ground(), {{46.0001, 7.0}}, terrain(), kStart, 0.0, Tick{0}};
    const TerrainStepOutcome arrived = parked.step(1000);  // reached before moving
    EXPECT_TRUE(arrived.mission_completed);
    EXPECT_EQ(arrived.distance_m, 0.0);
    EXPECT_EQ(arrived.turn_rad, 0.0);
}

TEST(TerrainMotionTest, TurnRateIsBoundedAndExactlyBehindTurnsClockwise) {
    // Heading north with the waypoint due south: a half turn at 0.5 rad/s.
    TerrainMotion motion{ground(), {{45.95, 7.0}}, terrain(), kStart, 0.0, Tick{0}};
    (void)motion.step(1000);
    EXPECT_DOUBLE_EQ(motion.state().heading_rad, 0.5);  // clockwise, by the full budget
    double previous = motion.state().heading_rad;
    for (int step = 0; step < 10; ++step) {
        (void)motion.step(1000);
        const double heading = motion.state().heading_rad;
        EXPECT_LE(std::abs(heading - previous), 0.5 + 1e-12);
        previous = heading;
    }
    EXPECT_NEAR(motion.state().heading_rad, kPi, 0.05);  // now heading (almost) south
}

TEST(TerrainMotionTest, ReachesWaypointsInOrderThenStays) {
    const std::vector<Wgs84Coordinate> square{{46.0, 7.01}, {46.008, 7.01}, {46.008, 7.0}};
    TerrainMotion motion{ground(), square, terrain(), kStart, kPi / 2.0, Tick{0}};
    std::vector<std::size_t> reached;
    bool completed = false;
    for (int step = 0; step < 1000 && !motion.mission_complete(); ++step) {
        const TerrainStepOutcome outcome = motion.step(1000);
        reached.insert(reached.end(), outcome.reached_waypoints.begin(),
                       outcome.reached_waypoints.end());
        completed = completed || outcome.mission_completed;
    }
    ASSERT_TRUE(motion.mission_complete());
    EXPECT_TRUE(completed);
    EXPECT_EQ(reached, (std::vector<std::size_t>{0, 1, 2}));
    EXPECT_EQ(motion.state().next_waypoint, 3U);

    const Wgs84Coordinate parked = motion.state().position;
    const Tick parked_at = motion.state().at;
    const TerrainStepOutcome idle = motion.step(1000);
    EXPECT_TRUE(idle.reached_waypoints.empty());
    EXPECT_FALSE(idle.mission_completed);
    EXPECT_EQ(motion.state().position, parked);
    EXPECT_EQ(motion.state().at, parked_at + 1000);
}

TEST(TerrainMotionTest, ConsumesSeveralCloseWaypointsInOneStep) {
    // Both waypoints are already inside the 50 m arrival radius.
    TerrainMotion motion{ground(), {{46.0001, 7.0}, {46.0002, 7.0}}, terrain(), kStart, 0.0,
                         Tick{0}};
    const TerrainStepOutcome outcome = motion.step(1000);
    EXPECT_EQ(outcome.reached_waypoints, (std::vector<std::size_t>{0, 1}));
    EXPECT_TRUE(outcome.mission_completed);
    EXPECT_EQ(motion.state().position, kStart);  // reached before moving
}

TEST(TerrainMotionTest, GroundBodyFollowsTheTerrainPlusMountHeight) {
    const auto plane = slope();
    TerrainMotion motion{ground(), {{46.0, 7.08}}, plane, kStart, kPi / 2.0, Tick{0}};
    double previous = motion.state().terrain_height_m;
    for (int step = 0; step < 50; ++step) {
        (void)motion.step(1000);
        const auto& state = motion.state();
        EXPECT_EQ(state.terrain_height_m, plane->height_at(state.position).height_m.value());
        EXPECT_EQ(state.altitude_m, state.terrain_height_m + 1.5);
        EXPECT_GT(state.terrain_height_m, previous);  // driving east, uphill
        previous = state.terrain_height_m;
    }
}

TEST(TerrainMotionTest, AirBodyKeepsItsAltitudeOverChangingTerrain) {
    const auto plane = slope();
    TerrainMotion motion{air(1000.0), {{46.0, 7.08}}, plane, kStart, kPi / 2.0, Tick{0}};
    for (int step = 0; step < 50; ++step) {
        (void)motion.step(1000);
        EXPECT_EQ(motion.state().altitude_m, 1000.0);
        EXPECT_EQ(motion.state().terrain_height_m,
                  plane->height_at(motion.state().position).height_m.value());
    }
}

TEST(TerrainMotionTest, LeavingTheTruthTerrainFailsWithoutChangingState) {
    // Waypoint beyond the east edge (7.1): the body must fail at the edge.
    TerrainMotionConfig config = ground(40.0);
    config.max_turn_rate_rad_per_s = 1.0;
    config.arrival_radius_m = 80.0;
    TerrainMotion motion{config, {{46.0, 7.2}}, terrain(), {46.0, 7.099}, kPi / 2.0, Tick{0}};
    std::string message;
    for (int step = 0; step < 10; ++step) {
        const auto before = motion.state();
        try {
            (void)motion.step(1000);
        } catch (const std::runtime_error& error) {
            message = error.what();
            EXPECT_EQ(motion.state().position, before.position);
            EXPECT_EQ(motion.state().at, before.at);
            break;
        }
    }
    EXPECT_NE(message.find("outside the truth terrain coverage"), std::string::npos) << message;
}

TEST(TerrainMotionTest, AirBodyFlyingIntoTerrainFails) {
    // A 500 m hill 3 km east of the start; flight altitude 300 m.
    const auto hilly = terrain({{Wgs84Coordinate{46.0, 7.04}, 500.0, 1500.0}});
    TerrainMotion motion{air(300.0), {{46.0, 7.08}}, hilly, kStart, kPi / 2.0, Tick{0}};
    std::string message;
    for (int step = 0; step < 1000 && message.empty(); ++step) {
        try {
            (void)motion.step(1000);
        } catch (const std::runtime_error& error) {
            message = error.what();
        }
    }
    EXPECT_NE(message.find("is not above the truth terrain"), std::string::npos) << message;
}

TEST(TerrainMotionTest, IdenticalInputsReplayBitIdentically) {
    const auto hilly = terrain({{Wgs84Coordinate{46.02, 7.03}, 200.0, 2000.0}});
    const std::vector<Wgs84Coordinate> route{{46.03, 7.02}, {46.01, 7.05}, {45.99, 7.01}};
    TerrainMotion first{ground(), route, hilly, kStart, 1.0, Tick{0}};
    TerrainMotion second{ground(), route, hilly, kStart, 1.0, Tick{0}};
    for (int step = 0; step < 600; ++step) {
        (void)first.step(1000);
        (void)second.step(1000);
        ASSERT_EQ(first.state().position, second.state().position);
        ASSERT_EQ(first.state().heading_rad, second.state().heading_rad);
        ASSERT_EQ(first.state().altitude_m, second.state().altitude_m);
        ASSERT_EQ(first.state().next_waypoint, second.state().next_waypoint);
    }
}

}  // namespace

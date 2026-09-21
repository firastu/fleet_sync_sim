#include "fleet/world/truth_pose.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/localization/pose.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/robot/robot_state.hpp"
#include "fleet/robot/robot.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::Tick;
using fleet::localization::GroundTruthPose;
using fleet::map::BaseMap;
using fleet::map::Graph;
using fleet::map::MapGeometry;
using fleet::map::NodePosition;
using fleet::map::Wgs84Coordinate;
using fleet::robot::RobotState;
using fleet::robot::RobotTransit;
using fleet::world::truth_pose;

constexpr double kPi = 3.14159265358979323846;

// A two-node map with an L-shaped canonical polyline (a->b):
//
//   X --(due east, 0.002 deg lon, ~137 m)--> V --(due north, 0.001 deg
//   lat, ~111 m)--> Y
//
// The legs have DIFFERENT physical lengths, so a test landing mid-arc
// discriminates arc-length interpolation from point-index interpolation.
struct BentMap {
    BaseMap base;
    fleet::common::NodeId x{};
    fleet::common::NodeId y{};
    fleet::common::EdgeId edge{};
};

[[nodiscard]] BentMap make_bent_map() {
    Graph::Builder builder;
    const fleet::common::NodeId x = builder.add_node("X", NodePosition{0, 0});
    const fleet::common::NodeId y = builder.add_node("Y", NodePosition{1, 0});
    builder.connect(x, y);
    const Graph graph = builder.build();
    const fleet::common::EdgeId edge = graph.edge_between(x, y).value();

    MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    geometry.set_node_position(x, Wgs84Coordinate{52.0000, 10.0000});
    geometry.set_node_position(y, Wgs84Coordinate{52.0010, 10.0020});
    // Canonical a->b orientation; endpoints REUSE the stored node
    // coordinate values exactly (the BaseMap identity contract).
    geometry.set_edge_polyline(edge,
                               std::vector<Wgs84Coordinate>{
                                   Wgs84Coordinate{52.0000, 10.0000},
                                   Wgs84Coordinate{52.0000, 10.0020},
                                   Wgs84Coordinate{52.0010, 10.0020}});
    return BentMap{BaseMap{graph, fleet::common::MapVersion{1}, geometry.build()}, x, y, edge};
}

[[nodiscard]] RobotState at_node(fleet::common::NodeId node) {
    return RobotState{.position = node,
                      .in_transit = std::nullopt,
                      .mission_complete = false};
}

[[nodiscard]] RobotState in_transit(fleet::common::EdgeId edge, fleet::common::NodeId from,
                                    fleet::common::NodeId to, std::uint64_t departed_at,
                                    std::uint64_t arrival) {
    return RobotState{.position = from,
                      .in_transit =
                          RobotTransit{edge, from, to, Tick{departed_at}, Tick{arrival}},
                      .mission_complete = false};
}

class TruthPoseTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{fleet::testsupport::make_grid_map_with_geometry()};
    const BentMap bent_{make_bent_map()};

    [[nodiscard]] fleet::common::EdgeId grid_edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

TEST_F(TruthPoseTest, AtRestHeadingIsNormalized) {
    const GroundTruthPose pose =
        truth_pose(
            grid_.base,
            at_node(grid_.node("C")),
            5.0 * kPi / 2.0,
            Tick{1200});

    EXPECT_NEAR(pose.heading_rad, kPi / 2.0, 1e-15);
}

TEST_F(TruthPoseTest, NonFiniteAtRestHeadingFails) {
    EXPECT_THROW(
        (void)truth_pose(
            grid_.base,
            at_node(grid_.node("C")),
            std::numeric_limits<double>::quiet_NaN(),
            Tick{0}),
        std::invalid_argument);
}

TEST_F(TruthPoseTest, AtNodeUsesCanonicalCoordinateAndSuppliedPhysicalHeading) {
    constexpr double kAtRestHeading = 1.25;

    const GroundTruthPose pose =
        truth_pose(
            grid_.base,
            at_node(grid_.node("C")),
            kAtRestHeading,
            Tick{1200});

    EXPECT_EQ(pose.position, (Wgs84Coordinate{52.370, 9.734}));
    EXPECT_DOUBLE_EQ(pose.heading_rad, kAtRestHeading);
    EXPECT_EQ(pose.at, Tick{1200});
}

TEST_F(TruthPoseTest, NodeWithoutGeometryFailsExplicitly) {
    // Presence is per node (ADR-012): a map where B lacks coordinates is
    // legal — but truth at B cannot be manufactured.
    Graph::Builder builder;
    const auto a = builder.add_node("A", NodePosition{0, 0});
    const auto b = builder.add_node("B", NodePosition{1, 0});
    builder.connect(a, b);
    const Graph graph = builder.build();
    MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    geometry.set_node_position(a, Wgs84Coordinate{52.0, 10.0});
    const BaseMap base{graph, fleet::common::MapVersion{1}, geometry.build()};
    EXPECT_THROW((void)truth_pose(base, at_node(b), 0.0, Tick{0}), std::invalid_argument);
}

TEST_F(TruthPoseTest, MapWithoutGeometryFailsExplicitly) {
    const fleet::testsupport::GridMap bare = fleet::testsupport::make_grid_map();
    const auto edge = bare.base.graph().edge_between(bare.node("A"), bare.node("B")).value();
    EXPECT_THROW((void)truth_pose(bare.base,
                                  in_transit(edge, bare.node("A"), bare.node("B"), 0, 1000),
                                  0.0,
                                  Tick{500}),
                 std::invalid_argument);
}

TEST_F(TruthPoseTest, DepartureAndArrivalLandExactlyOnEndpoints) {
    const auto edge = grid_edge("A", "B");
    const auto canonical = [this](const char* node) {
        return *grid_.base.geometry()->node_position(grid_.node(node));
    };
    const GroundTruthPose start = truth_pose(
        grid_.base, in_transit(edge, grid_.node("A"), grid_.node("B"), 0, 1000), 0.0, Tick{0});
    EXPECT_EQ(start.position, canonical("A"));  // bit-exact canonical coordinate
    const GroundTruthPose end = truth_pose(
        grid_.base, in_transit(edge, grid_.node("A"), grid_.node("B"), 0, 1000), 0.0, Tick{1000});
    EXPECT_EQ(end.position, canonical("B"));  // bit-exact canonical coordinate
}

TEST_F(TruthPoseTest, MidpointOfStraightEdgeIsHalfway) {
    const auto edge = grid_edge("A", "B");
    const GroundTruthPose pose = truth_pose(
        grid_.base, in_transit(edge, grid_.node("A"), grid_.node("B"), 0, 1000), 0.0, Tick{500});
    // A and B share a latitude: an east-west traversal keeps it exactly.
    EXPECT_DOUBLE_EQ(pose.position.latitude_deg, 52.370);
    EXPECT_NEAR(pose.position.longitude_deg, 9.731, 1e-12);
}

TEST_F(TruthPoseTest, ForwardTraversalHeadingIsTheSegmentBearing) {
    const auto edge = grid_edge("A", "B");  // due east in canonical geometry
    const GroundTruthPose pose = truth_pose(
        grid_.base, in_transit(edge, grid_.node("A"), grid_.node("B"), 0, 1000), 0.0, Tick{500});
    // The bearing is the GREAT-CIRCLE initial bearing between the
    // segment endpoints: for equal-latitude points it is microscopically
    // north of east (the geodesic bulges poleward; the parallel is a
    // rhumb line). ~1.4e-5 rad here — far below any physical relevance.
    EXPECT_NEAR(pose.heading_rad, kPi / 2.0, 1e-4);
}

TEST_F(TruthPoseTest, ReverseTraversalMirrorsGeometryAndHeadingsWest) {
    // The same Bidirectional edge traversed B->A: same canonical
    // geometry, opposite physical motion (ADR-013).
    const auto edge = grid_edge("A", "B");
    const GroundTruthPose pose = truth_pose(
        grid_.base, in_transit(edge, grid_.node("B"), grid_.node("A"), 0, 1000), 0.0, Tick{500});
    EXPECT_DOUBLE_EQ(pose.position.latitude_deg, 52.370);
    EXPECT_NEAR(pose.position.longitude_deg, 9.731, 1e-12);
    // Due west: 3*pi/2 clockwise from north (the mirror of the ~1.4e-5
    // geodesic bulge applies here too).
    EXPECT_NEAR(pose.heading_rad, 3.0 * kPi / 2.0, 1e-4);
    // Endpoints swap with the travel direction: departing B means the
    // bit-exact canonical coordinate of B.
    const GroundTruthPose start = truth_pose(
        grid_.base, in_transit(edge, grid_.node("B"), grid_.node("A"), 0, 1000), 0.0, Tick{0});
    EXPECT_EQ(start.position, *grid_.base.geometry()->node_position(grid_.node("B")));
}

TEST_F(TruthPoseTest, BentPolylineInterpolatesByArcLengthNotPointIndex) {
    // HALF the total arc length (~124 m of ~248 m) falls short of the
    // bend (~137 m along the east leg): the point is on the east leg,
    // strictly BEFORE the vertex. An index-based halfway point (one of
    // two segments) would sit exactly ON the vertex instead.
    const GroundTruthPose half = truth_pose(
        bent_.base, in_transit(bent_.edge, bent_.x, bent_.y, 0, 10000), 0.0, Tick{5000});
    EXPECT_DOUBLE_EQ(half.position.latitude_deg, 52.0000);  // the east leg keeps latitude
    EXPECT_GT(half.position.longitude_deg, 10.0000);
    EXPECT_LT(half.position.longitude_deg, 10.0019);  // strictly before the bend
    EXPECT_NEAR(half.heading_rad, kPi / 2.0, 1e-4);

    // THREE QUARTERS of the arc (~186 m) is past the bend: the north
    // leg owns the point — longitude exactly the bend's, latitude
    // strictly between bend and Y, heading exactly north (a meridian is
    // a geodesic; atan2(0, +) is exactly 0).
    const GroundTruthPose past = truth_pose(
        bent_.base, in_transit(bent_.edge, bent_.x, bent_.y, 0, 10000), 0.0, Tick{7500});
    EXPECT_DOUBLE_EQ(past.position.longitude_deg, 10.0020);
    EXPECT_GT(past.position.latitude_deg, 52.0000);
    EXPECT_LT(past.position.latitude_deg, 52.0010);
    EXPECT_DOUBLE_EQ(past.heading_rad, 0.0);
}

TEST_F(TruthPoseTest, NearVertexSegmentSelectionIsDeterministicFromBothSides) {
    // The vertex sits at ~0.552 of the arc (east leg ~137 m, north leg
    // ~111 m). Half-open [start, end) segment ownership: just below the
    // vertex the east leg owns the traveler, just above it the north
    // leg does — deterministic on both sides of the bend.
    const GroundTruthPose before = truth_pose(
        bent_.base, in_transit(bent_.edge, bent_.x, bent_.y, 0, 10000), 0.0, Tick{5500});
    EXPECT_NEAR(before.heading_rad, kPi / 2.0, 1e-4);  // still the east leg
    const GroundTruthPose after = truth_pose(
        bent_.base, in_transit(bent_.edge, bent_.x, bent_.y, 0, 10000), 0.0, Tick{5600});
    EXPECT_DOUBLE_EQ(after.heading_rad, 0.0);  // the north leg
}

TEST_F(TruthPoseTest, EarlyAndLateLegHeadings) {
    const GroundTruthPose early = truth_pose(
        bent_.base, in_transit(bent_.edge, bent_.x, bent_.y, 0, 10000), 0.0, Tick{100});
    EXPECT_NEAR(early.heading_rad, kPi / 2.0, 1e-4);
    const GroundTruthPose late = truth_pose(
        bent_.base, in_transit(bent_.edge, bent_.x, bent_.y, 0, 10000), 0.0, Tick{9900});
    EXPECT_DOUBLE_EQ(late.heading_rad, 0.0);
}

TEST_F(TruthPoseTest, DeadReckoningFollowsBentPolylineInBothDirectionsWithoutSnapping) {
    for (const bool reverse : {false, true}) {
        for (const double scale_error : {0.0, 0.1}) {
            const auto start = reverse ? bent_.y : bent_.x;
            const auto goal = reverse ? bent_.x : bent_.y;
            fleet::robot::Robot robot{fleet::common::RobotId{1}, {start, goal}, bent_.base, {}};
            robot.configure_dead_reckoning({scale_error, 0.0});
            ASSERT_TRUE(robot.begin_transit(Tick{0}, 1000));
            const auto initial = truth_pose(bent_.base, robot.state(), 0.0, Tick{0});
            (void)robot.apply_gnss_sample(fleet::localization::LocalizationEstimate{
                initial.position, initial.heading_rad, Tick{0}});
            const auto final = truth_pose(bent_.base, robot.state(), 0.0, Tick{1000});
            EXPECT_TRUE(robot.complete_transit());
            const auto& estimate = *robot.localization().estimate();
            const double error = fleet::world::localization_position_error_m(final, estimate);
            if (scale_error == 0.0) {
                EXPECT_LT(error, 0.01);
            } else {
                EXPECT_GT(error, 10.0);
            }
            EXPECT_NEAR(estimate.heading_rad, final.heading_rad, 1e-12);
        }
    }
}

TEST_F(TruthPoseTest, PositionErrorDiagnosticIsMetricAndReadOnly) {
    const GroundTruthPose truth{{0.0, 0.0}, 0.0, Tick{10}};
    const fleet::localization::LocalizationEstimate estimate{{0.0, 0.001}, 0.0, Tick{0}};
    EXPECT_NEAR(fleet::world::localization_position_error_m(truth, estimate), 111.19508, 1e-5);
    EXPECT_EQ(estimate.estimated_at, Tick{0});
}

}  // namespace

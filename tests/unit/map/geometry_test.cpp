#include "fleet/map/base_map.hpp"
#include "fleet/map/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::MapVersion;
using fleet::common::NodeId;
using fleet::map::BaseMap;
using fleet::map::CoordinateReferenceSystem;
using fleet::map::DynamicMapOverlay;
using fleet::map::MapGeometry;
using fleet::map::MapView;
using fleet::map::Wgs84Coordinate;
using fleet::planning::AStarPlanner;
using fleet::planning::Route;
using fleet::testsupport::make_grid_map;

constexpr Wgs84Coordinate kHamburg{53.5511, 9.9937};
constexpr Wgs84Coordinate kBerlin{52.5200, 13.4050};
constexpr Wgs84Coordinate kMidpoint{53.1000, 11.5000};

class MapGeometryTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] std::size_t node_count() const {
        return grid_.base.graph().node_count();
    }
    [[nodiscard]] std::size_t edge_count() const {
        return grid_.base.graph().edge_count();
    }
    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

TEST_F(MapGeometryTest, BaseMapRemainsValidWithoutGeometry) {
    const BaseMap map{grid_.base.graph(), MapVersion{1}};
    EXPECT_EQ(map.geometry(), nullptr);
    EXPECT_EQ(map.graph().node_count(), node_count());
}

TEST_F(MapGeometryTest, AttachesCoordinatesAndPolylinesToValidEntries) {
    MapGeometry::Builder builder{node_count(), edge_count()};
    builder.set_node_position(grid_.node("A"), kHamburg);
    builder.set_node_position(grid_.node("B"), kBerlin);
    builder.set_edge_polyline(edge("A", "B"), {kHamburg, kMidpoint, kBerlin});
    MapGeometry geometry = builder.build();

    const BaseMap map{grid_.base.graph(), MapVersion{1}, std::move(geometry)};
    ASSERT_NE(map.geometry(), nullptr);
    EXPECT_EQ(map.geometry()->crs(), CoordinateReferenceSystem::Wgs84);

    const Wgs84Coordinate* position = map.geometry()->node_position(grid_.node("A"));
    ASSERT_NE(position, nullptr);
    EXPECT_EQ(*position, kHamburg);

    const std::vector<Wgs84Coordinate>* polyline =
        map.geometry()->edge_polyline(edge("A", "B"));
    ASSERT_NE(polyline, nullptr);
    ASSERT_EQ(polyline->size(), 3U);
    EXPECT_EQ((*polyline)[1], kMidpoint);

    // Absent entries read as nullptr (documented fallback).
    EXPECT_EQ(map.geometry()->node_position(grid_.node("L")), nullptr);
    EXPECT_EQ(map.geometry()->edge_polyline(edge("C", "D")), nullptr);
}

TEST_F(MapGeometryTest, RejectsUnknownNodeId) {
    MapGeometry::Builder builder{node_count(), edge_count()};
    EXPECT_THROW(static_cast<void>(builder.set_node_position(NodeId{9999}, kHamburg)),
                 std::invalid_argument);
}

TEST_F(MapGeometryTest, RejectsUnknownEdgeId) {
    MapGeometry::Builder builder{node_count(), edge_count()};
    EXPECT_THROW(static_cast<void>(builder.set_edge_polyline(EdgeId{9999},
                                                             {kHamburg, kBerlin})),
                 std::invalid_argument);
}

TEST_F(MapGeometryTest, RejectsOutOfRangeCoordinates) {
    MapGeometry::Builder builder{node_count(), edge_count()};
    EXPECT_THROW(static_cast<void>(builder.set_node_position(
                      grid_.node("A"), Wgs84Coordinate{90.5, 0.0})),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(builder.set_node_position(
                      grid_.node("A"), Wgs84Coordinate{0.0, -180.5})),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(builder.set_node_position(
                      grid_.node("A"), Wgs84Coordinate{std::nan(""), 0.0})),
                 std::invalid_argument);
}

TEST_F(MapGeometryTest, RejectsShortOrInvalidPolylines) {
    MapGeometry::Builder builder{node_count(), edge_count()};
    EXPECT_THROW(static_cast<void>(builder.set_edge_polyline(edge("A", "B"), {})),
                 std::invalid_argument);
    EXPECT_THROW(
        static_cast<void>(builder.set_edge_polyline(edge("A", "B"), {kHamburg})),
        std::invalid_argument);
    EXPECT_THROW(static_cast<void>(builder.set_edge_polyline(
                      edge("A", "B"), {kHamburg, Wgs84Coordinate{95.0, 0.0}})),
                 std::invalid_argument);
}

TEST_F(MapGeometryTest, PolylineMustFollowEdgeOrientation) {
    // Edge A-B is stored as (a=A, b=B): the polyline must run from A's
    // coordinate to B's coordinate (ADR-012).
    MapGeometry::Builder correct{node_count(), edge_count()};
    correct.set_node_position(grid_.node("A"), kHamburg);
    correct.set_node_position(grid_.node("B"), kBerlin);
    correct.set_edge_polyline(edge("A", "B"), {kHamburg, kMidpoint, kBerlin});
    EXPECT_NO_THROW(static_cast<void>(
        BaseMap{grid_.base.graph(), MapVersion{1}, correct.build()}));

    MapGeometry::Builder reversed{node_count(), edge_count()};
    reversed.set_node_position(grid_.node("A"), kHamburg);
    reversed.set_node_position(grid_.node("B"), kBerlin);
    reversed.set_edge_polyline(edge("A", "B"), {kBerlin, kMidpoint, kHamburg});
    EXPECT_THROW(
        static_cast<void>(BaseMap{grid_.base.graph(), MapVersion{1}, reversed.build()}),
        std::invalid_argument);
}

TEST_F(MapGeometryTest, PolylineOrientationUncheckedWithoutEndpointCoordinates) {
    // Straight-line fallback: without the endpoint coordinates the
    // orientation cannot be verified and is not rejected (documented).
    MapGeometry::Builder builder{node_count(), edge_count()};
    builder.set_edge_polyline(edge("A", "B"), {kHamburg, kBerlin});
    EXPECT_NO_THROW(static_cast<void>(
        BaseMap{grid_.base.graph(), MapVersion{1}, builder.build()}));
}

TEST_F(MapGeometryTest, RejectsGeometrySizeMismatch) {
    MapGeometry::Builder builder{node_count() + 1, edge_count()};
    builder.set_node_position(NodeId{static_cast<std::uint32_t>(node_count())},
                              kHamburg);  // valid for the builder, not for the map
    EXPECT_THROW(
        static_cast<void>(BaseMap{grid_.base.graph(), MapVersion{1}, builder.build()}),
        std::invalid_argument);
}

TEST_F(MapGeometryTest, GeometryIsImmutableAfterConstruction) {
    MapGeometry::Builder builder{node_count(), edge_count()};
    builder.set_node_position(grid_.node("A"), kHamburg);
    (void)builder.build();
    // The builder is single-shot, mirroring Graph::Builder (ADR-001).
    EXPECT_THROW(
        static_cast<void>(builder.set_node_position(grid_.node("B"), kBerlin)),
        std::logic_error);
    EXPECT_THROW(static_cast<void>(builder.build()), std::logic_error);
}

TEST_F(MapGeometryTest, PlannerIgnoresGeometry) {
    // ADR-012, locked by test: the same topology with and without the
    // geographic side produces the identical route.
    MapGeometry::Builder builder{node_count(), edge_count()};
    for (const fleet::map::Node& node : grid_.base.graph().nodes()) {
        builder.set_node_position(
            node.id, Wgs84Coordinate{50.0 + node.position.x, 8.0 + node.position.y});
    }
    const BaseMap geographic{grid_.base.graph(), MapVersion{1}, builder.build()};
    const BaseMap plain{grid_.base.graph(), MapVersion{1}};

    const AStarPlanner planner;
    const DynamicMapOverlay empty_overlay{edge_count()};
    const Route geographic_route = planner.plan(MapView{geographic, empty_overlay},
                                                grid_.node("I"), grid_.node("H"));
    const Route plain_route =
        planner.plan(MapView{plain, empty_overlay}, grid_.node("I"), grid_.node("H"));

    ASSERT_TRUE(geographic_route.found);
    EXPECT_EQ(geographic_route, plain_route);  // operator<=>: full route equality
}

}  // namespace

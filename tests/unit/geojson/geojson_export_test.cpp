#include "fleet/geojson/geojson_export.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/scenario/trace.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::MapVersion;
using fleet::geojson::position_text;
using fleet::geojson::write_base_map_geojson;
using fleet::geojson::write_trace_geojson;
using fleet::map::BaseMap;
using fleet::map::EdgeDirection;
using fleet::map::MapGeometry;
using fleet::map::NodePosition;
using fleet::map::Wgs84Coordinate;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceValue;
using fleet::testsupport::make_grid_map;

constexpr Wgs84Coordinate kHamburg{53.5511, 9.9937};
constexpr Wgs84Coordinate kBerlin{52.5200, 13.4050};
constexpr Wgs84Coordinate kMidpoint{53.1000, 11.5000};

// A -- B with a polyline on the single edge; geographic map fixture.
[[nodiscard]] BaseMap geographic_line_map(Wgs84Coordinate* a_out, Wgs84Coordinate* b_out) {
    fleet::map::Graph::Builder builder;
    const auto a = builder.add_node("A", NodePosition{0.0, 0.0});
    const auto b = builder.add_node("B", NodePosition{1.0, 0.0});
    builder.connect(a, b, 5.0);
    const fleet::map::Graph graph = builder.build();

    MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    geometry.set_node_position(a, kHamburg);
    geometry.set_node_position(b, kBerlin);
    geometry.set_edge_polyline(fleet::common::EdgeId{0}, {kHamburg, kMidpoint, kBerlin});

    *a_out = kHamburg;
    *b_out = kBerlin;
    return BaseMap{graph, MapVersion{1}, geometry.build()};
}

TEST(GeoJsonExportTest, PositionTextIsLongitudeFirst) {
    // THE coordinate-order boundary, locked: Wgs84Coordinate stores
    // {latitude, longitude}; GeoJSON positions are [longitude, latitude].
    EXPECT_EQ(position_text(Wgs84Coordinate{53.5511, 9.9937}), "[9.9937000,53.5511000]");
    EXPECT_EQ(position_text(Wgs84Coordinate{-33.92, 151.19}), "[151.1900000,-33.9200000]");
}

TEST(GeoJsonExportTest, MapExportRequiresGeographicSide) {
    const fleet::testsupport::GridMap grid = make_grid_map();
    const BaseMap plain{grid.base.graph(), MapVersion{1}};
    std::ostringstream output;
    EXPECT_THROW(static_cast<void>(write_base_map_geojson(plain, output)),
                 std::invalid_argument);
}

TEST(GeoJsonExportTest, MapExportEmitsNodesThenEdgesDeterministically) {
    Wgs84Coordinate a, b;
    const BaseMap map = geographic_line_map(&a, &b);

    std::ostringstream first;
    const std::size_t features = write_base_map_geojson(map, first);
    EXPECT_EQ(features, 3U);  // 2 nodes + 1 edge

    const std::string text = first.str();
    // Node Point before edge LineString; polyline with 3 points;
    // properties stable for tooling correlation.
    EXPECT_NE(text.find("\"type\":\"Point\",\"coordinates\":[9.9937000,53.5511000]"),
              std::string::npos);
    EXPECT_NE(text.find("\"type\":\"LineString\",\"coordinates\":"
                        "[[9.9937000,53.5511000],[11.5000000,53.1000000],"
                        "[13.4050000,52.5200000]]"),
              std::string::npos);
    EXPECT_NE(text.find("\"name\":\"A\""), std::string::npos);
    EXPECT_NE(text.find("\"edge\":0,\"from\":\"A\",\"to\":\"B\","
                        "\"direction\":\"bidirectional\",\"cost\":5.00"),
              std::string::npos);

    // Deterministic: same input, byte-identical output.
    std::ostringstream second;
    (void)write_base_map_geojson(map, second);
    EXPECT_EQ(first.str(), second.str());
}

TEST(GeoJsonExportTest, MapExportFallsBackToStraightEdgesWithoutPolylines) {
    fleet::map::Graph::Builder builder;
    const auto a = builder.add_node("A", NodePosition{0.0, 0.0});
    const auto b = builder.add_node("B", NodePosition{1.0, 0.0});
    builder.connect(a, b, 2.0, EdgeDirection::Forward);  // no polyline
    const fleet::map::Graph graph = builder.build();
    MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    geometry.set_node_position(a, kHamburg);
    geometry.set_node_position(b, kBerlin);
    const BaseMap map{graph, MapVersion{1}, geometry.build()};

    std::ostringstream output;
    (void)write_base_map_geojson(map, output);
    const std::string text = output.str();
    // Straight-line fallback + direction property from ADR-013.
    EXPECT_NE(text.find("\"coordinates\":[[9.9937000,53.5511000],[13.4050000,52.5200000]]"),
              std::string::npos);
    EXPECT_NE(text.find("\"direction\":\"forward\""), std::string::npos);
}

TEST(GeoJsonExportTest, ReverseEdgeKeepsCanonicalPolylineAndExplicitDirection) {
    // ADR-013/015 contract: the stored polyline stays in canonical A->B
    // order; reverse-ness lives ONLY in the direction property, never
    // inferred from coordinate order.
    fleet::map::Graph::Builder builder;
    const auto a = builder.add_node("A", NodePosition{0.0, 0.0});
    const auto b = builder.add_node("B", NodePosition{1.0, 0.0});
    builder.connect(a, b, 3.0, EdgeDirection::Reverse);
    const fleet::map::Graph graph = builder.build();
    MapGeometry::Builder geometry{graph.node_count(), graph.edge_count()};
    geometry.set_node_position(a, kHamburg);
    geometry.set_node_position(b, kBerlin);
    geometry.set_edge_polyline(fleet::common::EdgeId{0}, {kHamburg, kMidpoint, kBerlin});
    const BaseMap map{graph, MapVersion{1}, geometry.build()};

    std::ostringstream output;
    (void)write_base_map_geojson(map, output);
    const std::string text = output.str();
    // Canonical stored order (Hamburg -> midpoint -> Berlin)...
    EXPECT_NE(text.find("\"coordinates\":[[9.9937000,53.5511000],[11.5000000,53.1000000],"
                        "[13.4050000,52.5200000]]"),
              std::string::npos);
    // ...and the explicit direction property.
    EXPECT_NE(text.find("\"direction\":\"reverse\""), std::string::npos);
}

TEST(GeoJsonExportTest, TraceExportReconstructsTrajectories) {
    // Hand-built trace: robot_a departs A->B then B->A (returns), robot_b
    // departs A->B; robot_a completes at A. Robots must appear in
    // first-appearance order; consecutive duplicate points merge.
    Wgs84Coordinate a, b;
    const BaseMap map = geographic_line_map(&a, &b);
    const auto departure = [](std::uint64_t at, const char* source, const char* from,
                              const char* to) {
        return TraceEvent{fleet::common::Tick{at},
                          source,
                          "departure",
                          {{"edge", std::string{""}},
                           {"from", std::string{from}},
                           {"to", std::string{to}},
                           {"arrival", std::int64_t{0}}}};
    };
    const std::vector<TraceEvent> events{
        departure(0, "robot_a", "A", "B"),
        departure(1000, "robot_a", "B", "A"),
        departure(500, "robot_b", "A", "B"),
        TraceEvent{fleet::common::Tick{2000},
                   "robot_a",
                   "mission_complete",
                   {{"goal", std::string{"A"}}}},
    };

    std::ostringstream output;
    const std::size_t features = write_trace_geojson(map, events, output);
    ASSERT_EQ(features, 3U);  // robot_a LineString + completion Point, robot_b LineString

    const std::string text = output.str();
    // robot_a first (first appearance), its trajectory A->B->A as three
    // points (no duplicate merge across B).
    const std::size_t robot_a = text.find("\"robot\":\"robot_a\"");
    const std::size_t robot_b = text.find("\"robot\":\"robot_b\"");
    ASSERT_NE(robot_a, std::string::npos);
    ASSERT_NE(robot_b, std::string::npos);
    EXPECT_LT(robot_a, robot_b);
    EXPECT_NE(text.find("\"coordinates\":[[9.9937000,53.5511000],[13.4050000,52.5200000],"
                        "[9.9937000,53.5511000]]"),
              std::string::npos);
    EXPECT_NE(text.find("\"mission_complete\":true"), std::string::npos);

    // Deterministic.
    std::ostringstream again;
    (void)write_trace_geojson(map, events, again);
    EXPECT_EQ(output.str(), again.str());
}

TEST(GeoJsonExportTest, TraceExportFailsOnUnknownNode) {
    Wgs84Coordinate a, b;
    const BaseMap map = geographic_line_map(&a, &b);
    const std::vector<TraceEvent> events{
        TraceEvent{fleet::common::Tick{0},
                   "robot_a",
                   "departure",
                   {{"from", std::string{"A"}}, {"to", std::string{"NOWHERE"}}}},
    };
    std::ostringstream output;
    EXPECT_THROW(static_cast<void>(write_trace_geojson(map, events, output)),
                 std::invalid_argument);
}

TEST(GeoJsonExportTest, TraceExportIgnoresNonGeographicEvents) {
    Wgs84Coordinate a, b;
    const BaseMap map = geographic_line_map(&a, &b);
    const std::vector<TraceEvent> events{
        TraceEvent{fleet::common::Tick{0}, "world", "scenario", {{"name", std::string{"x"}}}},
        TraceEvent{fleet::common::Tick{1}, "world", "link_state", {{"up", false}}},
        TraceEvent{fleet::common::Tick{2}, "station", "reconcile", {{"from", std::int64_t{1}}}},
    };
    std::ostringstream output;
    EXPECT_EQ(write_trace_geojson(map, events, output), 0U);
    EXPECT_EQ(output.str(), "{\"type\":\"FeatureCollection\",\"features\":[]}\n");
}

TEST(GeoJsonExportTest, TrajectoryCardinalityZeroAndCompletionOnly) {
    // A robot with zero location samples but a completed mission emits
    // exactly ONE Point (the completion marker) and NO LineString —
    // the 0-samples case of the cardinality contract.
    Wgs84Coordinate a, b;
    const BaseMap map = geographic_line_map(&a, &b);
    const std::vector<TraceEvent> events{
        TraceEvent{fleet::common::Tick{100},
                   "robot_a",
                   "mission_complete",
                   {{"goal", std::string{"A"}}}},
    };
    std::ostringstream output;
    const std::size_t features = write_trace_geojson(map, events, output);
    ASSERT_EQ(features, 1U);
    const std::string text = output.str();
    EXPECT_EQ(text.find("\"type\":\"LineString\""), std::string::npos);
    EXPECT_NE(text.find("\"type\":\"Point\",\"coordinates\":[9.9937000,53.5511000]"),
              std::string::npos);
    EXPECT_NE(text.find("\"mission_complete\":true"), std::string::npos);
}

TEST(GeoJsonExportTest, TrajectoryCardinalityTwoOrMoreIsLineString) {
    // One departure = exactly two samples = one LineString with two
    // positions; never a Point for a moved robot, never one-point
    // LineStrings (departure events always append from AND to).
    Wgs84Coordinate a, b;
    const BaseMap map = geographic_line_map(&a, &b);
    const std::vector<TraceEvent> events{
        TraceEvent{fleet::common::Tick{0},
                   "robot_a",
                   "departure",
                   {{"from", std::string{"A"}}, {"to", std::string{"B"}}}},
    };
    std::ostringstream output;
    const std::size_t features = write_trace_geojson(map, events, output);
    ASSERT_EQ(features, 1U);
    const std::string text = output.str();
    EXPECT_NE(text.find("\"type\":\"LineString\",\"coordinates\":"
                        "[[9.9937000,53.5511000],[13.4050000,52.5200000]]"),
              std::string::npos);
    EXPECT_EQ(text.find("\"mission_complete\""), std::string::npos);
}

}  // namespace

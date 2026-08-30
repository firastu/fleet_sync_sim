#include "fleet/osm/osm_pbf_importer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::NodeId;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDirection;
using fleet::map::MapView;
using fleet::map::Wgs84Coordinate;
using fleet::osm::OsmImportResult;
using fleet::osm::OsmPbfImporter;
using fleet::planning::AStarPlanner;
using fleet::planning::Route;

constexpr double kEarthRadiusMeters = 6371008.8;  // importer's documented constant

[[nodiscard]] double haversine(const Wgs84Coordinate& a, const Wgs84Coordinate& b) {
    const double lat1 = a.latitude_deg * std::numbers::pi / 180.0;
    const double lat2 = b.latitude_deg * std::numbers::pi / 180.0;
    const double dlat = (b.latitude_deg - a.latitude_deg) * std::numbers::pi / 180.0;
    const double dlon = (b.longitude_deg - a.longitude_deg) * std::numbers::pi / 180.0;
    const double h = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) *
                         std::sin(dlon / 2.0);
    return 2.0 * kEarthRadiusMeters * std::asin(std::sqrt(h));
}

[[nodiscard]] double length_of(const std::vector<Wgs84Coordinate>& points) {
    double total = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        total += haversine(points[i - 1], points[i]);
    }
    return total;
}

constexpr Wgs84Coordinate c100{53.55000, 9.99000};
constexpr Wgs84Coordinate c101{53.55001, 9.99010};
// Node 102 is deliberately OFF the 100->104 line: way 500 is a bent
// segment (regression guard for full-polyline costs).
constexpr Wgs84Coordinate c102{53.55006, 9.99020};
constexpr Wgs84Coordinate c103{53.55003, 9.99030};
constexpr Wgs84Coordinate c104{53.55004, 9.99040};
constexpr Wgs84Coordinate c200{53.55014, 9.99050};
constexpr Wgs84Coordinate c300{53.55024, 9.99060};
constexpr Wgs84Coordinate c400{53.55034, 9.99070};
constexpr Wgs84Coordinate c500{53.55044, 9.99080};

class OsmPbfImporterTest : public ::testing::Test {
protected:
    const OsmPbfImporter importer_{};
    const OsmImportResult result_{importer_.import("fixtures/osm/tiny_network.osm.pbf")};

    [[nodiscard]] NodeId node(const char* osm_id) const {
        for (const fleet::map::Node& candidate : result_.map.graph().nodes()) {
            if (candidate.name == osm_id) {
                return candidate.id;
            }
        }
        throw std::runtime_error(std::string("node not found: ") + osm_id);
    }
};

TEST_F(OsmPbfImporterTest, StatsReportSeenImportedNodesAndEdges) {
    // 4 ways in the file (footway seen but not imported).
    EXPECT_EQ(result_.stats.ways_seen, 4U);
    EXPECT_EQ(result_.stats.ways_imported, 3U);
    EXPECT_EQ(result_.stats.topology_nodes, 4U);  // 100, 104, 300, 500
    EXPECT_EQ(result_.stats.directed_edges, 3U);
    EXPECT_NE(result_.map.geometry(), nullptr);
}

TEST_F(OsmPbfImporterTest, NodeIdsFollowAscendingOsmIds) {
    ASSERT_EQ(result_.map.graph().node_count(), 4U);
    EXPECT_EQ(result_.map.graph().node(NodeId{0}).name, "100");
    EXPECT_EQ(result_.map.graph().node(NodeId{1}).name, "104");
    EXPECT_EQ(result_.map.graph().node(NodeId{2}).name, "300");
    EXPECT_EQ(result_.map.graph().node(NodeId{3}).name, "500");
    // Intermediate and excluded OSM nodes are NOT graph nodes.
    for (const fleet::map::Node& candidate : result_.map.graph().nodes()) {
        EXPECT_NE(candidate.name, "101");
        EXPECT_NE(candidate.name, "200");
        EXPECT_NE(candidate.name, "600");
    }
}

TEST_F(OsmPbfImporterTest, EdgeTableFollowsCanonicalWayOrder) {
    // Sorted by (way_id, segment_index): way 500, then 600, then 700.
    ASSERT_EQ(result_.map.graph().edge_count(), 3U);
    const fleet::map::Edge& e0 = result_.map.graph().edge(EdgeId{0});
    EXPECT_EQ(e0.a, node("100"));
    EXPECT_EQ(e0.b, node("104"));
    EXPECT_EQ(e0.direction, EdgeDirection::Bidirectional);

    const fleet::map::Edge& e1 = result_.map.graph().edge(EdgeId{1});
    EXPECT_EQ(e1.a, node("104"));
    EXPECT_EQ(e1.b, node("300"));
    EXPECT_EQ(e1.direction, EdgeDirection::Forward);  // oneway=yes

    const fleet::map::Edge& e2 = result_.map.graph().edge(EdgeId{2});
    EXPECT_EQ(e2.a, node("500"));  // way-order endpoints (a=500, b=300)...
    EXPECT_EQ(e2.b, node("300"));
    EXPECT_EQ(e2.direction, EdgeDirection::Reverse);  // ...traversable 300->500
}

TEST_F(OsmPbfImporterTest, CostsAreHaversinePolylineLengths) {
    const fleet::map::Edge& e0 = result_.map.graph().edge(EdgeId{0});
    EXPECT_NEAR(e0.base_cost, length_of({c100, c101, c102, c103, c104}), 1e-9);
    const fleet::map::Edge& e1 = result_.map.graph().edge(EdgeId{1});
    EXPECT_NEAR(e1.base_cost, length_of({c104, c200, c300}), 1e-9);
    const fleet::map::Edge& e2 = result_.map.graph().edge(EdgeId{2});
    EXPECT_NEAR(e2.base_cost, length_of({c500, c400, c300}), 1e-9);
}

TEST_F(OsmPbfImporterTest, BentSegmentCostExceedsEndpointGeodesic) {
    // Regression guard (ADR-014): way 500's geometry bends at node 102
    // (off the 100->104 line). The edge cost must be the SUM over
    // consecutive polyline points — not the endpoint-to-endpoint
    // geodesic, which underestimates winding roads.
    const fleet::map::Edge& bent = result_.map.graph().edge(EdgeId{0});
    const double endpoint_chord = haversine(c100, c104);
    const double expected_sum = length_of({c100, c101, c102, c103, c104});
    ASSERT_GT(expected_sum, endpoint_chord + 1.0);  // fixture is measurably bent
    EXPECT_NEAR(bent.base_cost, expected_sum, 1e-9);
    EXPECT_GT(bent.base_cost, endpoint_chord + 1.0);
}

TEST_F(OsmPbfImporterTest, GeometryCarriesCanonicalCoordinatesAndPolylines) {
    const fleet::map::MapGeometry* geometry = result_.map.geometry();
    ASSERT_NE(geometry, nullptr);

    // Canonical node coordinates (exact values from the fixture).
    EXPECT_EQ(*geometry->node_position(node("100")), c100);
    EXPECT_EQ(*geometry->node_position(node("104")), c104);
    EXPECT_EQ(*geometry->node_position(node("300")), c300);
    EXPECT_EQ(*geometry->node_position(node("500")), c500);

    // Intermediate way geometry survives as polylines; endpoints reuse
    // the canonical node values exactly (the BaseMap orientation
    // contract validated at construction).
    const std::vector<Wgs84Coordinate>* p0 = geometry->edge_polyline(EdgeId{0});
    ASSERT_NE(p0, nullptr);
    EXPECT_EQ(*p0, (std::vector<Wgs84Coordinate>{c100, c101, c102, c103, c104}));

    const std::vector<Wgs84Coordinate>* p1 = geometry->edge_polyline(EdgeId{1});
    ASSERT_NE(p1, nullptr);
    EXPECT_EQ(*p1, (std::vector<Wgs84Coordinate>{c104, c200, c300}));

    const std::vector<Wgs84Coordinate>* p2 = geometry->edge_polyline(EdgeId{2});
    ASSERT_NE(p2, nullptr);
    EXPECT_EQ(*p2, (std::vector<Wgs84Coordinate>{c500, c400, c300}));
}

TEST_F(OsmPbfImporterTest, PlannerRespectsImportedOneWays) {
    const DynamicMapOverlay empty{result_.map.graph().edge_count()};
    const AStarPlanner planner;

    // 100 -> 300: over the bidirectional edge, then the forward oneway.
    const Route forward =
        planner.plan(MapView{result_.map, empty}, node("100"), node("300"));
    ASSERT_TRUE(forward.found);
    EXPECT_NEAR(forward.cost,
                result_.map.graph().edge(EdgeId{0}).base_cost +
                    result_.map.graph().edge(EdgeId{1}).base_cost,
                1e-9);

    // 300 -> 100: the forward oneway cannot be entered backwards, and
    // node 500 is a one-way sink => unreachable.
    EXPECT_FALSE(planner.plan(MapView{result_.map, empty}, node("300"), node("100")).found);

    // 300 -> 500: the reverse (oneway=-1) edge is traversable this way.
    const Route to_500 = planner.plan(MapView{result_.map, empty}, node("300"), node("500"));
    ASSERT_TRUE(to_500.found);
    EXPECT_NEAR(to_500.cost, result_.map.graph().edge(EdgeId{2}).base_cost, 1e-9);

    // 500 -> 300: against oneway=-1 => unreachable.
    EXPECT_FALSE(planner.plan(MapView{result_.map, empty}, node("500"), node("300")).found);
}

TEST_F(OsmPbfImporterTest, ImportTwiceProducesIdenticalRepresentation) {
    // The determinism contract (ADR-014): same PBF + same options =>
    // identical ids, adjacency, costs, coordinates, polylines, stats.
    const OsmPbfImporter second_pass{};
    const OsmImportResult again = second_pass.import("fixtures/osm/tiny_network.osm.pbf");

    ASSERT_EQ(again.stats.ways_seen, result_.stats.ways_seen);
    ASSERT_EQ(again.stats.ways_imported, result_.stats.ways_imported);
    ASSERT_EQ(again.stats.topology_nodes, result_.stats.topology_nodes);
    ASSERT_EQ(again.stats.directed_edges, result_.stats.directed_edges);

    ASSERT_EQ(again.map.graph().node_count(), result_.map.graph().node_count());
    for (const fleet::map::Node& expected : result_.map.graph().nodes()) {
        const fleet::map::Node& actual = again.map.graph().node(expected.id);
        EXPECT_EQ(actual.name, expected.name);
        EXPECT_DOUBLE_EQ(actual.position.x, expected.position.x);
        EXPECT_DOUBLE_EQ(actual.position.y, expected.position.y);
    }
    ASSERT_EQ(again.map.graph().edge_count(), result_.map.graph().edge_count());
    for (const fleet::map::Edge& expected : result_.map.graph().edges()) {
        const fleet::map::Edge& actual = again.map.graph().edge(expected.id);
        EXPECT_EQ(actual.a, expected.a);
        EXPECT_EQ(actual.b, expected.b);
        EXPECT_DOUBLE_EQ(actual.base_cost, expected.base_cost);
        EXPECT_EQ(actual.direction, expected.direction);
    }

    ASSERT_NE(again.map.geometry(), nullptr);
    for (const fleet::map::Node& node_entry : result_.map.graph().nodes()) {
        const Wgs84Coordinate* expected = result_.map.geometry()->node_position(node_entry.id);
        const Wgs84Coordinate* actual = again.map.geometry()->node_position(node_entry.id);
        ASSERT_EQ(actual == nullptr, expected == nullptr);
        if (expected != nullptr) {
            EXPECT_EQ(*actual, *expected);
        }
    }
    for (const fleet::map::Edge& edge_entry : result_.map.graph().edges()) {
        const auto* expected = result_.map.geometry()->edge_polyline(edge_entry.id);
        const auto* actual = again.map.geometry()->edge_polyline(edge_entry.id);
        ASSERT_EQ(actual == nullptr, expected == nullptr);
        if (expected != nullptr) {
            EXPECT_EQ(*actual, *expected);
        }
    }
}

TEST(OsmPbfImporterErrorTest, ExoticOnewayValuesFailLoudly) {
    const OsmPbfImporter importer{};
    try {
        static_cast<void>(importer.import("fixtures/osm/tiny_invalid_oneway.osm.pbf"));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string{error.what()}.find("way 900"), std::string::npos);
        EXPECT_NE(std::string{error.what()}.find("alternating"), std::string::npos);
    }
}

TEST(OsmPbfImporterErrorTest, AntiParallelWaysFailExplicitly) {
    // Two ways connecting the same pair in opposite directions must fail
    // with BOTH ways named — never silently merged (ADR-013).
    const OsmPbfImporter importer{};
    try {
        static_cast<void>(importer.import("fixtures/osm/tiny_antiparallel.osm.pbf"));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string{error.what()}.find("way 910"), std::string::npos);
        EXPECT_NE(std::string{error.what()}.find("way 920"), std::string::npos);
    }
}

TEST(OsmPbfImporterErrorTest, ParallelSameDirectionWaysFailExplicitly) {
    // Same-direction parallels are STILL two physical roads (different
    // geometry/identity/class possible) and must fail exactly like
    // anti-parallel ones — direction agreement must not cause a silent
    // merge (ADR-013/014).
    const OsmPbfImporter importer{};
    try {
        static_cast<void>(importer.import("fixtures/osm/tiny_parallel_same_direction.osm.pbf"));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string{error.what()}.find("way 930"), std::string::npos);
        EXPECT_NE(std::string{error.what()}.find("way 940"), std::string::npos);
    }
}

TEST(OsmPbfImporterErrorTest, RevisitedPairWithinOneWayFailsExplicitly) {
    // A single way whose refs produce the same retained pair twice
    // (60->61 then 61->60) is also unrepresentable: the error names the
    // way AND both segment indices.
    const OsmPbfImporter importer{};
    try {
        static_cast<void>(importer.import("fixtures/osm/tiny_self_loop_pair.osm.pbf"));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string{error.what()}.find("way 960 segment 0"), std::string::npos);
        EXPECT_NE(std::string{error.what()}.find("way 960 segment 1"), std::string::npos);
    }
}

TEST(OsmPbfImporterErrorTest, ImplicitRoundaboutNeverImportsSilently) {
    // junction=roundabout with an absent oneway tag has an IMPLICIT
    // direction in OSM; importing it as bidirectional would confidently
    // create the wrong topology. The import fails loudly instead
    // (ADR-014); explicit oneway values go through the normal parser.
    const OsmPbfImporter importer{};
    try {
        static_cast<void>(importer.import("fixtures/osm/tiny_roundabout.osm.pbf"));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string{error.what()}.find("way 950"), std::string::npos);
        EXPECT_NE(std::string{error.what()}.find("roundabout"), std::string::npos);
    }
}

TEST(OsmPbfImporterErrorTest, MissingFileFails) {
    const OsmPbfImporter importer{};
    EXPECT_THROW(static_cast<void>(importer.import("fixtures/osm/does_not_exist.osm.pbf")),
                 std::exception);
}

}  // namespace

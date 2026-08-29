#include "fleet/planning/a_star_planner.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/route.hpp"
#include "test_maps.hpp"

// Note: MapView models dynamic state as Open/Blocked only (see
// map_view.hpp); dynamic traversal-cost multipliers do not exist yet, so
// there is deliberately no dynamic-cost-override planning test here. One
// will be added together with that feature. Static builder cost overrides
// are covered by PrefersCheaperMultiHopOverExpensiveDirectEdge.

namespace {

using fleet::common::EdgeId;
using fleet::common::MapVersion;
using fleet::common::NodeId;
using fleet::common::OverlayVersion;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::BaseMap;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::map::MapView;
using fleet::map::NodePosition;
using fleet::planning::AStarPlanner;
using fleet::planning::Route;
using fleet::planning::euclidean_heuristic;
using fleet::testsupport::make_grid_map;

class AStarPlannerTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    DynamicMapOverlay overlay_{grid_.base.graph().edge_count()};
    const MapView view_{grid_.base, overlay_};
    const AStarPlanner planner_;

    [[nodiscard]] EdgeId edge_id(const char* a, const char* b) const {
        const std::optional<EdgeId> edge =
            grid_.base.graph().edge_between(grid_.node(a), grid_.node(b));
        EXPECT_TRUE(edge.has_value());
        return edge.value_or(EdgeId{});
    }

    [[nodiscard]] std::vector<NodeId> ids(std::initializer_list<const char*> names) const {
        std::vector<NodeId> result;
        for (const char* name : names) {
            result.push_back(grid_.node(name));
        }
        return result;
    }

    [[nodiscard]] static EdgeDynamicState blocked_observation(std::uint64_t sequence) {
        return EdgeDynamicState{
            .status = EdgeStatus::Blocked,
            .observed_at = Tick{5000},
            .source = RobotId{1},
            .source_sequence = sequence,
            .confidence = 0.9,
        };
    }
};

TEST_F(AStarPlannerTest, FindsShortestRouteOnUnitCostGrid) {
    const Route route = planner_.plan(view_, grid_.node("A"), grid_.node("D"));

    ASSERT_TRUE(route.found);
    EXPECT_EQ(route.nodes, ids({"A", "B", "C", "D"}));
    EXPECT_EQ(route.edges,
              (std::vector<EdgeId>{edge_id("A", "B"), edge_id("B", "C"), edge_id("C", "D")}));
    EXPECT_DOUBLE_EQ(route.cost, 3.0);
    EXPECT_EQ(route.base_version, MapVersion{1});
    EXPECT_EQ(route.overlay_version, OverlayVersion{0});
}

TEST_F(AStarPlannerTest, PrefersCheaperMultiHopOverExpensiveDirectEdge) {
    fleet::map::Graph::Builder builder;
    const NodeId s = builder.add_node("S", NodePosition{0.0, 0.0});
    const NodeId m = builder.add_node("M", NodePosition{2.0, 0.0});
    const NodeId t = builder.add_node("T", NodePosition{4.0, 0.0});
    (void)builder.connect(s, t, 10.0);  // few hops, expensive
    (void)builder.connect(s, m);        // euclidean default: 2.0
    (void)builder.connect(m, t);        // euclidean default: 2.0

    const BaseMap base{builder.build(), MapVersion{1}};
    const DynamicMapOverlay overlay{base.graph().edge_count()};
    const MapView view{base, overlay};

    const Route route = planner_.plan(view, s, t);

    ASSERT_TRUE(route.found);
    EXPECT_EQ(route.nodes, (std::vector<NodeId>{s, m, t}));
    EXPECT_EQ(route.edges.size(), 2U);
    EXPECT_DOUBLE_EQ(route.cost, 4.0);
}

TEST_F(AStarPlannerTest, ReroutesAroundBlockedEdgeOnRoute) {
    const Route initial = planner_.plan(view_, grid_.node("A"), grid_.node("D"));
    ASSERT_TRUE(initial.found);
    ASSERT_TRUE(initial.uses_edge(edge_id("B", "C")));

    ASSERT_TRUE(overlay_.apply(edge_id("B", "C"), blocked_observation(7)));

    const Route rerouted = planner_.plan(view_, grid_.node("A"), grid_.node("D"));
    ASSERT_TRUE(rerouted.found);
    EXPECT_FALSE(rerouted.uses_edge(edge_id("B", "C")));
    // Canonical reroute is A-B-F-G-C-D (cost 5.0): the equal-cost
    // alternative A-E-F-G-C-D loses the tie at F, because B (id 1) is
    // expanded before E (id 4) and equal-cost predecessors are kept
    // (strict improvement only, ADR-003).
    EXPECT_EQ(rerouted.nodes, ids({"A", "B", "F", "G", "C", "D"}));
    EXPECT_DOUBLE_EQ(rerouted.cost, 5.0);
}

TEST_F(AStarPlannerTest, StartEqualsGoalYieldsZeroCostRoute) {
    const Route route = planner_.plan(view_, grid_.node("F"), grid_.node("F"));

    EXPECT_TRUE(route.found);
    EXPECT_EQ(route.nodes, (std::vector<NodeId>{grid_.node("F")}));
    EXPECT_TRUE(route.edges.empty());
    EXPECT_DOUBLE_EQ(route.cost, 0.0);
}

TEST_F(AStarPlannerTest, EqualCostTiesResolveToCanonicalRoute) {
    // A -> L has several optimal routes (cost 5.0). Under the documented
    // rules (pop order f, g, NodeId; strict-improvement parents; CSR
    // adjacency order) the canonical route runs along the top row first.
    const Route expected = planner_.plan(view_, grid_.node("A"), grid_.node("L"));
    ASSERT_TRUE(expected.found);
    EXPECT_EQ(expected.nodes, ids({"A", "B", "C", "D", "H", "L"}));
    EXPECT_EQ(expected.edges,
              (std::vector<EdgeId>{edge_id("A", "B"), edge_id("B", "C"), edge_id("C", "D"),
                                   edge_id("D", "H"), edge_id("H", "L")}));
    EXPECT_DOUBLE_EQ(expected.cost, 5.0);

    // Repeated planning must be bit-stable, including scratch reuse.
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(planner_.plan(view_, grid_.node("A"), grid_.node("L")), expected);
    }
}

TEST_F(AStarPlannerTest, RouteIsContiguousAndParallel) {
    const Route route = planner_.plan(view_, grid_.node("E"), grid_.node("H"));
    ASSERT_TRUE(route.found);
    ASSERT_EQ(route.edges.size() + 1U, route.nodes.size());

    for (std::size_t i = 0; i < route.edges.size(); ++i) {
        const std::optional<EdgeId> via =
            grid_.base.graph().edge_between(route.nodes[i], route.nodes[i + 1]);
        ASSERT_TRUE(via.has_value());
        EXPECT_EQ(*via, route.edges[i]);
    }
    EXPECT_DOUBLE_EQ(route.cost, 3.0);  // E-F-G-H
}

TEST_F(AStarPlannerTest, SequentialQueriesAreIndependent) {
    // The planner owns no scratch state; this locks in that nothing from a
    // previous query (or any hidden global state) leaks into the next one.
    ASSERT_TRUE(planner_.plan(view_, grid_.node("A"), grid_.node("L")).found);

    const Route route = planner_.plan(view_, grid_.node("E"), grid_.node("H"));
    ASSERT_TRUE(route.found);
    EXPECT_EQ(route.nodes, ids({"E", "F", "G", "H"}));
    EXPECT_DOUBLE_EQ(route.cost, 3.0);
}

TEST_F(AStarPlannerTest, PlanningDoesNotMutateBaseMapOrOverlay) {
    const OverlayVersion overlay_version_before = overlay_.version();
    const std::size_t tracked_before = overlay_.tracked_count();
    const MapVersion base_version_before = grid_.base.version();

    const Route route = planner_.plan(view_, grid_.node("A"), grid_.node("D"));
    ASSERT_TRUE(route.found);

    EXPECT_EQ(overlay_.version(), overlay_version_before);
    EXPECT_EQ(overlay_.tracked_count(), tracked_before);
    EXPECT_EQ(grid_.base.version(), base_version_before);
    // Planning is a pure function of the view: same inputs, same route.
    EXPECT_EQ(planner_.plan(view_, grid_.node("A"), grid_.node("D")), route);
}

TEST_F(AStarPlannerTest, EuclideanHeuristicFindsSameUniqueOptimum) {
    // On the unit grid Euclidean distance is admissible (costs equal
    // distances) and A -> D has a unique optimum, so a valid heuristic
    // cannot change the result.
    AStarPlanner planner{euclidean_heuristic(grid_.base.graph())};

    const Route route = planner.plan(view_, grid_.node("A"), grid_.node("D"));

    ASSERT_TRUE(route.found);
    EXPECT_EQ(route.nodes, ids({"A", "B", "C", "D"}));
    EXPECT_DOUBLE_EQ(route.cost, 3.0);
}

TEST_F(AStarPlannerTest, ReportsNoRouteWhenGoalIsSevered) {
    // D's only edges are C-D and D-H; blocking both severs it.
    ASSERT_TRUE(overlay_.apply(edge_id("C", "D"), blocked_observation(1)));
    ASSERT_TRUE(overlay_.apply(edge_id("D", "H"), blocked_observation(2)));

    const Route route = planner_.plan(view_, grid_.node("A"), grid_.node("D"));

    EXPECT_FALSE(route.found);
    EXPECT_TRUE(route.nodes.empty());
    EXPECT_TRUE(route.edges.empty());
    EXPECT_DOUBLE_EQ(route.cost, 0.0);
    // Provenance of the failed attempt is still recorded.
    EXPECT_EQ(route.overlay_version, OverlayVersion{2});
}

}  // namespace

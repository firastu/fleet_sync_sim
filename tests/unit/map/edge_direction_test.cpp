#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"

#include <gtest/gtest.h>

#include <optional>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::MapVersion;
using fleet::common::NodeId;
using fleet::map::BaseMap;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDirection;
using fleet::map::MapView;
using fleet::map::NodePosition;
using fleet::planning::AStarPlanner;
using fleet::planning::Route;

// Line map S -- M -- G; the middle edge's direction is the test variable.
// Explicit costs keep node positions inert.
[[nodiscard]] BaseMap make_line(EdgeDirection middle_direction, NodeId* s, NodeId* m,
                                NodeId* g) {
    fleet::map::Graph::Builder builder;
    *s = builder.add_node("S", NodePosition{0.0, 0.0});
    *m = builder.add_node("M", NodePosition{1.0, 0.0});
    *g = builder.add_node("G", NodePosition{2.0, 0.0});
    builder.connect(*s, *m, 1.0);
    builder.connect(*m, *g, 1.0, middle_direction);
    return BaseMap{builder.build(), MapVersion{1}};
}

TEST(EdgeDirectionTest, BuilderDefaultsToBidirectional) {
    fleet::map::Graph::Builder builder;
    const NodeId a = builder.add_node("a", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("b", NodePosition{1.0, 0.0});
    const EdgeId edge = builder.connect(a, b, 2.0);
    EXPECT_EQ(builder.build().edge(edge).direction, EdgeDirection::Bidirectional);
}

TEST(EdgeDirectionTest, ConnectStoresDirection) {
    fleet::map::Graph::Builder builder;
    const NodeId a = builder.add_node("a", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("b", NodePosition{1.0, 0.0});
    const EdgeId forward = builder.connect(a, b, 1.0, EdgeDirection::Forward);
    const fleet::map::Graph graph = builder.build();
    EXPECT_EQ(graph.edge(forward).direction, EdgeDirection::Forward);
    EXPECT_EQ(graph.edge(forward).a, a);  // a -> b ordering is meaningful
}

TEST(EdgeDirectionTest, TraversabilityMatrix) {
    NodeId s, m, g;
    const BaseMap base = make_line(EdgeDirection::Forward, &s, &m, &g);
    const DynamicMapOverlay overlay{base.graph().edge_count()};
    const MapView view{base, overlay};

    const EdgeId sm = base.graph().edge_between(s, m).value();
    const EdgeId mg = base.graph().edge_between(m, g).value();

    // Bidirectional default: both sides.
    EXPECT_TRUE(view.traversable_from(sm, s));
    EXPECT_TRUE(view.traversable_from(sm, m));
    // Forward (m -> g): only from m.
    EXPECT_TRUE(view.traversable_from(mg, m));
    EXPECT_FALSE(view.traversable_from(mg, g));
    // Not an endpoint: not traversable.
    EXPECT_FALSE(view.traversable_from(mg, s));
}

TEST(EdgeDirectionTest, ReverseDirectionIsSymmetric) {
    NodeId s, m, g;
    const BaseMap base = make_line(EdgeDirection::Reverse, &s, &m, &g);
    const DynamicMapOverlay overlay{base.graph().edge_count()};
    const MapView view{base, overlay};
    const EdgeId mg = base.graph().edge_between(m, g).value();
    EXPECT_FALSE(view.traversable_from(mg, m));  // Reverse: g -> m only
    EXPECT_TRUE(view.traversable_from(mg, g));
}

TEST(EdgeDirectionTest, ForwardEdgeIsUnreachableAgainstItsDirection) {
    NodeId s, m, g;
    const BaseMap base = make_line(EdgeDirection::Forward, &s, &m, &g);
    const DynamicMapOverlay overlay{base.graph().edge_count()};
    const AStarPlanner planner;

    const Route forward = planner.plan(MapView{base, overlay}, s, g);
    ASSERT_TRUE(forward.found);
    EXPECT_DOUBLE_EQ(forward.cost, 2.0);

    const Route backward = planner.plan(MapView{base, overlay}, g, s);
    EXPECT_FALSE(backward.found);  // the only path uses m->g backwards
}

TEST(EdgeDirectionTest, PlannerDetoursAroundOneWay) {
    // Diamond: S-A, A->G (forward, cost 1), S-B, B-G (bidirectional,
    // cost 2 each). S->G uses the one-way shortcut (cost 2 via A);
    // G->S cannot enter A->G backwards and detours via B (cost 4).
    fleet::map::Graph::Builder builder;
    const NodeId s = builder.add_node("S", NodePosition{0.0, 0.0});
    const NodeId a = builder.add_node("A", NodePosition{1.0, 1.0});
    const NodeId b = builder.add_node("B", NodePosition{1.0, -1.0});
    const NodeId g = builder.add_node("G", NodePosition{2.0, 0.0});
    builder.connect(s, a, 1.0);
    builder.connect(a, g, 1.0, EdgeDirection::Forward);
    builder.connect(s, b, 2.0);
    builder.connect(b, g, 2.0);
    const BaseMap base{builder.build(), MapVersion{1}};
    const DynamicMapOverlay overlay{base.graph().edge_count()};
    const AStarPlanner planner;

    const Route to_goal = planner.plan(MapView{base, overlay}, s, g);
    ASSERT_TRUE(to_goal.found);
    EXPECT_DOUBLE_EQ(to_goal.cost, 2.0);
    ASSERT_EQ(to_goal.nodes.size(), 3U);
    EXPECT_EQ(to_goal.nodes[1], a);

    const Route from_goal = planner.plan(MapView{base, overlay}, g, s);
    ASSERT_TRUE(from_goal.found);
    EXPECT_DOUBLE_EQ(from_goal.cost, 4.0);
    ASSERT_EQ(from_goal.nodes.size(), 3U);
    EXPECT_EQ(from_goal.nodes[1], b);
}

TEST(EdgeDirectionTest, BlockingAOneWayEdgeStillBlocksItsDirection) {
    // Dynamic state composes with direction: blocking a forward-only
    // edge removes it entirely (it was only ever traversable forward).
    NodeId s, m, g;
    const BaseMap base = make_line(EdgeDirection::Forward, &s, &m, &g);
    DynamicMapOverlay overlay{base.graph().edge_count()};
    const EdgeId mg = base.graph().edge_between(m, g).value();
    (void)overlay.apply(mg, fleet::map::EdgeDynamicState{
                                .status = fleet::map::EdgeStatus::Blocked,
                                .source = fleet::common::RobotId{1},
                            });
    const MapView view{base, overlay};
    EXPECT_FALSE(view.traversable_from(mg, m));
    const AStarPlanner planner;
    EXPECT_FALSE(planner.plan(view, s, g).found);
}

}  // namespace

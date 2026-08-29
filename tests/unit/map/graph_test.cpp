#include "fleet/map/graph.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "fleet/common/ids.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::NodeId;
using fleet::map::AdjacencyEntry;
using fleet::map::Graph;
using fleet::map::NodePosition;
using fleet::testsupport::make_grid_map;

std::vector<NodeId> neighbor_ids(const Graph& graph, NodeId node) {
    std::vector<NodeId> neighbors;
    for (const AdjacencyEntry& entry : graph.adjacency(node)) {
        neighbors.push_back(entry.neighbor);
    }
    return neighbors;
}

TEST(GraphTest, BuildsThreeByFourGrid) {
    const auto grid = make_grid_map();
    const Graph& graph = grid.base.graph();

    EXPECT_EQ(graph.node_count(), 12U);
    EXPECT_EQ(graph.edge_count(), 17U);
    EXPECT_EQ(graph.node(grid.node("K")).name, "K");
    EXPECT_DOUBLE_EQ(graph.node(grid.node("D")).position.x, 3.0);
}

TEST(GraphTest, ContainsRejectsIdsOutsideTheGraph) {
    const auto grid = make_grid_map();
    const Graph& graph = grid.base.graph();

    EXPECT_TRUE(graph.contains(grid.node("A")));
    EXPECT_FALSE(graph.contains(NodeId{9999}));
}

TEST(GraphTest, AdjacencyReflectsTopology) {
    const auto grid = make_grid_map();
    const Graph& graph = grid.base.graph();

    EXPECT_EQ(neighbor_ids(graph, grid.node("A")),
              (std::vector<NodeId>{grid.node("B"), grid.node("E")}));
    EXPECT_EQ(neighbor_ids(graph, grid.node("L")),
              (std::vector<NodeId>{grid.node("K"), grid.node("H")}));
}

TEST(GraphTest, AdjacencyIsOrderedByEdgeInsertion) {
    const auto grid = make_grid_map();
    const Graph& graph = grid.base.graph();

    // G's neighbors appear in edge-insertion order (rows first, then
    // columns). This ordering is a determinism guarantee for later planners.
    EXPECT_EQ(neighbor_ids(graph, grid.node("G")),
              (std::vector<NodeId>{grid.node("F"), grid.node("H"), grid.node("C"),
                                   grid.node("K")}));
}

TEST(GraphTest, EdgeBetweenFindsUndirectedEdge) {
    const auto grid = make_grid_map();
    const Graph& graph = grid.base.graph();

    const std::optional<EdgeId> fg = graph.edge_between(grid.node("F"), grid.node("G"));
    ASSERT_TRUE(fg.has_value());
    EXPECT_DOUBLE_EQ(graph.edge(*fg).base_cost, 1.0);
    // Symmetric: the edge is the same object in either direction.
    EXPECT_EQ(graph.edge_between(grid.node("G"), grid.node("F")), fg);
    EXPECT_EQ(graph.edge_between(grid.node("A"), grid.node("L")), std::nullopt);
}

TEST(GraphBuilderTest, RejectsUnknownNodeId) {
    Graph::Builder builder;
    const NodeId a = builder.add_node("A", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("B", NodePosition{1.0, 0.0});

    EXPECT_THROW(builder.connect(NodeId{42}, b), std::invalid_argument);
    EXPECT_THROW(builder.connect(a, NodeId{42}), std::invalid_argument);
}

TEST(GraphBuilderTest, RejectsEmptyNodeName) {
    Graph::Builder builder;
    EXPECT_THROW(builder.add_node("", NodePosition{0.0, 0.0}), std::invalid_argument);
}

TEST(GraphBuilderTest, RejectsSelfLoop) {
    Graph::Builder builder;
    const NodeId a = builder.add_node("A", NodePosition{0.0, 0.0});
    EXPECT_THROW(builder.connect(a, a), std::invalid_argument);
}

TEST(GraphBuilderTest, RejectsDuplicateEdgeInEitherDirection) {
    Graph::Builder builder;
    const NodeId a = builder.add_node("A", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("B", NodePosition{1.0, 0.0});
    (void)builder.connect(a, b);

    EXPECT_THROW(builder.connect(a, b), std::invalid_argument);
    EXPECT_THROW(builder.connect(b, a), std::invalid_argument);
}

TEST(GraphBuilderTest, RejectsNonPositiveOrInfiniteCost) {
    Graph::Builder builder;
    const NodeId a = builder.add_node("A", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("B", NodePosition{1.0, 0.0});

    EXPECT_THROW(builder.connect(a, b, 0.0), std::invalid_argument);
    EXPECT_THROW(builder.connect(a, b, -2.0), std::invalid_argument);
    EXPECT_THROW(builder.connect(a, b, std::numeric_limits<double>::infinity()),
                 std::invalid_argument);
}

TEST(GraphBuilderTest, DefaultCostIsEuclideanDistance) {
    Graph::Builder builder;
    const NodeId a = builder.add_node("A", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("B", NodePosition{3.0, 4.0});
    const EdgeId edge = builder.connect(a, b);

    const Graph graph = builder.build();
    EXPECT_NEAR(graph.edge(edge).base_cost, 5.0, 1e-12);
}

TEST(GraphBuilderTest, ExplicitCostOverridesDistance) {
    Graph::Builder builder;
    const NodeId a = builder.add_node("A", NodePosition{0.0, 0.0});
    const NodeId b = builder.add_node("B", NodePosition{3.0, 4.0});
    const EdgeId edge = builder.connect(a, b, 2.5);

    EXPECT_DOUBLE_EQ(builder.build().edge(edge).base_cost, 2.5);
}

TEST(GraphBuilderTest, BuildCanBeCalledOnlyOnce) {
    Graph::Builder builder;
    (void)builder.add_node("A", NodePosition{0.0, 0.0});
    (void)builder.build();

    EXPECT_THROW(builder.build(), std::logic_error);
    EXPECT_THROW(builder.add_node("B", NodePosition{0.0, 1.0}), std::logic_error);
}

TEST(GraphBuilderTest, EmptyGraphIsAllowed) {
    Graph::Builder builder;
    const Graph graph = builder.build();

    EXPECT_EQ(graph.node_count(), 0U);
    EXPECT_EQ(graph.edge_count(), 0U);
}

}  // namespace

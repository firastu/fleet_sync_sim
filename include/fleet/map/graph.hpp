#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"

namespace fleet::map {

// Planar position in map units. Used for A* heuristics and, by default, to
// derive edge traversal costs.
struct NodePosition {
    double x = 0.0;
    double y = 0.0;
};

[[nodiscard]] double distance(NodePosition from, NodePosition to) noexcept;

struct Node {
    common::NodeId id{};
    std::string name;  // human-readable, for logs, tests and scenarios
    NodePosition position{};
};

// Directionality of an edge (ADR-013). Bidirectional is the default and
// the pre-existing semantics of every map so far; Forward/Reverse model
// one-way edges (e.g. OSM oneway ways). `Forward` means traversable only
// from `a` to `b`; `Reverse` only from `b` to `a`. Direction affects
// TRAVERSABILITY only — never cost, knowledge or reconciliation.
enum class EdgeDirection : std::uint8_t {
    Bidirectional,
    Forward,
    Reverse,
};

// Edge of the road network. `a` and `b` are endpoints (ordered for the
// one-way direction, unordered otherwise); `base_cost` is the static
// traversal cost in the allowed direction(s).
struct Edge {
    common::EdgeId id{};
    common::NodeId a{};
    common::NodeId b{};
    double base_cost = 1.0;
    EdgeDirection direction = EdgeDirection::Bidirectional;
};

// One entry of a node's adjacency list: the edge to traverse and the node
// reached on the other side.
struct AdjacencyEntry {
    common::EdgeId edge{};
    common::NodeId neighbor{};
};

// Immutable road-network topology stored with compressed sparse row (CSR)
// adjacency: cache-friendly, allocation-free iteration and stable neighbor
// order. Deterministic A* tie-breaking later relies on that ordering.
//
// Construction is only possible through Graph::Builder, which validates the
// input. NodeId/EdgeId are dense indices assigned at build time.
//
// Thread-safety: immutable after construction; safe to share without
// synchronization.
class Graph {
public:
    class Builder;

    [[nodiscard]] std::size_t node_count() const noexcept { return nodes_.size(); }
    [[nodiscard]] std::size_t edge_count() const noexcept { return edges_.size(); }

    // Whole-table access in id order; NodeId/EdgeId index into these spans.
    [[nodiscard]] std::span<const Node> nodes() const noexcept { return nodes_; }
    [[nodiscard]] std::span<const Edge> edges() const noexcept { return edges_; }

    [[nodiscard]] bool contains(common::NodeId id) const noexcept;

    // Precondition: contains(id) — ids must originate from the same graph.
    [[nodiscard]] const Node& node(common::NodeId id) const noexcept;

    // Precondition: id.value() < edge_count().
    [[nodiscard]] const Edge& edge(common::EdgeId id) const noexcept;

    // Returns the unique edge between adjacent nodes, nullopt otherwise.
    // Complexity: O(degree(a)).
    [[nodiscard]] std::optional<common::EdgeId> edge_between(common::NodeId a,
                                                             common::NodeId b) const;

    // Neighbor entries ordered by edge insertion.
    // Precondition: contains(id).
    [[nodiscard]] std::span<const AdjacencyEntry> adjacency(common::NodeId id) const noexcept;

private:
    Graph(std::vector<Node> nodes,
          std::vector<Edge> edges,
          std::vector<std::uint32_t> offsets,
          std::vector<AdjacencyEntry> entries) noexcept;

    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::vector<std::uint32_t> offsets_;   // size node_count() + 1 (CSR row bounds)
    std::vector<AdjacencyEntry> entries_;  // size 2 * edge_count()

    friend class Builder;
};

// Two-phase construction: collect nodes and edges, validate, then build()
// the immutable Graph.
//
// Error handling: Builder methods throw std::invalid_argument on malformed
// input (unknown ids, self-loops, duplicate edges, non-positive costs) and
// std::logic_error on reuse after build(). Graph construction is therefore
// a validated, cannot-fail operation — a deliberate split between
// "programmer error at construction time" (exceptions) and runtime lookups
// (documented preconditions, checked with asserts in debug builds).
class Graph::Builder {
public:
    common::NodeId add_node(std::string name, NodePosition position);

    // Adds an edge (bidirectional by default; one-way via `direction`,
    // ADR-013). When `cost` is not given it defaults to the Euclidean
    // distance between the endpoints.
    common::EdgeId connect(common::NodeId a,
                           common::NodeId b,
                           std::optional<double> cost = std::nullopt,
                           EdgeDirection direction = EdgeDirection::Bidirectional);

    [[nodiscard]] Graph build();

private:
    struct PendingEdge {
        common::NodeId a;
        common::NodeId b;
        double cost;
        EdgeDirection direction;
    };

    void require_known(common::NodeId id) const;
    void require_absent(common::NodeId a, common::NodeId b) const;

    std::vector<Node> nodes_;
    std::vector<PendingEdge> pending_;
    bool built_ = false;
};

}  // namespace fleet::map

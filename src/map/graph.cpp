#include "fleet/map/graph.hpp"

#include <cassert>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace fleet::map {

double distance(NodePosition from, NodePosition to) noexcept {
    return std::hypot(from.x - to.x, from.y - to.y);
}

bool Graph::contains(common::NodeId id) const noexcept {
    return id.value() < nodes_.size();
}

const Node& Graph::node(common::NodeId id) const noexcept {
    assert(contains(id));
    return nodes_[id.value()];
}

const Edge& Graph::edge(common::EdgeId id) const noexcept {
    assert(id.value() < edges_.size());
    return edges_[id.value()];
}

std::optional<common::EdgeId> Graph::edge_between(common::NodeId a, common::NodeId b) const {
    assert(contains(a) && contains(b));
    for (const AdjacencyEntry& entry : adjacency(a)) {
        if (entry.neighbor == b) {
            return entry.edge;
        }
    }
    return std::nullopt;
}

std::span<const AdjacencyEntry> Graph::adjacency(common::NodeId id) const noexcept {
    assert(contains(id));
    const std::size_t begin = offsets_[id.value()];
    const std::size_t end = offsets_[id.value() + 1];
    return std::span<const AdjacencyEntry>{entries_.data() + begin, end - begin};
}

Graph::Graph(std::vector<Node> nodes,
             std::vector<Edge> edges,
             std::vector<std::uint32_t> offsets,
             std::vector<AdjacencyEntry> entries) noexcept
    : nodes_{std::move(nodes)},
      edges_{std::move(edges)},
      offsets_{std::move(offsets)},
      entries_{std::move(entries)} {}

common::NodeId Graph::Builder::add_node(std::string name, NodePosition position) {
    if (built_) {
        throw std::logic_error("Graph::Builder: build() already called");
    }
    if (name.empty()) {
        throw std::invalid_argument("Graph::Builder::add_node: name must not be empty");
    }
    const auto id = static_cast<std::uint32_t>(nodes_.size());
    nodes_.push_back(Node{common::NodeId{id}, std::move(name), position});
    return common::NodeId{id};
}

common::EdgeId Graph::Builder::connect(common::NodeId a, common::NodeId b,
                                       std::optional<double> cost,
                                       EdgeDirection direction) {
    if (built_) {
        throw std::logic_error("Graph::Builder: build() already called");
    }
    require_known(a);
    require_known(b);
    if (a == b) {
        throw std::invalid_argument("Graph::Builder::connect: self-loops are not supported");
    }
    require_absent(a, b);

    const double resolved = cost.value_or(distance(nodes_[a.value()].position,
                                                   nodes_[b.value()].position));
    if (!std::isfinite(resolved) || resolved <= 0.0) {
        throw std::invalid_argument("Graph::Builder::connect: cost must be finite and > 0");
    }
    pending_.push_back(PendingEdge{a, b, resolved, direction});
    return common::EdgeId{static_cast<std::uint32_t>(pending_.size() - 1)};
}

Graph Graph::Builder::build() {
    if (built_) {
        throw std::logic_error("Graph::Builder: build() already called");
    }
    built_ = true;

    const std::size_t node_count = nodes_.size();
    const std::size_t edge_count = pending_.size();

    std::vector<Edge> edges;
    edges.reserve(edge_count);
    for (std::size_t i = 0; i < edge_count; ++i) {
        edges.push_back(Edge{common::EdgeId{static_cast<std::uint32_t>(i)}, pending_[i].a,
                             pending_[i].b, pending_[i].cost, pending_[i].direction});
    }

    // CSR construction: count endpoint degrees, prefix-sum into row bounds,
    // then scatter entries in edge order (which yields deterministic,
    // insertion-ordered adjacency).
    std::vector<std::uint32_t> offsets(node_count + 1, 0);
    for (const PendingEdge& edge : pending_) {
        ++offsets[edge.a.value() + 1];
        ++offsets[edge.b.value() + 1];
    }
    std::partial_sum(offsets.begin(), offsets.end(), offsets.begin());

    std::vector<AdjacencyEntry> entries(2 * edge_count);
    std::vector<std::uint32_t> cursor{offsets};
    for (std::size_t i = 0; i < edge_count; ++i) {
        const common::EdgeId id{static_cast<std::uint32_t>(i)};
        entries[cursor[pending_[i].a.value()]++] = AdjacencyEntry{id, pending_[i].b};
        entries[cursor[pending_[i].b.value()]++] = AdjacencyEntry{id, pending_[i].a};
    }

    return Graph{std::move(nodes_), std::move(edges), std::move(offsets), std::move(entries)};
}

void Graph::Builder::require_known(common::NodeId id) const {
    if (id.value() >= nodes_.size()) {
        throw std::invalid_argument("Graph::Builder: unknown NodeId (" +
                                    std::to_string(id.value()) + ")");
    }
}

void Graph::Builder::require_absent(common::NodeId a, common::NodeId b) const {
    for (const PendingEdge& edge : pending_) {
        if ((edge.a == a && edge.b == b) || (edge.a == b && edge.b == a)) {
            throw std::invalid_argument("Graph::Builder::connect: duplicate edge (" +
                                        std::to_string(a.value()) + ", " +
                                        std::to_string(b.value()) + ")");
        }
    }
}

}  // namespace fleet::map

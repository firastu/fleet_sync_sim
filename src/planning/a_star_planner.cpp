#include "fleet/planning/a_star_planner.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "fleet/map/graph.hpp"

namespace fleet::planning {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Predecessor {
    common::NodeId node{};
    common::EdgeId edge{};
};

struct OpenEntry {
    double f = 0.0;
    double g = 0.0;
    common::NodeId node{};
};

// Ordering for the open list. std::priority_queue surfaces the entry that
// compares greatest under this comparator, so it must return true when `a`
// is worse (less promising) than `b`. This yields the documented pop
// order: lowest f, then lowest g, then lowest NodeId — a strict total
// order on entries; identical entries cannot occur because entries are
// pushed only on strict improvement.
struct LessPromising {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
        if (a.f != b.f) {
            return a.f > b.f;
        }
        if (a.g != b.g) {
            return a.g > b.g;
        }
        return a.node.value() > b.node.value();
    }
};

// All mutable state of a single search. Local to plan(): the planner
// object carries no scratch state, so plan() is const and repeated
// queries cannot interfere. Dense vectors indexed by NodeId mirror the
// graph's CSR representation (O(1) access, no per-expansion allocation).
// If per-search allocation ever shows up in Stage 6 measurements, this is
// the type to hoist into an explicit reusable workspace.
struct SearchState {
    explicit SearchState(std::size_t node_count)
        : g(node_count, kInfinity), parent(node_count, std::nullopt) {}

    std::vector<double> g;                           // best known cost from start
    std::vector<std::optional<Predecessor>> parent;  // how that cost was reached
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, LessPromising> open;
};

}  // namespace

Heuristic euclidean_heuristic(const map::Graph& graph) {
    return [&graph](common::NodeId node, common::NodeId goal) {
        return map::distance(graph.node(node).position, graph.node(goal).position);
    };
}

AStarPlanner::AStarPlanner(Heuristic heuristic) : heuristic_{std::move(heuristic)} {}

double AStarPlanner::heuristic_estimate(common::NodeId node, common::NodeId goal) const {
    const double estimate = heuristic_ ? heuristic_(node, goal) : 0.0;
    // Numeric validity of heuristic values is a hard precondition for
    // well-defined open-list ordering and cost arithmetic.
    assert(std::isfinite(estimate) && estimate >= 0.0 &&
           "AStarPlanner: heuristic returned a non-finite or negative estimate");
    return estimate;
}

Route AStarPlanner::plan(const map::MapView& view, common::NodeId start,
                         common::NodeId goal) const {
    const map::Graph& graph = view.base().graph();
    assert(graph.contains(start) && "AStarPlanner::plan: start not in base map");
    assert(graph.contains(goal) && "AStarPlanner::plan: goal not in base map");

    Route route;
    route.base_version = view.base().version();
    route.overlay_version = view.overlay().version();

    if (start == goal) {
        route.found = true;
        route.nodes.push_back(start);
        route.cost = 0.0;
        return route;
    }

    SearchState search{graph.node_count()};
    search.g[start.value()] = 0.0;
    search.open.push(OpenEntry{heuristic_estimate(start, goal), 0.0, start});

    bool reached = false;
    while (!search.open.empty()) {
        const OpenEntry current = search.open.top();
        search.open.pop();

        // Lazy deletion: an improved cost pushes an additional entry, so an
        // entry whose g exceeds the best known g for its node is stale.
        // Dropping it is safe: the node was already relaxed again with the
        // better cost, so this entry cannot discover anything new. Must be
        // checked *before* the goal test — a stale goal entry must not
        // terminate the search.
        if (current.g > search.g[current.node.value()]) {
            continue;
        }
        if (current.node == goal) {
            reached = true;
            break;  // first current goal pop is optimal for admissible h
        }
        // Note: popping does NOT close the node permanently. If a lower
        // g-score is discovered later, a fresh entry is pushed and the
        // node is expanded again — re-opening support is exactly why the
        // algorithm requires admissibility only, not consistency.

        for (const map::AdjacencyEntry& adjacency : view.adjacency(current.node)) {
            // One-way edges are not enterable against their direction
            // (ADR-013); blocked edges are skipped as before.
            if (!view.traversable_from(adjacency.edge, current.node)) {
                continue;
            }
            const std::optional<double> step_cost = view.traversal_cost(adjacency.edge);
            if (!step_cost.has_value()) {
                continue;  // dynamically blocked
            }
            const double candidate_g = current.g + *step_cost;
            const std::size_t index = adjacency.neighbor.value();
            // Strict comparison only (no epsilon): exact ties are resolved
            // by the documented first-found-predecessor-wins rule.
            if (candidate_g < search.g[index]) {
                search.g[index] = candidate_g;
                search.parent[index] = Predecessor{current.node, adjacency.edge};
                search.open.push(
                    OpenEntry{candidate_g + heuristic_estimate(adjacency.neighbor, goal),
                              candidate_g, adjacency.neighbor});
            }
        }
    }

    if (!reached) {
        return route;  // found == false; versions kept as provenance
    }

    // Reconstruct goal -> start, then reverse. The parent chain is acyclic
    // by construction: every edge cost is strictly positive, so g strictly
    // decreases toward the start.
    route.nodes.push_back(goal);
    common::NodeId cursor = goal;
    while (cursor != start) {
        const std::optional<Predecessor>& predecessor = search.parent[cursor.value()];
        assert(predecessor.has_value() && "AStarPlanner::plan: broken predecessor chain");
        route.edges.push_back(predecessor->edge);
        route.nodes.push_back(predecessor->node);
        cursor = predecessor->node;
    }
    std::reverse(route.nodes.begin(), route.nodes.end());
    std::reverse(route.edges.begin(), route.edges.end());

    route.found = true;
    route.cost = search.g[goal.value()];
    return route;
}

}  // namespace fleet::planning

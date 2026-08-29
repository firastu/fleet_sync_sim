#pragma once

#include <functional>

#include "fleet/common/ids.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/route.hpp"

namespace fleet::planning {

// Estimated remaining cost from `node` to `goal`.
//
// Two independent requirements:
//
// 1. Numeric validity — always required: the callable must return a
//    finite value >= 0 for every node of the base graph. Non-finite or
//    negative values corrupt open-list ordering and cost arithmetic and
//    are rejected by debug assertions in the planner.
//
// 2. Admissibility — required for optimality only: h must never exceed
//    the true cheapest remaining cost under the *effective* traversal
//    costs the MapView reports. An inadmissible heuristic does not break
//    the search itself; it voids only the optimality guarantee (the
//    planner still returns a route, possibly a suboptimal one).
//
// Consistency/monotonicity is NOT required: this planner never
// permanently closes a node. If a node's best known g-score improves
// after it was expanded, a new open-list entry supersedes the old one
// and the node is expanded again. Consistent heuristics merely reduce
// the number of such re-expansions.
using Heuristic = std::function<double(common::NodeId node, common::NodeId goal)>;

// Straight-line heuristic from node positions. Admissible ONLY if every
// edge's base cost is at least the Euclidean distance between its
// endpoints. Builder-default costs satisfy this; explicit cost overrides
// may not (Graph::Builder::connect accepts any cost > 0).
//
// Lifetime: the returned callable borrows `graph` and must not outlive it.
[[nodiscard]] Heuristic euclidean_heuristic(const map::Graph& graph);

// Deterministic A* over a MapView (tie-breaking contract: ADR-003).
// Stage 0 scope: no cancellation, no incremental replanning.
//
// The planner owns only its configuration (the heuristic). All per-search
// state lives in a SearchState local to plan(), so plan() is const and a
// planner instance carries no hidden coupling between queries. If
// per-search allocation ever becomes measurable, a reusable per-worker
// workspace is a Stage 6 optimization — not made now, without
// measurements.
//
// Determinism contract: for a fixed (BaseMap, overlay, start, goal,
// heuristic) plan() always returns the same Route, including equal-cost
// ties, via three rules:
//   1. open-list pop order: lowest f, then lowest g, then lowest NodeId;
//   2. predecessors are replaced only on strict cost improvement, so the
//      first-found predecessor wins equal-cost ties;
//   3. neighbors are relaxed in the graph's CSR (edge-insertion) order.
//
// The default heuristic is zero: the search degenerates to Dijkstra with
// the same tie-breaking rules, and is always optimal. Correctness is
// preferred over "real A*" when no admissible heuristic is available.
//
// No IPlanner interface (yet): there is exactly one implementation and no
// consumer needs runtime polymorphism (Robot does not exist). The boundary
// that matters today is the MapView: planners are pure functions of a
// view. Revisit when a second algorithm (e.g., D* Lite) lands.
//
// Thread-safety: plan() does not mutate planner state, but concurrent
// calls on one instance are safe only if the heuristic callable and the
// map/overlay objects behind the MapView are themselves safe for
// concurrent reads. Stage 0 guarantees neither (ADR-002); treat planners
// as single-threaded until the concurrency stages define those
// guarantees.
class AStarPlanner {
public:
    explicit AStarPlanner(Heuristic heuristic = {});

    // Precondition: start and goal are contained in the base map's graph
    // (asserted in debug builds, mirroring the Graph lookup contract).
    [[nodiscard]] Route plan(const map::MapView& view, common::NodeId start,
                             common::NodeId goal) const;

private:
    [[nodiscard]] double heuristic_estimate(common::NodeId node, common::NodeId goal) const;

    Heuristic heuristic_;
};

}  // namespace fleet::planning

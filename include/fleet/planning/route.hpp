#pragma once

#include <algorithm>
#include <vector>

#include "fleet/common/ids.hpp"

namespace fleet::planning {

// Result of one planning query over a MapView.
//
// The representation is edge-oriented in addition to node-oriented: dynamic
// map updates (MapDelta, later commit) reference EdgeIds, so a route
// consumer must be able to ask "does this update invalidate my route?"
// without re-deriving adjacency.
//
// Invariants (found == true):
//   - nodes.front() == start, nodes.back() == goal;
//   - edges.size() == nodes.size() - 1, and edges[i] connects nodes[i] and
//     nodes[i + 1] in the base graph;
//   - cost == sum of effective traversal costs (MapView::traversal_cost)
//     of the edges, accumulated in route order;
//   - no edge in the route is blocked in the MapView used for planning;
//   - base_version/overlay_version record which map state produced this
//     plan (stale-plan detection later).
//
// found == false: nodes and edges are empty, cost is 0.0; the version
// fields still record the state of the attempted plan.
//
// Thread-safety: plain value type.
struct Route {
    bool found = false;
    std::vector<common::NodeId> nodes;  // start ... goal, inclusive
    std::vector<common::EdgeId> edges;  // parallel to consecutive node pairs
    double cost = 0.0;
    common::MapVersion base_version{};
    common::OverlayVersion overlay_version{};

    // O(route length): does this route traverse `edge`?
    [[nodiscard]] bool uses_edge(common::EdgeId edge) const {
        return std::find(edges.begin(), edges.end(), edge) != edges.end();
    }

    auto operator<=>(const Route&) const = default;
};

}  // namespace fleet::planning

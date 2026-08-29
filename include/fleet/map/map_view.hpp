#pragma once

#include <optional>
#include <span>

#include "fleet/map/base_map.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::map {

// Read-only projection combining an immutable BaseMap with exactly one
// participant's DynamicMapOverlay. This is the only window into map state
// the planner will get: a stable, composed view captured at planning time.
//
// Semantics (v0):
//   - untracked edge    -> base cost, traversable;
//   - tracked + Open    -> base cost (confidence does not scale cost yet);
//   - tracked + Blocked -> not traversable.
//
// Ownership: observes, never owns or copies. A MapView must not outlive the
// BaseMap and DynamicMapOverlay it was created from. Views are cheap value
// objects: create one per planning pass rather than caching across map
// changes.
//
// Thread-safety: not synchronized; single-threaded reference stage (ADR-002).
class MapView {
public:
    MapView(const BaseMap& base, const DynamicMapOverlay& overlay) noexcept;

    [[nodiscard]] const BaseMap& base() const noexcept { return *base_; }

    // The overlay this view was composed with (planners record its version
    // as plan provenance).
    [[nodiscard]] const DynamicMapOverlay& overlay() const noexcept { return *overlay_; }

    // Precondition: base().graph().contains(node).
    [[nodiscard]] std::span<const AdjacencyEntry> adjacency(common::NodeId node) const noexcept;

    [[nodiscard]] bool is_blocked(common::EdgeId edge) const noexcept;

    // nullopt when the edge cannot be traversed (blocked).
    // Precondition: edge.value() < base().graph().edge_count().
    [[nodiscard]] std::optional<double> traversal_cost(common::EdgeId edge) const noexcept;

    [[nodiscard]] const EdgeDynamicState* dynamic_state(common::EdgeId edge) const noexcept;

private:
    const BaseMap* base_;
    const DynamicMapOverlay* overlay_;
};

}  // namespace fleet::map

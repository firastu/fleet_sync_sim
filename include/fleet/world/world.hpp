#pragma once

#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/dynamic_overlay.hpp"

namespace fleet::world {

// Simulation ground truth: the authoritative dynamic state of every edge
// (ADR-011). Robots NEVER access the World directly — truth reaches robot
// belief only through an ObservationModel. The scenario (world changes) and
// tests (direct observation injection) are the only writers/readers besides
// sensing.
//
// Initial truth equals the immutable base map (every edge open): a scenario
// without set_world_edge_state events models an unchanging world, so
// position-based sensing has nothing to report at start.
//
// Determinism: dense vector indexed by EdgeId, exact lookups only.
//
// Lifetime: borrows `base` (must outlive the world).
// Thread-safety: not synchronized (ADR-002).
class World {
public:
    explicit World(const map::BaseMap& base);

    // Authoritative truth change. Setting an edge to its current state is a
    // no-op. Throws std::invalid_argument for an unknown edge.
    void set_edge_state(common::EdgeId edge, map::EdgeStatus status);

    [[nodiscard]] map::EdgeStatus edge_state(common::EdgeId edge) const;

    [[nodiscard]] const map::BaseMap& base() const noexcept { return base_; }

private:
    const map::BaseMap& base_;
    std::vector<map::EdgeStatus> truth_;  // dense by EdgeId
};

}  // namespace fleet::world

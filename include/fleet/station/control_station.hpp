#pragma once

#include "fleet/map/base_map.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/map/map_reconciler.hpp"

namespace fleet::station {

// The local control station: a fleet participant whose role is
// aggregating fleet-wide map knowledge (ADR-008). It is deliberately
// thin in Stage 0 — an overlay plus the shared reconciliation policy —
// with no mission, route or transport knowledge of its own. The station
// is an aggregator, never a hard dependency for robot autonomy: while
// the station is unreachable, robots keep operating; on reconnect, a
// state synchronization (Robot::resynchronize()) closes the gap, and
// the reconciler's idempotence makes re-announced facts safe.
//
// Lifetime: borrows `base` (must outlive the station).
// Thread-safety: not synchronized (ADR-002).
class ControlStation {
public:
    explicit ControlStation(const map::BaseMap& base);

    // Reconciles a transported delta into the station's fleet knowledge.
    map::ReconcileDecision receive(const map::MapDelta& delta);

    [[nodiscard]] const map::DynamicMapOverlay& overlay() const noexcept { return overlay_; }

private:
    map::DynamicMapOverlay overlay_;
    map::MapReconciler reconciler_;
};

}  // namespace fleet::station

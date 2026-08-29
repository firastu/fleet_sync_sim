#pragma once

#include <cassert>
#include <cstdint>
#include <optional>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"

namespace fleet::map {

enum class EdgeStatus : std::uint8_t {
    Open,
    Blocked,
};

// Best-known dynamic state of one edge, as last applied to the owning
// overlay. This is storage, not policy: conflict resolution between
// competing observations lives in the reconciler (later commit).
//
// `source_sequence` is the per-source monotonic counter of the delta that
// carried this state; together with `source` it identifies the exact
// observation, enabling duplicate and stale detection during reconciliation.
struct EdgeDynamicState {
    EdgeStatus status = EdgeStatus::Open;
    common::Tick observed_at{};         // simulation tick of the observation
    common::RobotId source{};           // who observed it
    std::uint64_t source_sequence = 0;  // sequence number within source's delta stream
    double confidence = 1.0;            // sensor confidence, in [0, 1] (producer-clamped)

    // Note: field-wise comparison; with NaN confidences (never expected)
    // equality comparisons behave per IEEE-754.
    constexpr auto operator<=>(const EdgeDynamicState&) const noexcept = default;
};

// Per-participant overlay of dynamic edge states on top of the shared
// BaseMap. Dense vector indexed by EdgeId for O(1) lookup (see ADR-001 for
// the memory trade-off).
//
// Invariants:
//   - tracked_count() == number of slots holding a value;
//   - version() increases by exactly one per *changed* application;
//     idempotent re-application does not bump it;
//   - apply() performs no conflict resolution (mechanism, not policy).
//
// Thread-safety: not synchronized; single-threaded reference stage (ADR-003).
class DynamicMapOverlay {
public:
    explicit DynamicMapOverlay(std::size_t edge_count);

    // Stores `state` for `edge`; returns true when the stored state changed.
    // Precondition: edge.value() < edge_count.
    [[nodiscard]] bool apply(common::EdgeId edge, EdgeDynamicState state);

    // nullptr when the overlay holds no dynamic state for the edge.
    [[nodiscard]] const EdgeDynamicState* find(common::EdgeId edge) const noexcept;

    [[nodiscard]] std::size_t tracked_count() const noexcept { return tracked_count_; }
    [[nodiscard]] common::OverlayVersion version() const noexcept { return version_; }

    // Tracked edges in ascending EdgeId order (deterministic).
    [[nodiscard]] std::vector<common::EdgeId> tracked_edges() const;

private:
    std::vector<std::optional<EdgeDynamicState>> states_;
    std::size_t tracked_count_ = 0;
    common::OverlayVersion version_{0};
};

}  // namespace fleet::map

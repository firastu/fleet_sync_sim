#pragma once

#include <cstdint>
#include <optional>

#include "fleet/common/time.hpp"
#include "fleet/localization/pose.hpp"

namespace fleet::localization {

// Robot-local localization state (#16, ADR-016/018): the LAST VALID
// LocalizationEstimate this robot holds, plus the staleness that grows
// around it. This is BELIEF, not truth — it is written only through GNSS
// sample outcomes and never reads GroundTruthPose, the world or the
// sensor model that produced the sample.
//
// Retention contract (the heart of #16):
//   - apply_sample(estimate) — a successful fix — REPLACES the retained
//     estimate; estimated_at is the fix's own tick;
//   - apply_sample(nullopt) — no fix this sample — RETAINS the previous
//     estimate unchanged: an outage never erases knowledge, it ages it;
//   - before the first successful fix there is NO estimate — none is
//     manufactured (no zero/default coordinate ever becomes belief).
//
// Age is DERIVED, never stored: querying does not mutate state and no
// "last inspected" timestamp exists. age_at(now) is nullopt while no
// estimate exists; otherwise it is now - estimated_at in ticks.
//
// Thread-safety: not synchronized (ADR-002).
class LocalizationTracker {
public:
    // Feeds one GNSS sample outcome into robot-local state (fix replaces,
    // no-fix retains — see the class contract). Throws
    // std::invalid_argument for a fix with non-finite position or
    // heading: NaN/inf never silently become localization state.
    void apply_sample(const std::optional<LocalizationEstimate>& sample);

    // The retained estimate; nullopt before the first successful fix.
    [[nodiscard]] const std::optional<LocalizationEstimate>& estimate() const noexcept {
        return estimate_;
    }

    // Derived age in ticks: nullopt while no estimate exists. Throws
    // std::invalid_argument when now < estimated_at — logical time does
    // not run backward, and a "negative age" would silently corrupt the
    // staleness observable this type exists to provide.
    [[nodiscard]] std::optional<std::uint64_t> age_at(common::Tick now) const;

private:
    std::optional<LocalizationEstimate> estimate_;
};

}  // namespace fleet::localization

#pragma once

#include <cstdint>
#include <optional>

#include "fleet/common/time.hpp"
#include "fleet/localization/pose.hpp"

namespace fleet::localization {

struct DeadReckoningConfig {
    double distance_scale_error = 0.0;
    double heading_drift_rad_per_m = 0.0;

    void validate() const;
};

struct OdometryIncrement {
    double distance_m = 0.0;
    double turn_rad = 0.0;
    common::Tick from{};
    common::Tick to{};
};

// Robot-local localization state (#16, ADR-016/018): the LAST VALID
// LocalizationEstimate this robot holds, plus the staleness that grows
// around it. This is BELIEF, not truth: GNSS sample outcomes and opt-in
// relative odometry propagation write it. It never reads GroundTruthPose,
// the world or the sensor model that produced a sample (ADR-019).
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

    void propagate(const OdometryIncrement& motion, const DeadReckoningConfig& config);

    // The retained estimate; nullopt before the first successful fix.
    [[nodiscard]] const std::optional<LocalizationEstimate>& estimate() const noexcept {
        return estimate_;
    }

    // Derived age in ticks: nullopt while no estimate exists. Throws
    // std::invalid_argument when now < estimated_at — logical time does
    // not run backward, and a "negative age" would silently corrupt the
    // staleness observable this type exists to provide.
    [[nodiscard]] std::optional<std::uint64_t> age_at(common::Tick now) const;

    [[nodiscard]] std::optional<common::Tick> last_fix_at() const noexcept {
        return last_fix_at_;
    }
    [[nodiscard]] bool dead_reckoned() const noexcept { return dead_reckoned_; }

private:
    std::optional<LocalizationEstimate> estimate_;
    std::optional<common::Tick> last_fix_at_;
    bool dead_reckoned_ = false;
};

}  // namespace fleet::localization

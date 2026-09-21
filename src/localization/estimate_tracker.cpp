#include "fleet/localization/estimate_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fleet::localization {

void DeadReckoningConfig::validate() const {
    if (!std::isfinite(distance_scale_error) || distance_scale_error <= -1.0 ||
        !std::isfinite(heading_drift_rad_per_m)) {
        throw std::invalid_argument("dead reckoning: invalid distance scale or heading drift");
    }
}

void LocalizationTracker::propagate(const OdometryIncrement& motion,
                                    const DeadReckoningConfig& config) {
    config.validate();
    if (!std::isfinite(motion.distance_m) || motion.distance_m < 0.0 ||
        !std::isfinite(motion.turn_rad) || motion.to < motion.from) {
        throw std::invalid_argument("dead reckoning: invalid motion increment");
    }
    if (!estimate_) {
        return;
    }
    if (motion.from != estimate_->estimated_at) {
        throw std::invalid_argument("dead reckoning: motion must continue the estimate timestamp");
    }
    auto next = *estimate_;
    next.heading_rad = normalize_heading(next.heading_rad + motion.turn_rad);
    const double distance = motion.distance_m * (1.0 + config.distance_scale_error);
    const double drift = motion.distance_m * config.heading_drift_rad_per_m;
    if (!std::isfinite(distance) || !std::isfinite(drift) || distance > 1000000.0) {
        throw std::invalid_argument("dead reckoning: motion exceeds supported domain");
    }
    const auto steps = static_cast<std::uint64_t>(std::max(1.0, std::ceil(distance / 10.0)));
    const double step_distance = distance / static_cast<double>(steps);
    const double step_drift = drift / static_cast<double>(steps);
    for (std::uint64_t step = 0; step < steps; ++step) {
        const double heading = normalize_heading(next.heading_rad + step_drift / 2.0);
        if (step_distance != 0.0) {
            next.position = apply_en_displacement(next.position,
                step_distance * std::sin(heading), step_distance * std::cos(heading));
        }
        next.heading_rad = normalize_heading(next.heading_rad + step_drift);
    }
    next.estimated_at = motion.to;
    if (motion.to > motion.from || motion.distance_m != 0.0 || motion.turn_rad != 0.0) {
        dead_reckoned_ = true;
    }
    estimate_ = next;
}

void LocalizationTracker::apply_sample(const std::optional<LocalizationEstimate>& sample) {
    if (!sample.has_value()) {
        // No fix this sample: the previous estimate is retained exactly
        // as-is (estimated_at unchanged) — staleness grows around it.
        return;
    }
    if (!std::isfinite(sample->position.latitude_deg) ||
        !std::isfinite(sample->position.longitude_deg) ||
        !std::isfinite(sample->heading_rad)) {
        throw std::invalid_argument(
            "LocalizationTracker: non-finite fix rejected (position or heading)");
    }
    estimate_ = *sample;
    last_fix_at_ = sample->estimated_at;
    dead_reckoned_ = false;
}

std::optional<std::uint64_t> LocalizationTracker::age_at(common::Tick now) const {
    if (!estimate_.has_value()) {
        return std::nullopt;
    }
    if (now < estimate_->estimated_at) {
        throw std::invalid_argument("LocalizationTracker: query tick precedes estimated_at");
    }
    return now.value - estimate_->estimated_at.value;
}

}  // namespace fleet::localization

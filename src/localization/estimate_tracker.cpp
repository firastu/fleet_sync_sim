#include "fleet/localization/estimate_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace fleet::localization {

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

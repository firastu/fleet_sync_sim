#include "fleet/localization/estimate_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fleet::localization {
namespace {

// Mean earth radius in meters — the same physical constant as the world
// truth-pose seam and the OSM importer (ADR-014/018); same libm class,
// same-toolchain bit identity.
constexpr double kEarthRadiusMeters = 6371008.8;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] double to_radians(double degrees) noexcept {
    return degrees * kPi / 180.0;
}

// Great-circle distance in meters (the repository's shared haversine).
[[nodiscard]] double haversine_meters(map::Wgs84Coordinate from,
                                      map::Wgs84Coordinate to) noexcept {
    const double lat1 = to_radians(from.latitude_deg);
    const double lat2 = to_radians(to.latitude_deg);
    const double d_lat = lat2 - lat1;
    const double d_lon = to_radians(to.longitude_deg - from.longitude_deg);
    const double a = std::sin(d_lat / 2.0) * std::sin(d_lat / 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::sin(d_lon / 2.0) *
                         std::sin(d_lon / 2.0);
    return 2.0 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(a)));
}

// Initial bearing of the great-circle path from -> to, radians clockwise
// from north (the heading convention of ADR-016); callers normalize.
[[nodiscard]] double initial_bearing(map::Wgs84Coordinate from,
                                     map::Wgs84Coordinate to) noexcept {
    const double lat1 = to_radians(from.latitude_deg);
    const double lat2 = to_radians(to.latitude_deg);
    const double d_lon = to_radians(to.longitude_deg - from.longitude_deg);
    const double y = std::sin(d_lon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
                     std::sin(lat1) * std::cos(lat2) * std::cos(d_lon);
    return std::atan2(y, x);
}

}  // namespace

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

void LocalizationTracker::validate_fix(const LocalizationEstimate& fix) {
    if (!std::isfinite(fix.position.latitude_deg) ||
        !std::isfinite(fix.position.longitude_deg) || !std::isfinite(fix.heading_rad)) {
        throw std::invalid_argument(
            "LocalizationTracker: non-finite fix rejected (position or heading)");
    }
}

FixApplication LocalizationTracker::apply_sample(
    const std::optional<LocalizationEstimate>& sample) {
    if (!sample.has_value()) {
        // No fix this sample: the previous estimate is retained exactly
        // as-is (estimated_at unchanged) — staleness grows around it. The
        // miss is only counted: it defines when the NEXT fix becomes a
        // reacquisition (ADR-021) and never changes behavior.
        ++missed_samples_;
        return FixApplication{};  // fix = false
    }
    validate_fix(*sample);

    // Transition observability (#18, ADR-021), measured BEFORE the fix
    // replaces belief. The caller is expected to have propagated the prior
    // up to the fix's represented time (Robot::apply_gnss_sample does), so
    // the correction is the honest "belief jump" of the replace step.
    FixApplication outcome;
    outcome.fix = true;
    outcome.reacquisition = missed_samples_ > 0 && estimate_.has_value();
    outcome.prior_dead_reckoned = dead_reckoned_;
    if (last_fix_at_.has_value() && sample->estimated_at >= *last_fix_at_) {
        outcome.since_last_fix_ms =
            sample->estimated_at.value - last_fix_at_->value;
    }
    if (estimate_.has_value()) {
        outcome.correction_distance_m =
            haversine_meters(estimate_->position, sample->position);
        if (*outcome.correction_distance_m > 0.0) {
            outcome.correction_heading_rad = normalize_heading(
                initial_bearing(estimate_->position, sample->position));
        }
    }

    estimate_ = *sample;
    last_fix_at_ = sample->estimated_at;
    dead_reckoned_ = false;
    missed_samples_ = 0;
    return outcome;
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

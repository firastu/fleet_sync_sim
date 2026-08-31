#include "fleet/localization/gnss_model.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace fleet::localization {

double normalize_heading(double heading_rad) {
    if (!std::isfinite(heading_rad)) {
        throw std::invalid_argument("localization: heading must be finite");
    }
    const double two_pi = 2.0 * std::numbers::pi;
    double normalized = std::fmod(heading_rad, two_pi);
    if (normalized < 0.0) {
        normalized += two_pi;
    }
    return normalized;
}

std::optional<LocalizationEstimate> PerfectGnss::measure(
    const GroundTruthPose& truth, simulation::DeterministicRng& rng) const {
    (void)rng;  // zero-noise baseline: no randomness consumed
    return LocalizationEstimate{truth.position, truth.heading_rad, truth.at};
}

std::optional<LocalizationEstimate> UnavailableGnss::measure(
    const GroundTruthPose& truth, simulation::DeterministicRng& rng) const {
    (void)truth;
    (void)rng;
    return std::nullopt;
}

}  // namespace fleet::localization

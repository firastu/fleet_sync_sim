#include "fleet/localization/gnss_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "fleet/localization/noise.hpp"

namespace fleet::localization {

namespace {

// Mean earth radius in meters — the same constant the OSM importer uses
// for haversine costs (ADR-014); one physical constant across the
// project.
constexpr double kEarthRadiusMeters = 6371008.8;

// Irwin-Hall / CLT noise (ADR-017): the shared deterministic sample of
// fleet/localization/noise.hpp — 12 midpoint-centered finite-resolution
// uniforms, variance 1 - 2^-64, no transcendental and no
// std::normal_distribution in the sampling path. Scaling by sigma happens
// here, after the sum, exactly as before the sampler was shared.
[[nodiscard]] double sigma_scaled_noise(simulation::DeterministicRng& rng, double sigma) {
    return irwin_hall_standard_noise(rng) * sigma;
}

}  // namespace

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

map::Wgs84Coordinate apply_en_displacement(map::Wgs84Coordinate position, double east_m,
                                           double north_m) {
    if (!std::isfinite(east_m) || !std::isfinite(north_m)) {
        throw std::invalid_argument("localization: displacement must be finite");
    }
    // Supported domain of the local-tangent engineering approximation:
    // the east conversion divides by cos(latitude), which degenerates
    // toward the poles. East displacements outside this domain fail
    // explicitly — never silently clamped, never a manufactured
    // position (the ADR-014 importer philosophy).
    constexpr double kSupportedLatitudeLimit_deg = 89.0;
    if (east_m != 0.0 && std::abs(position.latitude_deg) > kSupportedLatitudeLimit_deg) {
        throw std::invalid_argument(std::format(
            "localization: east displacement at latitude {:.6f} is outside the "
            "local-tangent approximation's supported domain (|latitude| <= {})",
            position.latitude_deg, kSupportedLatitudeLimit_deg));
    }

    const double meters_per_degree_latitude =
        kEarthRadiusMeters * std::numbers::pi / 180.0;
    const double new_latitude =
        position.latitude_deg + north_m / meters_per_degree_latitude;
    if (new_latitude > 90.0 || new_latitude < -90.0) {
        throw std::invalid_argument(std::format(
            "localization: north displacement of {:.3f} m at latitude {:.6f} crosses a "
            "pole (unsupported)",
            north_m, position.latitude_deg));
    }

    const double latitude_radians = position.latitude_deg * std::numbers::pi / 180.0;
    const double meters_per_degree_longitude =
        meters_per_degree_latitude * std::cos(latitude_radians);
    double new_longitude =
        position.longitude_deg + east_m / meters_per_degree_longitude;
    // Wrap across the antimeridian into the canonical [-180, +180):
    // continuing east past +180 emerges at -180 (and mirrored westward).
    // NEVER clamp — clamping would destroy the requested displacement.
    new_longitude = std::fmod(new_longitude + 180.0, 360.0);
    if (new_longitude < 0.0) {
        new_longitude += 360.0;
    }
    new_longitude -= 180.0;

    return map::Wgs84Coordinate{new_latitude, new_longitude};
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

NoisyGnss::NoisyGnss(GnssNoiseConfig config) : config_{config} {
    const auto require_valid = [](double value, const char* name) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(
                std::format("localization: {} must be finite and non-negative", name));
        }
    };
    require_valid(config_.position_axis_sigma_m, "position_axis_sigma_m");
    require_valid(config_.heading_sigma_rad, "heading_sigma_rad");
}

std::optional<LocalizationEstimate> NoisyGnss::measure(
    const GroundTruthPose& truth, simulation::DeterministicRng& rng) const {
    map::Wgs84Coordinate position = truth.position;
    if (config_.position_axis_sigma_m > 0.0) {
        // Consumption order is part of the contract: east, then north
        // (12 draws each), only when the knob is non-zero.
        const double east_m = sigma_scaled_noise(rng, config_.position_axis_sigma_m);
        const double north_m = sigma_scaled_noise(rng, config_.position_axis_sigma_m);
        position = apply_en_displacement(truth.position, east_m, north_m);
    }
    double heading_rad = truth.heading_rad;
    if (config_.heading_sigma_rad > 0.0) {
        heading_rad =
            normalize_heading(truth.heading_rad + sigma_scaled_noise(rng, config_.heading_sigma_rad));
    }
    return LocalizationEstimate{position, heading_rad, truth.at};
}

}  // namespace fleet::localization

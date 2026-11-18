#include "fleet/world/terrain_motion.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include "fleet/localization/pose.hpp"

namespace fleet::world {

namespace {

// Spherical earth, the same model as localization::apply_en_displacement.
constexpr double kEarthRadiusMeters = 6371008.8;
constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

[[nodiscard]] bool is_valid_coordinate(const map::Wgs84Coordinate& position) {
    return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
           position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0 &&
           position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0;
}

[[nodiscard]] double haversine_m(const map::Wgs84Coordinate& a, const map::Wgs84Coordinate& b) {
    const double lat_a = a.latitude_deg * kDegToRad;
    const double lat_b = b.latitude_deg * kDegToRad;
    const double half_dlat = (b.latitude_deg - a.latitude_deg) * kDegToRad / 2.0;
    const double half_dlon = (b.longitude_deg - a.longitude_deg) * kDegToRad / 2.0;
    const double h = std::sin(half_dlat) * std::sin(half_dlat) +
                     std::cos(lat_a) * std::cos(lat_b) * std::sin(half_dlon) * std::sin(half_dlon);
    return 2.0 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(h)));
}

// Initial great-circle bearing from a to b, clockwise from north, [0, 2*pi).
[[nodiscard]] double initial_bearing_rad(const map::Wgs84Coordinate& a,
                                         const map::Wgs84Coordinate& b) {
    const double lat_a = a.latitude_deg * kDegToRad;
    const double lat_b = b.latitude_deg * kDegToRad;
    const double dlon = (b.longitude_deg - a.longitude_deg) * kDegToRad;
    const double y = std::sin(dlon) * std::cos(lat_b);
    const double x =
        std::cos(lat_a) * std::sin(lat_b) - std::sin(lat_a) * std::cos(lat_b) * std::cos(dlon);
    return localization::normalize_heading(std::atan2(y, x));
}

[[nodiscard]] std::string describe(const map::Wgs84Coordinate& position) {
    return std::format("({:.6f}, {:.6f})", position.latitude_deg, position.longitude_deg);
}

// Truth terrain directly below `position`, or an explicit failure: the
// body is never placed on a manufactured height (ADR-022).
template <typename Error>
[[nodiscard]] double terrain_below(const map::ElevationGrid& terrain,
                                   const map::Wgs84Coordinate& position) {
    const map::ElevationLookup lookup = terrain.height_at(position);
    switch (lookup.status) {
        case map::ElevationStatus::Ok:
            return *lookup.height_m;
        case map::ElevationStatus::OutsideCoverage:
            throw Error(std::format("terrain motion: {} is outside the truth terrain coverage",
                                    describe(position)));
        case map::ElevationStatus::NoData:
            break;
    }
    throw Error(
        std::format("terrain motion: the truth terrain has no data at {}", describe(position)));
}

template <typename Error>
[[nodiscard]] double body_altitude(const TerrainMotionConfig& config, double terrain_height_m,
                                   const map::Wgs84Coordinate& position) {
    if (config.platform == TerrainPlatform::Ground) {
        return terrain_height_m + config.mount_height_m;
    }
    if (!(config.altitude_m > terrain_height_m)) {
        throw Error(std::format(
            "terrain motion: air body at altitude {:.2f} m is not above the truth terrain "
            "({:.2f} m) at {}",
            config.altitude_m, terrain_height_m, describe(position)));
    }
    return config.altitude_m;
}

// Consumes consecutive waypoints within the arrival radius.
void reach_waypoints(TerrainBodyState& state, const std::vector<map::Wgs84Coordinate>& waypoints,
                     double arrival_radius_m, TerrainStepOutcome& outcome) {
    while (state.next_waypoint < waypoints.size() &&
           haversine_m(state.position, waypoints[state.next_waypoint]) <= arrival_radius_m) {
        outcome.reached_waypoints.push_back(state.next_waypoint);
        ++state.next_waypoint;
        if (state.next_waypoint == waypoints.size()) {
            outcome.mission_completed = true;
        }
    }
}

}  // namespace

void TerrainMotionConfig::validate() const {
    if (!std::isfinite(speed_mps) || !(speed_mps > 0.0)) {
        throw std::invalid_argument("terrain motion: speed_mps must be positive and finite");
    }
    if (!std::isfinite(max_turn_rate_rad_per_s) || !(max_turn_rate_rad_per_s > 0.0)) {
        throw std::invalid_argument(
            "terrain motion: max_turn_rate must be positive and finite");
    }
    if (!std::isfinite(arrival_radius_m) || !(arrival_radius_m > 0.0)) {
        throw std::invalid_argument(
            "terrain motion: arrival_radius_m must be positive and finite");
    }
    if (arrival_radius_m < turning_diameter_m()) {
        throw std::invalid_argument(std::format(
            "terrain motion: arrival_radius_m ({:.3f}) must be at least the turning diameter "
            "2 * speed / turn rate ({:.3f})",
            arrival_radius_m, turning_diameter_m()));
    }
    if (!std::isfinite(mount_height_m) || !std::isfinite(altitude_m)) {
        throw std::invalid_argument(
            "terrain motion: mount_height_m and altitude_m must be finite");
    }
    if (platform == TerrainPlatform::Ground) {
        if (mount_height_m < 0.0) {
            throw std::invalid_argument("terrain motion: mount_height_m must be >= 0");
        }
        if (altitude_m != 0.0) {
            throw std::invalid_argument(
                "terrain motion: a ground body follows the terrain; altitude_m does not apply");
        }
    } else if (mount_height_m != 0.0) {
        throw std::invalid_argument(
            "terrain motion: an air body flies at altitude_m; mount_height_m does not apply");
    }
}

TerrainMotion::TerrainMotion(TerrainMotionConfig config,
                             std::vector<map::Wgs84Coordinate> waypoints,
                             std::shared_ptr<const map::ElevationGrid> truth_terrain,
                             map::Wgs84Coordinate start, double start_heading_rad,
                             common::Tick start_at)
    : config_{config}, waypoints_{std::move(waypoints)}, truth_terrain_{std::move(truth_terrain)} {
    config_.validate();
    if (truth_terrain_ == nullptr) {
        throw std::invalid_argument("terrain motion: truth terrain is required");
    }
    if (waypoints_.empty()) {
        throw std::invalid_argument("terrain motion: at least one waypoint is required");
    }
    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
        if (!is_valid_coordinate(waypoints_[index])) {
            throw std::invalid_argument(std::format(
                "terrain motion: waypoint {} must be finite and within WGS84 bounds", index));
        }
    }
    if (!is_valid_coordinate(start)) {
        throw std::invalid_argument(
            "terrain motion: start must be finite and within WGS84 bounds");
    }
    if (!std::isfinite(start_heading_rad)) {
        throw std::invalid_argument("terrain motion: start heading must be finite");
    }

    state_.position = start;
    state_.heading_rad = localization::normalize_heading(start_heading_rad);
    state_.terrain_height_m = terrain_below<std::invalid_argument>(*truth_terrain_, start);
    state_.altitude_m =
        body_altitude<std::invalid_argument>(config_, state_.terrain_height_m, start);
    state_.next_waypoint = 0;
    state_.at = start_at;
}

void TerrainMotion::validate_step(const TerrainMotionConfig& config, std::uint64_t step_ms) {
    if (step_ms == 0) {
        throw std::invalid_argument("terrain motion: step_ms must be >= 1");
    }
    const double distance_m = config.speed_mps * (static_cast<double>(step_ms) / 1000.0);
    if (distance_m > kMaxTerrainStepDisplacementM) {
        throw std::invalid_argument(std::format(
            "terrain motion: a step moves {:.3f} m, more than the {:.1f} m step bound",
            distance_m, kMaxTerrainStepDisplacementM));
    }
    if (distance_m > config.arrival_radius_m) {
        throw std::invalid_argument(std::format(
            "terrain motion: a step moves {:.3f} m, more than arrival_radius_m ({:.3f})",
            distance_m, config.arrival_radius_m));
    }
}

TerrainStepOutcome TerrainMotion::step(std::uint64_t step_ms) {
    validate_step(config_, step_ms);
    const double dt_s = static_cast<double>(step_ms) / 1000.0;
    const double distance_m = config_.speed_mps * dt_s;

    // Work on a copy; commit only when the whole step succeeded.
    TerrainBodyState next = state_;
    next.at = state_.at + step_ms;
    TerrainStepOutcome outcome;

    reach_waypoints(next, waypoints_, config_.arrival_radius_m, outcome);
    if (next.next_waypoint < waypoints_.size()) {
        // Turn toward the next waypoint by at most the turn budget. The
        // difference is wrapped into (-pi, pi]: exactly behind turns
        // clockwise.
        double delta = initial_bearing_rad(next.position, waypoints_[next.next_waypoint]) -
                       next.heading_rad;
        if (delta > std::numbers::pi) {
            delta -= kTwoPi;
        } else if (delta <= -std::numbers::pi) {
            delta += kTwoPi;
        }
        const double max_turn = config_.max_turn_rate_rad_per_s * dt_s;
        const double turn = std::clamp(delta, -max_turn, max_turn);
        next.heading_rad = localization::normalize_heading(next.heading_rad + turn);
        outcome.turn_rad = turn;
        outcome.distance_m = distance_m;

        next.position = localization::apply_en_displacement(
            next.position, distance_m * std::sin(next.heading_rad),
            distance_m * std::cos(next.heading_rad));
        next.terrain_height_m = terrain_below<std::runtime_error>(*truth_terrain_, next.position);
        next.altitude_m =
            body_altitude<std::runtime_error>(config_, next.terrain_height_m, next.position);

        reach_waypoints(next, waypoints_, config_.arrival_radius_m, outcome);
    }

    state_ = next;
    return outcome;
}

}  // namespace fleet::world

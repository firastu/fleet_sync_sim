#include "fleet/world/truth_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <vector>

#include "fleet/localization/pose.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::world {

namespace {

// Mean earth radius in meters — the same physical constant the OSM
// importer (ADR-014) and the localization geodetic seam (ADR-017) use.
constexpr double kEarthRadiusMeters = 6371008.8;

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] double to_radians(double degrees) noexcept {
    return degrees * kPi / 180.0;
}

// Great-circle distance in meters (the ADR-014 importer's haversine).
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
// from north — the heading convention of ADR-016. atan2 range (-pi, pi];
// callers normalize.
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

localization::GroundTruthPose truth_pose(const map::BaseMap& base,
                                         const robot::RobotState& state,
                                         double at_rest_heading_rad,
                                         common::Tick now) {
    const map::MapGeometry* const geometry = base.geometry();
    if (geometry == nullptr) {
        throw std::invalid_argument(
            "truth_pose: the map carries no geographic geometry");
    }

    if (!state.in_transit.has_value()) {
        const map::Wgs84Coordinate* const position =
            geometry->node_position(state.position);
        if (position == nullptr) {
            throw std::invalid_argument(std::format(
                "truth_pose: node {} has no map geometry", state.position.value()));
        }
        // At-rest orientation contract (see header): the caller-held
        // physical orientation — real modeled truth (initial condition +
        // kinematic maintenance at arrivals), never a stationary reset.
        // normalize_heading validates finiteness (throws on NaN/inf).
        return localization::GroundTruthPose{
            *position, localization::normalize_heading(at_rest_heading_rad), now};
    }

    const robot::RobotTransit& transit = *state.in_transit;

    // Traversal timing fraction — from the movement timing contract
    // (ADR-010), not from planning cost: cost is not meters.
    const std::uint64_t total_ticks =
        transit.arrival.value - transit.departed_at.value;
    std::uint64_t elapsed_ticks = now.value - transit.departed_at.value;
    if (transit.arrival < transit.departed_at) {
        throw std::invalid_argument(
            "truth_pose: transit arrival precedes departure");
    }

    if (now < transit.departed_at) {
        throw std::invalid_argument(
            "truth_pose: requested tick precedes transit departure");
    }
    if (total_ticks == 0) {
        elapsed_ticks = 0;  // defensive: begin_transit guarantees >= 1
    }
    double fraction = total_ticks == 0
                          ? 1.0
                          : static_cast<double>(elapsed_ticks) /
                                static_cast<double>(total_ticks);
    fraction = std::clamp(fraction, 0.0, 1.0);

    // The canonical polyline runs a->b (ADR-012). Physical motion runs
    // from->to; a transit departing from the edge's `b` walks the same
    // geometry mirrored (Bidirectional reverse use, or EdgeDirection
    // Reverse — ADR-013).
    const map::Edge& edge = base.graph().edge(transit.edge);
    const bool forward = transit.from == edge.a;
    if (!forward && transit.from != edge.b) {
        throw std::invalid_argument(std::format(
            "truth_pose: transit endpoints do not match edge {}", edge.id.value()));
    }

    std::vector<map::Wgs84Coordinate> points;
    if (const std::vector<map::Wgs84Coordinate>* polyline =
            geometry->edge_polyline(transit.edge);
        polyline != nullptr) {
        points = *polyline;
    } else {
        // Straight-segment fallback: the documented geometry-free
        // rendering rule (ADR-012) applied to pose.
        const map::Wgs84Coordinate* const a = geometry->node_position(edge.a);
        const map::Wgs84Coordinate* const b = geometry->node_position(edge.b);
        if (a == nullptr || b == nullptr) {
            throw std::invalid_argument(std::format(
                "truth_pose: edge {} has no polyline and an endpoint node "
                "lacks geometry",
                edge.id.value()));
        }
        points = {*a, *b};
    }
    if (!forward) {
        std::reverse(points.begin(), points.end());
    }

    // Walk cumulative physical length; segments own [start, end), the
    // final segment additionally owns its end (vertex rule, see header).
    std::vector<double> cumulative{0.0};
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative.push_back(cumulative.back() +
                             haversine_meters(points[i - 1], points[i]));
    }
    const double total_length = cumulative.back();

    // Degenerate geometry: no physical travel bearing can be derived.
    // Preserve the caller-held simulation-truth orientation.
    if (total_length <= 0.0) {
        return localization::GroundTruthPose{
            points.front(),
            localization::normalize_heading(at_rest_heading_rad),
            now};
    }

    const double target = fraction * total_length;
    // Default: the FINAL segment (index size-2 of size points) — it owns
    // its end point, which is where target == total_length (an exact
    // arrival tick) lands. The loop below narrows to an earlier segment
    // whenever target is strictly inside one.
    std::size_t segment = points.size() - 2;
    for (std::size_t i = 1; i < cumulative.size(); ++i) {
        if (target < cumulative[i]) {
            segment = i - 1;
            break;
        }
    }

    const map::Wgs84Coordinate from_point = points[segment];
    const map::Wgs84Coordinate to_point = points[segment + 1];
    const double segment_length = cumulative[segment + 1] - cumulative[segment];
    double within = segment_length <= 0.0
                        ? 0.0
                        : (target - cumulative[segment]) / segment_length;
    within = std::clamp(within, 0.0, 1.0);

    // Snap the endpoints exactly: at progress 0/1 the truth position IS
    // the canonical node coordinate — bit-exact, not a rounded lerp
    // (departure and arrival poses must equal the node's stored value).
    map::Wgs84Coordinate position;
    if (within >= 1.0) {
        position = to_point;
    } else if (within <= 0.0) {
        position = from_point;
    } else {
        position = map::Wgs84Coordinate{
            from_point.latitude_deg +
                within * (to_point.latitude_deg - from_point.latitude_deg),
            from_point.longitude_deg +
                within * (to_point.longitude_deg - from_point.longitude_deg)};
    }
    const double heading =
        localization::normalize_heading(initial_bearing(from_point, to_point));

    return localization::GroundTruthPose{position, heading, now};
}

double localization_position_error_m(const localization::GroundTruthPose& truth,
                                      const localization::LocalizationEstimate& estimate) {
    return haversine_meters(truth.position, estimate.position);
}

}  // namespace fleet::world

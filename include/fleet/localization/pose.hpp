#pragma once

#include <compare>

#include "fleet/common/time.hpp"
#include "fleet/map/geometry.hpp"

namespace fleet::localization {

// WHERE THE ROBOT ACTUALLY IS (ADR-016). Simulation-side truth, like
// World's edge states: robots never read it directly — it reaches a
// LocalizationEstimate only through a sensor model. Derived by the
// simulation from RobotState + MapGeometry (node/transit positions on
// the geographic side); carried as its own type so truth and estimate
// can never be conflated.
//
// heading_rad is radians clockwise from north in the WGS84 frame
// (0 = north, pi/2 = east); see normalize_heading.
struct GroundTruthPose {
    map::Wgs84Coordinate position;
    double heading_rad = 0.0;
    common::Tick at{};

    constexpr auto operator<=>(const GroundTruthPose&) const noexcept = default;
};

// WHERE A ROBOT BELIEVES IT IS (ADR-016): the output of a sensor
// model. Deliberately carries NO uncertainty representation yet — the
// failure modes that would justify one (noise, outage, drift, staleness)
// become observable first, per the M3 plan. estimated_at is the tick
// the estimate refers to (not when it was computed): a stale estimate
// is a first-class state.
struct LocalizationEstimate {
    map::Wgs84Coordinate position;
    double heading_rad = 0.0;
    common::Tick estimated_at{};

    constexpr auto operator<=>(const LocalizationEstimate&) const noexcept = default;
};

// Normalizes a heading to [0, 2*pi). Deterministic, allocation-free.
//
// Contract: the heading must be finite — non-finite values (NaN, +/-inf)
// are rejected with std::invalid_argument rather than silently becoming
// localization state.
[[nodiscard]] double normalize_heading(double heading_rad);

// Applies a small East/North displacement in METERS to a WGS84 position
// using a local tangent-plane approximation anchored AT that position
// (no scenario-global anchor — independent fixes anchor independently;
// valid for meter-to-tens-of-meters displacements; ADR-017). This is
// the single metric<->WGS84 seam of the localization module: the same
// conversion serves noisy GNSS now and dead reckoning later.
//
// Failure domain is EXPLICIT, never silently clamped:
//   - longitude WRAPS across the antimeridian into [-180, +180):
//     continuing east past +180 emerges at -180 (and mirrored westward);
//   - east displacements at |latitude| > 89 deg are outside the
//     approximation's supported domain and throw std::invalid_argument;
//   - north displacements that would cross a pole throw
//     std::invalid_argument;
//   - non-finite displacements throw std::invalid_argument.
[[nodiscard]] map::Wgs84Coordinate apply_en_displacement(
    map::Wgs84Coordinate position, double east_m, double north_m);

}  // namespace fleet::localization

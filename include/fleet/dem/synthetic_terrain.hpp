#pragma once

#include <vector>

#include "fleet/map/elevation_grid.hpp"
#include "fleet/map/geometry.hpp"

namespace fleet::dem {

// One Gaussian terrain feature: a hill, or a depression when height_m is
// negative (ADR-023).
struct SyntheticHill {
    map::Wgs84Coordinate center;
    double height_m = 0.0;  // finite; negative = depression
    double radius_m = 0.0;  // > 0: the Gaussian standard deviation in meters
};

// Declarative synthetic terrain: an analytic surface sampled onto a lattice.
// Scenario fixtures and the M4 test suite use it so that rough, flat and
// repeating terrain can be described in a few lines without committing DEM
// tiles.
struct SyntheticTerrainSpec {
    map::ElevationGridLayout layout;
    double base_height_m = 0.0;
    std::vector<SyntheticHill> hills;
    map::ElevationSurface surface = map::ElevationSurface::BareEarth;
};

// Samples the analytic surface onto spec.layout (ADR-023):
//
//   h(p) = base_height_m + sum_i height_i * exp(-d_i^2 / (2 * radius_i^2))
//
// where d_i is the haversine distance in meters (mean earth radius
// 6371008.8 m) from sample position p to hill i's centre. Hills are summed in
// declaration order. Sample positions follow the lattice contract of
// ElevationGridLayout; the outermost rows and columns lie exactly on the
// layout corners. The result is labelled VerticalDatum::Synthetic.
//
// Throws std::invalid_argument for a non-finite base height, a hill with a
// non-finite height, a non-positive or non-finite radius or an invalid
// centre, for any layout ElevationGrid rejects, and for a sampled height
// that does not fit a finite float.
//
// Deterministic for a given toolchain (std::exp and the haversine use
// ordinary libm arithmetic, the ADR-014/017 reproducibility class).
[[nodiscard]] map::ElevationGrid synthesize_terrain(const SyntheticTerrainSpec& spec);

}  // namespace fleet::dem

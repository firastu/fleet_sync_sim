#include "fleet/dem/synthetic_terrain.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "fleet/map/elevation_grid.hpp"

namespace {

using fleet::dem::SyntheticHill;
using fleet::dem::SyntheticTerrainSpec;
using fleet::dem::synthesize_terrain;
using fleet::map::ElevationGrid;
using fleet::map::ElevationGridLayout;
using fleet::map::ElevationSurface;
using fleet::map::VerticalDatum;
using fleet::map::Wgs84Coordinate;

// 3 x 3 lattice with exactly representable positions: latitudes 1.0 / 0.5 /
// 0.0, longitudes 10.0 / 10.5 / 11.0. The centre sample is (0.5, 10.5).
constexpr ElevationGridLayout kLayout{Wgs84Coordinate{1.0, 10.0}, Wgs84Coordinate{0.0, 11.0},
                                      3, 3};
constexpr Wgs84Coordinate kCentre{0.5, 10.5};

[[nodiscard]] SyntheticTerrainSpec spec_with(std::vector<SyntheticHill> hills,
                                             double base = 100.0) {
    return SyntheticTerrainSpec{kLayout, base, std::move(hills), ElevationSurface::BareEarth};
}

TEST(SyntheticTerrainTest, FlatTerrainIsTheBaseHeightEverywhere) {
    SyntheticTerrainSpec spec = spec_with({}, 123.5);
    spec.surface = ElevationSurface::SurfaceModel;
    const ElevationGrid grid = synthesize_terrain(spec);
    EXPECT_EQ(grid.layout(), kLayout);
    EXPECT_EQ(grid.surface(), ElevationSurface::SurfaceModel);
    EXPECT_EQ(grid.vertical_datum(), VerticalDatum::Synthetic);
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            EXPECT_EQ(grid.sample_m(row, column), 123.5);
        }
    }
}

TEST(SyntheticTerrainTest, HillPeaksAtItsCentreAndFallsOffAsAGaussian) {
    constexpr double kRadius = 50'000.0;
    const ElevationGrid grid = synthesize_terrain(spec_with({{kCentre, 100.0, kRadius}}));
    EXPECT_EQ(grid.sample_m(1, 1), 200.0);  // distance 0: the full height

    // North neighbour: a pure 0.5 degree latitude offset, so the haversine
    // distance is exactly R * 0.5 degrees in radians.
    const double distance = 6371008.8 * (0.5 * std::numbers::pi / 180.0);
    const double expected = 100.0 + 100.0 * std::exp(-(distance * distance) /
                                                     (2.0 * kRadius * kRadius));
    EXPECT_NEAR(grid.sample_m(0, 1).value(), expected, 1e-3);

    // Symmetric neighbours are bit-identical; farther samples are lower.
    EXPECT_EQ(grid.sample_m(0, 1), grid.sample_m(2, 1));
    EXPECT_EQ(grid.sample_m(1, 0), grid.sample_m(1, 2));
    EXPECT_LT(grid.sample_m(0, 0).value(), grid.sample_m(0, 1).value());
}

TEST(SyntheticTerrainTest, NegativeHeightIsADepressionAndHillsSum) {
    EXPECT_EQ(synthesize_terrain(spec_with({{kCentre, -40.0, 1000.0}})).sample_m(1, 1), 60.0);
    EXPECT_EQ(synthesize_terrain(spec_with({{kCentre, 30.0, 1000.0}, {kCentre, 20.0, 5000.0}}))
                  .sample_m(1, 1),
              150.0);
}

TEST(SyntheticTerrainTest, IsDeterministic) {
    const SyntheticTerrainSpec spec =
        spec_with({{kCentre, 75.0, 40'000.0}, {Wgs84Coordinate{0.8, 10.2}, -12.5, 30'000.0}});
    const ElevationGrid first = synthesize_terrain(spec);
    const ElevationGrid second = synthesize_terrain(spec);
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            EXPECT_EQ(first.sample_m(row, column), second.sample_m(row, column));
        }
    }
}

TEST(SyntheticTerrainTest, RejectsInvalidSpecs) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW((void)synthesize_terrain(spec_with({}, nan)), std::invalid_argument);
    EXPECT_THROW((void)synthesize_terrain(spec_with({{kCentre, nan, 1000.0}})),
                 std::invalid_argument);
    EXPECT_THROW((void)synthesize_terrain(spec_with({{kCentre, 10.0, 0.0}})),
                 std::invalid_argument);
    EXPECT_THROW((void)synthesize_terrain(spec_with({{kCentre, 10.0, -5.0}})),
                 std::invalid_argument);
    EXPECT_THROW((void)synthesize_terrain(spec_with({{kCentre, 10.0, inf}})),
                 std::invalid_argument);
    EXPECT_THROW((void)synthesize_terrain(spec_with({{Wgs84Coordinate{91.0, 10.5}, 10.0, 1.0}})),
                 std::invalid_argument);
    // Heights must fit a finite float.
    EXPECT_THROW((void)synthesize_terrain(spec_with({}, 1e39)), std::invalid_argument);

    SyntheticTerrainSpec bad_layout = spec_with({});
    bad_layout.layout.rows = 1;
    EXPECT_THROW((void)synthesize_terrain(bad_layout), std::invalid_argument);
    bad_layout = spec_with({});
    bad_layout.layout.north_west.latitude_deg = nan;
    EXPECT_THROW((void)synthesize_terrain(bad_layout), std::invalid_argument);
}

}  // namespace

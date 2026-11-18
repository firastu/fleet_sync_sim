#include "fleet/map/elevation_grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using fleet::map::ElevationGrid;
using fleet::map::ElevationGridLayout;
using fleet::map::ElevationLookup;
using fleet::map::ElevationStatus;
using fleet::map::ElevationSurface;
using fleet::map::VerticalDatum;
using fleet::map::Wgs84Coordinate;

constexpr float kNoData = std::numeric_limits<float>::quiet_NaN();

// 3 x 3 lattice whose sample positions are exactly representable:
// latitudes 1.0 / 0.5 / 0.0, longitudes 10.0 / 10.5 / 11.0.
constexpr ElevationGridLayout kLayout3x3{Wgs84Coordinate{1.0, 10.0}, Wgs84Coordinate{0.0, 11.0},
                                         3, 3};

// Planar heights h = 30 * row + 10 * column (bilinear reproduces a plane).
[[nodiscard]] std::vector<float> planar_heights() {
    return {0.0F,  10.0F, 20.0F,   //
            30.0F, 40.0F, 50.0F,   //
            60.0F, 70.0F, 80.0F};
}

[[nodiscard]] ElevationGrid planar_grid() {
    return ElevationGrid{kLayout3x3, planar_heights(), ElevationSurface::SurfaceModel,
                         VerticalDatum::Egm96};
}

[[nodiscard]] ElevationLookup ok(double height_m) {
    return ElevationLookup{ElevationStatus::Ok, height_m};
}

const ElevationLookup kOutside{ElevationStatus::OutsideCoverage, std::nullopt};
const ElevationLookup kNoDataLookup{ElevationStatus::NoData, std::nullopt};

TEST(ElevationGridTest, ExposesLayoutSurfaceAndDatum) {
    const ElevationGrid grid{kLayout3x3, planar_heights(), ElevationSurface::BareEarth,
                             VerticalDatum::Egm96};
    EXPECT_EQ(grid.layout(), kLayout3x3);
    EXPECT_EQ(grid.surface(), ElevationSurface::BareEarth);
    EXPECT_EQ(grid.vertical_datum(), VerticalDatum::Egm96);
    EXPECT_EQ(grid.sample_m(0, 0), 0.0);
    EXPECT_EQ(grid.sample_m(1, 2), 50.0);
    EXPECT_EQ(grid.sample_m(2, 1), 70.0);
}

TEST(ElevationGridTest, LookupExactlyOnASampleReturnsThatSample) {
    const ElevationGrid grid = planar_grid();
    const double latitudes[] = {1.0, 0.5, 0.0};
    const double longitudes[] = {10.0, 10.5, 11.0};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            EXPECT_EQ(grid.height_at({latitudes[row], longitudes[column]}),
                      ok(30.0 * static_cast<double>(row) + 10.0 * static_cast<double>(column)))
                << "row " << row << " column " << column;
        }
    }
}

TEST(ElevationGridTest, InterpolatesBilinearlyInsideACell) {
    const ElevationGrid grid = planar_grid();
    // Cell centre: the mean of the four corners (0, 10, 30, 40).
    EXPECT_EQ(grid.height_at({0.75, 10.25}), ok(20.0));
    // Off-centre: the plane is reproduced (lattice row 0.2, column 0.2).
    const ElevationLookup off_centre = grid.height_at({0.9, 10.1});
    ASSERT_EQ(off_centre.status, ElevationStatus::Ok);
    EXPECT_NEAR(off_centre.height_m.value(), 30.0 * 0.2 + 10.0 * 0.2, 1e-9);
}

TEST(ElevationGridTest, InterpolationIsBilinearNotPlanar) {
    // Only the south-east sample is raised: a triangulated (planar)
    // interpolation would give a different value than the bilinear product.
    const ElevationGrid grid{
        ElevationGridLayout{Wgs84Coordinate{2.0, 0.0}, Wgs84Coordinate{0.0, 2.0}, 2, 2},
        {0.0F, 0.0F, 0.0F, 100.0F}, ElevationSurface::SurfaceModel, VerticalDatum::Egm96};
    EXPECT_EQ(grid.height_at({1.0, 1.0}), ok(25.0));    // weights 0.5 * 0.5
    EXPECT_EQ(grid.height_at({0.5, 1.5}), ok(56.25));   // weights 0.75 * 0.75
    EXPECT_EQ(grid.height_at({1.5, 0.5}), ok(6.25));    // weights 0.25 * 0.25
}

TEST(ElevationGridTest, LookupOnACellEdgeUsesOnlyThatEdge) {
    const ElevationGrid grid = planar_grid();
    EXPECT_EQ(grid.height_at({1.0, 10.25}), ok(5.0));   // north edge: 0 .. 10
    EXPECT_EQ(grid.height_at({0.0, 10.75}), ok(75.0));  // south edge: 70 .. 80
    EXPECT_EQ(grid.height_at({0.25, 11.0}), ok(65.0));  // east edge: 50 .. 80
    EXPECT_EQ(grid.height_at({0.75, 10.0}), ok(15.0));  // west edge: 0 .. 30
}

TEST(ElevationGridTest, CoverageIsTheClosedBoxAndOutsideIsNeverClamped) {
    const ElevationGrid grid = planar_grid();
    EXPECT_EQ(grid.height_at({1.0, 10.0}), ok(0.0));
    EXPECT_EQ(grid.height_at({1.0, 11.0}), ok(20.0));
    EXPECT_EQ(grid.height_at({0.0, 10.0}), ok(60.0));
    EXPECT_EQ(grid.height_at({0.0, 11.0}), ok(80.0));

    // One ulp beyond each edge is outside: no clamping to the nearest edge.
    EXPECT_EQ(grid.height_at({std::nextafter(1.0, 2.0), 10.5}), kOutside);
    EXPECT_EQ(grid.height_at({std::nextafter(0.0, -1.0), 10.5}), kOutside);
    EXPECT_EQ(grid.height_at({0.5, std::nextafter(10.0, 9.0)}), kOutside);
    EXPECT_EQ(grid.height_at({0.5, std::nextafter(11.0, 12.0)}), kOutside);
    EXPECT_EQ(grid.height_at({50.0, 50.0}), kOutside);
}

TEST(ElevationGridTest, ConsultedNoDataSampleMakesTheLookupNoData) {
    std::vector<float> heights = planar_heights();
    heights[4] = kNoData;  // centre sample (row 1, column 1)
    const ElevationGrid grid{kLayout3x3, std::move(heights), ElevationSurface::SurfaceModel,
                             VerticalDatum::Egm96};

    EXPECT_EQ(grid.sample_m(1, 1), std::nullopt);
    EXPECT_EQ(grid.height_at({0.5, 10.5}), kNoDataLookup);    // on the void itself
    EXPECT_EQ(grid.height_at({0.75, 10.25}), kNoDataLookup);  // a cell touching it
    EXPECT_EQ(grid.height_at({0.25, 10.75}), kNoDataLookup);  // another such cell
    EXPECT_EQ(grid.height_at({0.5, 10.25}), kNoDataLookup);   // an edge ending at it

    // Zero-weight samples are not consulted: valid samples and edges next to
    // the void still answer.
    EXPECT_EQ(grid.height_at({1.0, 10.0}), ok(0.0));
    EXPECT_EQ(grid.height_at({1.0, 10.5}), ok(10.0));
    EXPECT_EQ(grid.height_at({1.0, 10.25}), ok(5.0));
    EXPECT_EQ(grid.height_at({0.75, 10.0}), ok(15.0));
}

TEST(ElevationGridTest, SouthernAndWesternHemisphereSignsAreRespected) {
    const ElevationGrid grid{
        ElevationGridLayout{Wgs84Coordinate{-33.0, -71.0}, Wgs84Coordinate{-34.0, -70.0}, 2,
                            2},
        {1.0F, 2.0F, 3.0F, 4.0F}, ElevationSurface::SurfaceModel, VerticalDatum::Egm96};
    EXPECT_EQ(grid.height_at({-33.0, -71.0}), ok(1.0));  // north-west
    EXPECT_EQ(grid.height_at({-33.0, -70.0}), ok(2.0));  // north-east
    EXPECT_EQ(grid.height_at({-34.0, -71.0}), ok(3.0));  // south-west
    EXPECT_EQ(grid.height_at({-34.0, -70.0}), ok(4.0));  // south-east
    EXPECT_EQ(grid.height_at({-32.5, -70.5}), kOutside);
}

TEST(ElevationGridTest, GridTouchingTheAntimeridianDoesNotWrapQueries) {
    // A grid never crosses the antimeridian; a query at -180 is a different
    // longitude value from the grid's +180 edge and is outside (ADR-022).
    const ElevationGrid grid{
        ElevationGridLayout{Wgs84Coordinate{1.0, 179.0}, Wgs84Coordinate{0.0, 180.0}, 2, 2},
        {1.0F, 2.0F, 3.0F, 4.0F}, ElevationSurface::SurfaceModel, VerticalDatum::Egm96};
    EXPECT_EQ(grid.height_at({1.0, 180.0}), ok(2.0));
    EXPECT_EQ(grid.height_at({1.0, -180.0}), kOutside);
}

TEST(ElevationGridTest, RejectsInvalidLayouts) {
    const auto make = [](ElevationGridLayout layout, std::vector<float> heights) {
        return ElevationGrid{layout, std::move(heights), ElevationSurface::SurfaceModel,
                             VerticalDatum::Egm96};
    };
    const std::vector<float> four(4, 0.0F);
    const Wgs84Coordinate north_west{1.0, 10.0};
    const Wgs84Coordinate south_east{0.0, 11.0};

    EXPECT_THROW((void)make({north_west, south_east, 1, 4}, four), std::invalid_argument);
    EXPECT_THROW((void)make({north_west, south_east, 4, 1}, four), std::invalid_argument);
    // North must be strictly north of south; east strictly east of west.
    EXPECT_THROW((void)make({{0.0, 10.0}, {0.0, 11.0}, 2, 2}, four), std::invalid_argument);
    EXPECT_THROW((void)make({{0.0, 10.0}, {1.0, 11.0}, 2, 2}, four), std::invalid_argument);
    EXPECT_THROW((void)make({{1.0, 11.0}, {0.0, 11.0}, 2, 2}, four), std::invalid_argument);
    // An antimeridian-crossing box cannot be expressed: east < west.
    EXPECT_THROW((void)make({{1.0, 179.5}, {0.0, -179.5}, 2, 2}, four), std::invalid_argument);
    // Corners must be finite and within WGS84 bounds.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((void)make({{nan, 10.0}, south_east, 2, 2}, four), std::invalid_argument);
    EXPECT_THROW((void)make({{91.0, 10.0}, south_east, 2, 2}, four), std::invalid_argument);
    EXPECT_THROW((void)make({north_west, {0.0, 181.0}, 2, 2}, four), std::invalid_argument);
    // Sample count must match the lattice.
    EXPECT_THROW((void)make({north_west, south_east, 2, 2}, std::vector<float>(3, 0.0F)),
                 std::invalid_argument);
    EXPECT_THROW((void)make({north_west, south_east, 2, 2}, std::vector<float>(5, 0.0F)),
                 std::invalid_argument);
    // Infinite samples are rejected; only NaN means NODATA.
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_THROW((void)make({north_west, south_east, 2, 2}, {0.0F, inf, 0.0F, 0.0F}),
                 std::invalid_argument);
    EXPECT_THROW((void)make({north_west, south_east, 2, 2}, {0.0F, 0.0F, -inf, 0.0F}),
                 std::invalid_argument);
    EXPECT_NO_THROW((void)make({north_west, south_east, 2, 2}, {0.0F, kNoData, 0.0F, 0.0F}));
}

TEST(ElevationGridTest, RejectsInvalidQueryPositions) {
    const ElevationGrid grid = planar_grid();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((void)grid.height_at({nan, 10.5}), std::invalid_argument);
    EXPECT_THROW((void)grid.height_at({0.5, nan}), std::invalid_argument);
    EXPECT_THROW((void)grid.height_at({90.5, 10.5}), std::invalid_argument);
    EXPECT_THROW((void)grid.height_at({0.5, -180.5}), std::invalid_argument);
}

TEST(ElevationGridTest, SampleIndexOutsideTheLatticeThrows) {
    const ElevationGrid grid = planar_grid();
    EXPECT_THROW((void)grid.sample_m(3, 0), std::out_of_range);
    EXPECT_THROW((void)grid.sample_m(0, 3), std::out_of_range);
}

}  // namespace

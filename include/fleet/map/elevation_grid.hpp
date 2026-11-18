#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "fleet/map/geometry.hpp"

namespace fleet::map {

// What an elevation grid's heights describe (ADR-022). Explicit because the
// two differ by vegetation and building height, and different consumers need
// different ones.
enum class ElevationSurface : std::uint8_t {
    SurfaceModel,  // first surface: canopy, roofs — what a downward rangefinder sees
    BareEarth,     // ground under vegetation and buildings — what a ground robot drives on
};

// Vertical reference of the heights. A LABEL, never a conversion: grids keep
// their source heights, and the datum is explicit so grids on different
// references are never mixed silently (ADR-022). Values are added only when a
// supported source needs them.
enum class VerticalDatum : std::uint8_t {
    Egm96,      // EGM96 geoid heights in meters (SRTM)
    Synthetic,  // heights over an arbitrary scenario-defined reference (ADR-023);
                // comparable only with other synthetic grids
};

// Georeferencing of a regular latitude/longitude lattice of POINT samples
// (not cell areas): row 0 lies on north_west.latitude_deg and the last row on
// south_east.latitude_deg; column 0 lies on north_west.longitude_deg and the
// last column on south_east.longitude_deg; spacing is uniform in between.
// Corners, not spacings, are the stored truth, so lattice edges are exact
// (adjacent SRTM tiles share their edge coordinates bit for bit).
struct ElevationGridLayout {
    Wgs84Coordinate north_west;
    Wgs84Coordinate south_east;
    std::size_t rows = 0;     // >= 2
    std::size_t columns = 0;  // >= 2

    constexpr bool operator==(const ElevationGridLayout&) const noexcept = default;
};

enum class ElevationStatus : std::uint8_t {
    Ok,
    OutsideCoverage,  // outside the closed box between the outermost samples
    NoData,           // a sample the interpolation needs is a void in the source
};

// Outcome of one lookup. height_m is engaged EXACTLY when status == Ok: a
// lookup never substitutes a value — no clamping to the edge, no
// extrapolation, no zero-fill (ADR-022).
struct ElevationLookup {
    ElevationStatus status = ElevationStatus::OutsideCoverage;
    std::optional<double> height_m;

    bool operator==(const ElevationLookup&) const = default;
};

// An immutable regular elevation grid (ADR-022, M4 #19): heights in meters on
// a WGS84 latitude/longitude lattice, queried by bilinear interpolation.
//
// Construction contracts (std::invalid_argument otherwise):
//   - layout corners are finite and within WGS84 bounds;
//   - north latitude > south latitude and east longitude > west longitude:
//     a grid never crosses the antimeridian;
//   - rows >= 2 and columns >= 2;
//   - exactly rows * columns samples, row-major, north row first, west column
//     first; a NaN sample marks NODATA (a void in the source); infinite
//     samples are rejected.
//
// Lookup contracts:
//   - query positions must be finite and within WGS84 bounds
//     (std::invalid_argument otherwise): an invalid position is a caller
//     bug, not a coverage outcome;
//   - coverage is the CLOSED box between the outermost samples; anything
//     outside is OutsideCoverage;
//   - inside, the height is the bilinear interpolation of the enclosing
//     cell's four samples. A sample whose bilinear weight is exactly zero is
//     not consulted: a lookup exactly on a sample returns that sample, and a
//     lookup on a cell edge uses only that edge's two samples. Any consulted
//     NODATA sample makes the lookup NoData;
//   - a pure function of (grid, position): no state, no randomness, a fixed
//     summation order.
//
// Roles (ADR-022): world truth terrain and each robot's terrain map are
// separate handles to grids; this type does not know which role it plays.
//
// Thread-safety: immutable after construction; safe to share.
class ElevationGrid {
public:
    ElevationGrid(ElevationGridLayout layout, std::vector<float> heights_m,
                  ElevationSurface surface, VerticalDatum datum);

    // The layout half of the construction contract, exposed so producers
    // (e.g. synthetic terrain) can validate before computing samples.
    // Throws std::invalid_argument with the same messages as the constructor.
    static void validate_layout(const ElevationGridLayout& layout);

    [[nodiscard]] const ElevationGridLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] ElevationSurface surface() const noexcept { return surface_; }
    [[nodiscard]] VerticalDatum vertical_datum() const noexcept { return datum_; }

    // The stored sample in meters; nullopt for NODATA. Throws
    // std::out_of_range for an index outside the lattice.
    [[nodiscard]] std::optional<double> sample_m(std::size_t row, std::size_t column) const;

    [[nodiscard]] ElevationLookup height_at(Wgs84Coordinate position) const;

private:
    ElevationGridLayout layout_;
    std::vector<float> heights_m_;
    ElevationSurface surface_;
    VerticalDatum datum_;
};

}  // namespace fleet::map

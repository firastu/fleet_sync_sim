#include "fleet/map/elevation_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fleet::map {

namespace {

[[nodiscard]] bool is_valid_coordinate(const Wgs84Coordinate& position) {
    return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
           position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0 &&
           position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0;
}

}  // namespace

ElevationGrid::ElevationGrid(ElevationGridLayout layout, std::vector<float> heights_m,
                             ElevationSurface surface, VerticalDatum datum)
    : layout_{layout}, heights_m_{std::move(heights_m)}, surface_{surface}, datum_{datum} {
    if (!is_valid_coordinate(layout_.north_west) || !is_valid_coordinate(layout_.south_east)) {
        throw std::invalid_argument(
            "ElevationGrid: layout corners must be finite and within WGS84 bounds");
    }
    if (!(layout_.north_west.latitude_deg > layout_.south_east.latitude_deg)) {
        throw std::invalid_argument(
            "ElevationGrid: north latitude must be greater than south latitude");
    }
    if (!(layout_.south_east.longitude_deg > layout_.north_west.longitude_deg)) {
        throw std::invalid_argument(
            "ElevationGrid: east longitude must be greater than west longitude (a grid "
            "never crosses the antimeridian)");
    }
    if (layout_.rows < 2 || layout_.columns < 2) {
        throw std::invalid_argument(
            "ElevationGrid: a grid needs at least two rows and two columns");
    }
    if (layout_.rows > std::numeric_limits<std::size_t>::max() / layout_.columns) {
        throw std::invalid_argument("ElevationGrid: rows * columns overflows");
    }
    const std::size_t expected = layout_.rows * layout_.columns;
    if (heights_m_.size() != expected) {
        throw std::invalid_argument(std::format(
            "ElevationGrid: expected {} samples ({} rows x {} columns), got {}", expected,
            layout_.rows, layout_.columns, heights_m_.size()));
    }
    for (std::size_t index = 0; index < heights_m_.size(); ++index) {
        if (std::isinf(heights_m_[index])) {
            throw std::invalid_argument(std::format(
                "ElevationGrid: sample at row {}, column {} is infinite",
                index / layout_.columns, index % layout_.columns));
        }
    }
}

std::optional<double> ElevationGrid::sample_m(std::size_t row, std::size_t column) const {
    if (row >= layout_.rows || column >= layout_.columns) {
        throw std::out_of_range("ElevationGrid: sample index outside the lattice");
    }
    const float sample = heights_m_[row * layout_.columns + column];
    if (std::isnan(sample)) {
        return std::nullopt;
    }
    return static_cast<double>(sample);
}

ElevationLookup ElevationGrid::height_at(Wgs84Coordinate position) const {
    if (!is_valid_coordinate(position)) {
        throw std::invalid_argument(
            "ElevationGrid: query position must be finite and within WGS84 bounds");
    }

    const double north = layout_.north_west.latitude_deg;
    const double south = layout_.south_east.latitude_deg;
    const double west = layout_.north_west.longitude_deg;
    const double east = layout_.south_east.longitude_deg;
    if (position.latitude_deg > north || position.latitude_deg < south ||
        position.longitude_deg < west || position.longitude_deg > east) {
        return ElevationLookup{ElevationStatus::OutsideCoverage, std::nullopt};
    }

    // Fractional lattice coordinates. IEEE rounding is monotonic, so a
    // position inside the closed box maps into [0, rows - 1] x
    // [0, columns - 1] without any clamping: (north - lat) <= (north - south)
    // implies their rounded quotient <= 1, and the edges map exactly to
    // 0 and rows - 1 (x / x == 1).
    const double row =
        (north - position.latitude_deg) / (north - south) * static_cast<double>(layout_.rows - 1);
    const double column = (position.longitude_deg - west) / (east - west) *
                          static_cast<double>(layout_.columns - 1);

    // The enclosing cell. On the last row/column the cell below/right does
    // not exist: use the previous cell with fraction 1, which gives the
    // far-side samples zero weight.
    const std::size_t row0 = std::min(static_cast<std::size_t>(row), layout_.rows - 2);
    const std::size_t column0 = std::min(static_cast<std::size_t>(column), layout_.columns - 2);
    const double row_fraction = row - static_cast<double>(row0);
    const double column_fraction = column - static_cast<double>(column0);

    struct Corner {
        std::size_t row;
        std::size_t column;
        double weight;
    };
    const std::array<Corner, 4> corners{{
        {row0, column0, (1.0 - row_fraction) * (1.0 - column_fraction)},
        {row0, column0 + 1, (1.0 - row_fraction) * column_fraction},
        {row0 + 1, column0, row_fraction * (1.0 - column_fraction)},
        {row0 + 1, column0 + 1, row_fraction * column_fraction},
    }};

    double height_m = 0.0;
    for (const Corner& corner : corners) {
        if (corner.weight == 0.0) {
            continue;  // zero weight: not consulted, so a void here cannot matter
        }
        const float sample = heights_m_[corner.row * layout_.columns + corner.column];
        if (std::isnan(sample)) {
            return ElevationLookup{ElevationStatus::NoData, std::nullopt};
        }
        height_m += corner.weight * static_cast<double>(sample);
    }
    return ElevationLookup{ElevationStatus::Ok, height_m};
}

}  // namespace fleet::map

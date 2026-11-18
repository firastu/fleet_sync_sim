#include "fleet/dem/synthetic_terrain.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fleet::dem {

namespace {

constexpr double kEarthRadiusMeters = 6371008.8;

[[nodiscard]] bool is_valid_coordinate(const map::Wgs84Coordinate& position) {
    return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
           position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0 &&
           position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0;
}

[[nodiscard]] double haversine_m(const map::Wgs84Coordinate& a, const map::Wgs84Coordinate& b) {
    constexpr double kDegToRad = std::numbers::pi / 180.0;
    const double lat_a = a.latitude_deg * kDegToRad;
    const double lat_b = b.latitude_deg * kDegToRad;
    const double half_dlat = (b.latitude_deg - a.latitude_deg) * kDegToRad / 2.0;
    const double half_dlon = (b.longitude_deg - a.longitude_deg) * kDegToRad / 2.0;
    const double h = std::sin(half_dlat) * std::sin(half_dlat) +
                     std::cos(lat_a) * std::cos(lat_b) * std::sin(half_dlon) * std::sin(half_dlon);
    return 2.0 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(h)));
}

// Position of lattice index i of n between `first` and `last` (inclusive):
// the outermost indices are the corners exactly.
[[nodiscard]] double lattice_coordinate(double first, double last, std::size_t index,
                                        std::size_t count) {
    if (index + 1 == count) {
        return last;
    }
    return first + (last - first) * (static_cast<double>(index) / static_cast<double>(count - 1));
}

}  // namespace

map::ElevationGrid synthesize_terrain(const SyntheticTerrainSpec& spec) {
    if (!std::isfinite(spec.base_height_m)) {
        throw std::invalid_argument("synthetic terrain: base_height_m must be finite");
    }
    for (std::size_t index = 0; index < spec.hills.size(); ++index) {
        const SyntheticHill& hill = spec.hills[index];
        if (!is_valid_coordinate(hill.center)) {
            throw std::invalid_argument(std::format(
                "synthetic terrain: hill {} centre must be finite and within WGS84 bounds", index));
        }
        if (!std::isfinite(hill.height_m)) {
            throw std::invalid_argument(
                std::format("synthetic terrain: hill {} height_m must be finite", index));
        }
        if (!std::isfinite(hill.radius_m) || !(hill.radius_m > 0.0)) {
            throw std::invalid_argument(std::format(
                "synthetic terrain: hill {} radius_m must be positive and finite", index));
        }
    }

    const map::ElevationGridLayout& layout = spec.layout;
    map::ElevationGrid::validate_layout(layout);  // before any sample is computed

    std::vector<float> heights_m(layout.rows * layout.columns);
    for (std::size_t row = 0; row < layout.rows; ++row) {
        const double latitude = lattice_coordinate(
            layout.north_west.latitude_deg, layout.south_east.latitude_deg, row, layout.rows);
        for (std::size_t column = 0; column < layout.columns; ++column) {
            const double longitude =
                lattice_coordinate(layout.north_west.longitude_deg,
                                   layout.south_east.longitude_deg, column, layout.columns);
            const map::Wgs84Coordinate sample{latitude, longitude};
            double height = spec.base_height_m;
            for (const SyntheticHill& hill : spec.hills) {
                const double distance = haversine_m(sample, hill.center);
                height += hill.height_m *
                          std::exp(-(distance * distance) / (2.0 * hill.radius_m * hill.radius_m));
            }
            if (!(std::abs(height) <= static_cast<double>(std::numeric_limits<float>::max()))) {
                throw std::invalid_argument(std::format(
                    "synthetic terrain: height at row {}, column {} does not fit a finite float",
                    row, column));
            }
            heights_m[row * layout.columns + column] = static_cast<float>(height);
        }
    }
    return map::ElevationGrid{layout, std::move(heights_m), spec.surface,
                              map::VerticalDatum::Synthetic};
}

}  // namespace fleet::dem

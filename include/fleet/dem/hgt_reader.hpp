#pragma once

#include <compare>
#include <cstddef>
#include <filesystem>
#include <span>
#include <string_view>

#include "fleet/map/elevation_grid.hpp"

namespace fleet::dem {

// One SRTM tile, identified by the integer-degree SOUTH-WEST corner of the
// 1 x 1 degree area it covers.
struct HgtTile {
    int south_latitude_deg = 0;  // [-90, 89]
    int west_longitude_deg = 0;  // [-180, 179]

    constexpr auto operator<=>(const HgtTile&) const noexcept = default;
};

// Byte sizes of the two SRTM tile resolutions (square lattices of big-endian
// 16-bit samples).
inline constexpr std::size_t kHgt1ArcSecondSide = 3601;
inline constexpr std::size_t kHgt3ArcSecondSide = 1201;
inline constexpr std::size_t kHgt1ArcSecondBytes = kHgt1ArcSecondSide * kHgt1ArcSecondSide * 2;
inline constexpr std::size_t kHgt3ArcSecondBytes = kHgt3ArcSecondSide * kHgt3ArcSecondSide * 2;

// Parses an SRTM tile name: N|S and two digits, then E|W and three digits,
// uppercase, e.g. "N37W122" = the tile spanning 37..38 N, 122..121 W.
// "S00" and "W000" are rejected (those tiles are named N00 / E000), as is
// anything outside N00..N89 / S01..S90 and E000..E179 / W001..W180.
// Throws std::invalid_argument naming the offending text.
[[nodiscard]] HgtTile parse_hgt_tile_name(std::string_view name);

// Decodes SRTM .hgt content into an ElevationGrid (ADR-022, M4 #19).
//
// Format contract:
//   - exactly kHgt1ArcSecondBytes (3601 x 3601) or kHgt3ArcSecondBytes
//     (1201 x 1201); any other size is rejected — no guessing;
//   - samples are big-endian signed 16-bit meters, row-major, north row
//     first, west column first;
//   - -32768 is a void and becomes a NODATA sample, never a height;
//   - samples are points: the first row lies on south + 1, the last on
//     south; the first column on west, the last on west + 1 (adjacent
//     tiles share their edge rows/columns);
//   - heights are EGM96 geoid heights (the format's datum); `surface` is
//     stated by the caller, never assumed — SRTM itself is a surface model,
//     but other producers use the same container for bare-earth data.
//
// Throws std::invalid_argument for a wrong size or a tile outside the
// supported range.
[[nodiscard]] map::ElevationGrid decode_hgt(std::span<const std::byte> bytes, HgtTile tile,
                                            map::ElevationSurface surface);

// Reads an SRTM tile from disk. The file name must be exactly
// "<tile name>.hgt" (e.g. N37W122.hgt): the tile comes from the name, so a
// renamed file cannot silently land somewhere else. Throws
// std::invalid_argument for a bad name, extension or size (naming the file),
// std::runtime_error when the file cannot be read.
[[nodiscard]] map::ElevationGrid read_hgt(const std::filesystem::path& file,
                                          map::ElevationSurface surface);

}  // namespace fleet::dem

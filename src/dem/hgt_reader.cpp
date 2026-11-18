#include "fleet/dem/hgt_reader.hpp"

#include <cstdint>
#include <format>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fleet::dem {

namespace {

constexpr std::int16_t kVoidSample = std::numeric_limits<std::int16_t>::min();  // -32768

[[nodiscard]] bool is_digit(char c) { return c >= '0' && c <= '9'; }

[[nodiscard]] int parse_digits(std::string_view digits) {
    int value = 0;
    for (const char c : digits) {
        value = value * 10 + (c - '0');
    }
    return value;
}

void validate_tile(const HgtTile& tile) {
    if (tile.south_latitude_deg < -90 || tile.south_latitude_deg > 89 ||
        tile.west_longitude_deg < -180 || tile.west_longitude_deg > 179) {
        throw std::invalid_argument(std::format(
            "HGT: tile with south-west corner ({}, {}) is outside the supported range "
            "(latitude -90..89, longitude -180..179)",
            tile.south_latitude_deg, tile.west_longitude_deg));
    }
}

}  // namespace

HgtTile parse_hgt_tile_name(std::string_view name) {
    const auto reject = [&name](const char* why) {
        return std::invalid_argument(
            std::format("HGT: '{}' is not an SRTM tile name ({})", name, why));
    };

    // Shape: [NS]dd[EW]ddd — exactly seven characters.
    if (name.size() != 7) {
        throw reject("expected 7 characters like N37W122");
    }
    const char hemisphere_ns = name[0];
    const char hemisphere_ew = name[3];
    if ((hemisphere_ns != 'N' && hemisphere_ns != 'S') ||
        (hemisphere_ew != 'E' && hemisphere_ew != 'W')) {
        throw reject("expected uppercase N/S then E/W");
    }
    for (const std::size_t index : {1U, 2U, 4U, 5U, 6U}) {
        if (!is_digit(name[index])) {
            throw reject("expected two latitude digits and three longitude digits");
        }
    }

    const int latitude = parse_digits(name.substr(1, 2));
    const int longitude = parse_digits(name.substr(4, 3));
    if (hemisphere_ns == 'N' ? latitude > 89 : (latitude < 1 || latitude > 90)) {
        throw reject("latitude must be N00..N89 or S01..S90");
    }
    if (hemisphere_ew == 'E' ? longitude > 179 : (longitude < 1 || longitude > 180)) {
        throw reject("longitude must be E000..E179 or W001..W180");
    }
    return HgtTile{hemisphere_ns == 'N' ? latitude : -latitude,
                   hemisphere_ew == 'E' ? longitude : -longitude};
}

map::ElevationGrid decode_hgt(std::span<const std::byte> bytes, HgtTile tile,
                              map::ElevationSurface surface) {
    validate_tile(tile);

    std::size_t side = 0;
    if (bytes.size() == kHgt1ArcSecondBytes) {
        side = kHgt1ArcSecondSide;
    } else if (bytes.size() == kHgt3ArcSecondBytes) {
        side = kHgt3ArcSecondSide;
    } else {
        throw std::invalid_argument(std::format(
            "HGT: {} bytes is neither a 1-arc-second tile ({} bytes) nor a 3-arc-second "
            "tile ({} bytes)",
            bytes.size(), kHgt1ArcSecondBytes, kHgt3ArcSecondBytes));
    }

    std::vector<float> heights_m(side * side);
    for (std::size_t index = 0; index < heights_m.size(); ++index) {
        const auto high = std::to_integer<unsigned>(bytes[2 * index]);
        const auto low = std::to_integer<unsigned>(bytes[2 * index + 1]);
        // Big-endian two's complement; the unsigned->signed conversion is
        // modular (well defined since C++20).
        const auto raw = static_cast<std::uint16_t>((high << 8U) | low);
        const auto sample = static_cast<std::int16_t>(raw);
        heights_m[index] = sample == kVoidSample ? std::numeric_limits<float>::quiet_NaN()
                                                 : static_cast<float>(sample);
    }

    const auto south = static_cast<double>(tile.south_latitude_deg);
    const auto west = static_cast<double>(tile.west_longitude_deg);
    const map::ElevationGridLayout layout{
        map::Wgs84Coordinate{south + 1.0, west},
        map::Wgs84Coordinate{south, west + 1.0},
        side,
        side,
    };
    return map::ElevationGrid{layout, std::move(heights_m), surface, map::VerticalDatum::Egm96};
}

map::ElevationGrid read_hgt(const std::filesystem::path& file, map::ElevationSurface surface) {
    const std::string file_name = file.string();
    if (file.extension() != ".hgt") {
        throw std::invalid_argument(
            std::format("HGT: '{}' must have the .hgt extension", file_name));
    }

    HgtTile tile;
    try {
        tile = parse_hgt_tile_name(file.stem().string());
    } catch (const std::invalid_argument& error) {
        throw std::invalid_argument(std::format("HGT: file '{}': {}", file_name, error.what()));
    }

    std::ifstream input{file, std::ios::binary | std::ios::ate};
    if (!input) {
        throw std::runtime_error(std::format("HGT: cannot open '{}'", file_name));
    }
    const std::streamoff size = input.tellg();
    if (size < 0) {
        throw std::runtime_error(std::format("HGT: cannot determine the size of '{}'", file_name));
    }
    const auto byte_count = static_cast<std::size_t>(size);
    // Checked before allocating, so a wrong file is never read into memory.
    if (byte_count != kHgt1ArcSecondBytes && byte_count != kHgt3ArcSecondBytes) {
        throw std::invalid_argument(std::format(
            "HGT: file '{}' has {} bytes, neither a 1-arc-second tile ({} bytes) nor a "
            "3-arc-second tile ({} bytes)",
            file_name, byte_count, kHgt1ArcSecondBytes, kHgt3ArcSecondBytes));
    }

    std::vector<std::byte> bytes(byte_count);
    input.seekg(0);
    input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(byte_count));
    if (!input) {
        throw std::runtime_error(std::format("HGT: failed to read '{}'", file_name));
    }
    return decode_hgt(bytes, tile, surface);
}

}  // namespace fleet::dem

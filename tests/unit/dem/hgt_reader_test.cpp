#include "fleet/dem/hgt_reader.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "fleet/map/elevation_grid.hpp"

namespace {

using fleet::dem::decode_hgt;
using fleet::dem::HgtTile;
using fleet::dem::kHgt1ArcSecondBytes;
using fleet::dem::kHgt1ArcSecondSide;
using fleet::dem::kHgt3ArcSecondBytes;
using fleet::dem::kHgt3ArcSecondSide;
using fleet::dem::parse_hgt_tile_name;
using fleet::dem::read_hgt;
using fleet::map::ElevationGrid;
using fleet::map::ElevationGridLayout;
using fleet::map::ElevationLookup;
using fleet::map::ElevationStatus;
using fleet::map::ElevationSurface;
using fleet::map::VerticalDatum;
using fleet::map::Wgs84Coordinate;

constexpr std::int16_t kVoid = -32768;

// Synthesizes HGT content: big-endian signed 16-bit samples, north row
// first. No binary fixture is committed — tiles are generated per test.
[[nodiscard]] std::vector<std::byte> make_hgt(
    std::size_t side, const std::function<std::int16_t(std::size_t, std::size_t)>& sample) {
    std::vector<std::byte> bytes(side * side * 2);
    for (std::size_t row = 0; row < side; ++row) {
        for (std::size_t column = 0; column < side; ++column) {
            const auto raw = static_cast<std::uint16_t>(sample(row, column));
            const std::size_t index = 2 * (row * side + column);
            bytes[index] = static_cast<std::byte>(raw >> 8U);
            bytes[index + 1] = static_cast<std::byte>(raw & 0xFFU);
        }
    }
    return bytes;
}

// The pattern used for the 3-arc-second tile: includes negative heights, a
// void, and a sample whose two bytes differ (byte order is observable).
[[nodiscard]] std::int16_t pattern(std::size_t row, std::size_t column) {
    if (row == 0 && column == 0) {
        return 1234;
    }
    if (row == 0 && column == 1) {
        return 0x0102;  // 258 big-endian; 513 if misread as little-endian
    }
    if (row == 600 && column == 600) {
        return kVoid;
    }
    if (row == kHgt3ArcSecondSide - 1 && column == kHgt3ArcSecondSide - 1) {
        return -5;
    }
    return static_cast<std::int16_t>(static_cast<int>((row * 7 + column * 3) % 2000) - 500);
}

[[nodiscard]] std::string error_message(const std::function<void()>& action) {
    try {
        action();
    } catch (const std::exception& error) {
        return error.what();
    }
    return {};
}

// Per-test scratch directory under the test working directory (the build
// tree), removed afterwards.
class ScratchDirectory {
public:
    ScratchDirectory()
        : path_{std::filesystem::path{"hgt_reader_test_tmp"} /
                ::testing::UnitTest::GetInstance()->current_test_info()->name()} {
        std::filesystem::remove_all(path_);
        std::filesystem::create_directories(path_);
    }
    ScratchDirectory(const ScratchDirectory&) = delete;
    ScratchDirectory& operator=(const ScratchDirectory&) = delete;
    ~ScratchDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

    [[nodiscard]] std::filesystem::path write(const std::string& name,
                                              const std::vector<std::byte>& bytes) const {
        const std::filesystem::path file = path_ / name;
        std::ofstream output{file, std::ios::binary};
        output.write(reinterpret_cast<const char*>(bytes.data()),
                     static_cast<std::streamsize>(bytes.size()));
        return file;
    }

private:
    std::filesystem::path path_;
};

TEST(HgtTileNameTest, ParsesTheSouthWestCorner) {
    EXPECT_EQ(parse_hgt_tile_name("N37W122"), (HgtTile{37, -122}));
    EXPECT_EQ(parse_hgt_tile_name("S01E000"), (HgtTile{-1, 0}));
    EXPECT_EQ(parse_hgt_tile_name("N00E000"), (HgtTile{0, 0}));
    EXPECT_EQ(parse_hgt_tile_name("S90W180"), (HgtTile{-90, -180}));
    EXPECT_EQ(parse_hgt_tile_name("N89E179"), (HgtTile{89, 179}));
}

TEST(HgtTileNameTest, RejectsMalformedAndOutOfRangeNames) {
    for (const char* name :
         {"", "N37W12", "N37W1222", "n37w122", "X37W122", "N37X122", "N3AW122", "N37W12A",
          "S00E000", "N00W000", "N90E000", "S91E000", "N00E180", "N00W181", "N37W122.hgt"}) {
        EXPECT_THROW((void)parse_hgt_tile_name(name), std::invalid_argument) << name;
    }
    EXPECT_NE(error_message([] { (void)parse_hgt_tile_name("N90E000"); }).find("N90E000"),
              std::string::npos);
}

TEST(HgtDecodeTest, Decodes3ArcSecondTile) {
    const std::vector<std::byte> bytes = make_hgt(kHgt3ArcSecondSide, pattern);
    ASSERT_EQ(bytes.size(), kHgt3ArcSecondBytes);
    const ElevationGrid grid = decode_hgt(bytes, HgtTile{37, -122}, ElevationSurface::SurfaceModel);

    EXPECT_EQ(grid.layout(), (ElevationGridLayout{Wgs84Coordinate{38.0, -122.0},
                                                  Wgs84Coordinate{37.0, -121.0},
                                                  kHgt3ArcSecondSide, kHgt3ArcSecondSide}));
    EXPECT_EQ(grid.surface(), ElevationSurface::SurfaceModel);
    EXPECT_EQ(grid.vertical_datum(), VerticalDatum::Egm96);

    EXPECT_EQ(grid.sample_m(0, 0), 1234.0);
    EXPECT_EQ(grid.sample_m(0, 1), 258.0);
    EXPECT_EQ(grid.sample_m(600, 600), std::nullopt);
    EXPECT_EQ(grid.sample_m(1200, 1200), -5.0);
    EXPECT_EQ(grid.sample_m(10, 20), static_cast<double>(pattern(10, 20)));
    EXPECT_EQ(grid.sample_m(2, 5), static_cast<double>(pattern(2, 5)));
    ASSERT_LT(pattern(2, 5), 0);  // a negative height decodes as negative

    // The first sample lies on the north-west corner, the last on the
    // south-east corner.
    EXPECT_EQ(grid.height_at({38.0, -122.0}), (ElevationLookup{ElevationStatus::Ok, 1234.0}));
    EXPECT_EQ(grid.height_at({37.0, -121.0}), (ElevationLookup{ElevationStatus::Ok, -5.0}));
    EXPECT_EQ(grid.height_at({37.5, -121.5}).status, ElevationStatus::NoData);
}

TEST(HgtDecodeTest, Decodes1ArcSecondTile) {
    const std::vector<std::byte> bytes =
        make_hgt(kHgt1ArcSecondSide, [](std::size_t row, std::size_t column) -> std::int16_t {
            return row == 3600 && column == 0 ? std::int16_t{-42} : std::int16_t{7};
        });
    ASSERT_EQ(bytes.size(), kHgt1ArcSecondBytes);
    const ElevationGrid grid = decode_hgt(bytes, HgtTile{-34, -71}, ElevationSurface::BareEarth);

    EXPECT_EQ(grid.layout(), (ElevationGridLayout{Wgs84Coordinate{-33.0, -71.0},
                                                  Wgs84Coordinate{-34.0, -70.0},
                                                  kHgt1ArcSecondSide, kHgt1ArcSecondSide}));
    EXPECT_EQ(grid.surface(), ElevationSurface::BareEarth);
    EXPECT_EQ(grid.sample_m(3600, 0), -42.0);
    EXPECT_EQ(grid.height_at({-34.0, -71.0}), (ElevationLookup{ElevationStatus::Ok, -42.0}));
    EXPECT_EQ(grid.height_at({-33.5, -70.5}), (ElevationLookup{ElevationStatus::Ok, 7.0}));
}

TEST(HgtDecodeTest, RejectsUnsupportedSizesAndTiles) {
    const auto decode_size = [](std::size_t size) {
        const std::vector<std::byte> bytes(size);
        (void)decode_hgt(bytes, HgtTile{0, 0}, ElevationSurface::SurfaceModel);
    };
    EXPECT_THROW(decode_size(0), std::invalid_argument);
    EXPECT_THROW(decode_size(2), std::invalid_argument);
    EXPECT_THROW(decode_size(kHgt3ArcSecondBytes - 2), std::invalid_argument);
    EXPECT_THROW(decode_size(kHgt3ArcSecondBytes + 1), std::invalid_argument);
    EXPECT_THROW(decode_size(kHgt1ArcSecondBytes + 2), std::invalid_argument);

    const std::vector<std::byte> bytes(kHgt3ArcSecondBytes);
    EXPECT_THROW((void)decode_hgt(bytes, HgtTile{90, 0}, ElevationSurface::SurfaceModel),
                 std::invalid_argument);
    EXPECT_THROW((void)decode_hgt(bytes, HgtTile{-91, 0}, ElevationSurface::SurfaceModel),
                 std::invalid_argument);
    EXPECT_THROW((void)decode_hgt(bytes, HgtTile{0, 180}, ElevationSurface::SurfaceModel),
                 std::invalid_argument);
}

TEST(HgtDecodeTest, AdjacentTilesShareTheirEdgeExactly) {
    const ElevationGrid west = decode_hgt(
        make_hgt(kHgt3ArcSecondSide,
                 [](std::size_t, std::size_t column) -> std::int16_t {
                     return column == kHgt3ArcSecondSide - 1 ? std::int16_t{100} : std::int16_t{0};
                 }),
        HgtTile{37, -122}, ElevationSurface::SurfaceModel);
    const ElevationGrid east = decode_hgt(
        make_hgt(kHgt3ArcSecondSide,
                 [](std::size_t, std::size_t column) -> std::int16_t {
                     return column == 0 ? std::int16_t{100} : std::int16_t{0};
                 }),
        HgtTile{37, -121}, ElevationSurface::SurfaceModel);

    EXPECT_EQ(west.layout().south_east.longitude_deg, east.layout().north_west.longitude_deg);
    const Wgs84Coordinate on_shared_edge{37.25, -121.0};
    EXPECT_EQ(west.height_at(on_shared_edge), (ElevationLookup{ElevationStatus::Ok, 100.0}));
    EXPECT_EQ(east.height_at(on_shared_edge), (ElevationLookup{ElevationStatus::Ok, 100.0}));
}

TEST(HgtReadTest, ReadsATileFromDiskByItsName) {
    const ScratchDirectory scratch;
    const std::vector<std::byte> bytes = make_hgt(kHgt3ArcSecondSide, pattern);
    const std::filesystem::path file = scratch.write("N37W122.hgt", bytes);

    const ElevationGrid from_disk = read_hgt(file, ElevationSurface::SurfaceModel);
    const ElevationGrid from_memory =
        decode_hgt(bytes, HgtTile{37, -122}, ElevationSurface::SurfaceModel);
    EXPECT_EQ(from_disk.layout(), from_memory.layout());
    EXPECT_EQ(from_disk.sample_m(0, 1), 258.0);
    EXPECT_EQ(from_disk.sample_m(600, 600), std::nullopt);
    EXPECT_EQ(from_disk.height_at({37.3, -121.7}), from_memory.height_at({37.3, -121.7}));
}

TEST(HgtReadTest, RejectsBadNamesSizesAndMissingFiles) {
    const ScratchDirectory scratch;
    const std::vector<std::byte> valid(kHgt3ArcSecondBytes);

    const std::filesystem::path unnamed = scratch.write("tile.hgt", valid);
    EXPECT_THROW((void)read_hgt(unnamed, ElevationSurface::SurfaceModel), std::invalid_argument);
    EXPECT_NE(error_message([&] { (void)read_hgt(unnamed, ElevationSurface::SurfaceModel); })
                  .find("tile.hgt"),
              std::string::npos);

    const std::filesystem::path wrong_extension = scratch.write("N37W122.bin", valid);
    EXPECT_THROW((void)read_hgt(wrong_extension, ElevationSurface::SurfaceModel),
                 std::invalid_argument);

    const std::filesystem::path wrong_size =
        scratch.write("N37W122.hgt", std::vector<std::byte>(100));
    EXPECT_THROW((void)read_hgt(wrong_size, ElevationSurface::SurfaceModel),
                 std::invalid_argument);

    EXPECT_THROW((void)read_hgt(std::filesystem::path{"hgt_reader_test_tmp"} / "missing" /
                                    "N10E010.hgt",
                                ElevationSurface::SurfaceModel),
                 std::runtime_error);
}

}  // namespace

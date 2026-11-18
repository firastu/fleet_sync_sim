#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/localization/terrain_estimator.hpp"
#include "fleet/localization/terrain_sensors.hpp"
#include "fleet/map/elevation_grid.hpp"
#include "fleet/map/geometry.hpp"
#include "fleet/world/terrain_motion.hpp"

namespace fleet::scenario {

// One robot of a terrain-domain scenario (ADR-023).
struct TerrainScenarioRobot {
    std::string name;
    common::RobotId id{};
    world::TerrainMotionConfig motion{};
    map::Wgs84Coordinate start{};
    double start_heading_rad = 0.0;
    std::vector<map::Wgs84Coordinate> waypoints;
    localization::TerrainSensorConfig sensors{};  // opt-in (ADR-024); empty = no sensors
    // Opt-in terrain-referenced estimator (ADR-025); needs odometry, gyro and
    // baro (and a rangefinder for air platforms).
    std::optional<localization::TerrainEstimatorConfig> estimator;
};

// Declarative description of one terrain-domain run (ADR-023, M4 #20).
//
// The loader resolves terrain sources into immutable grids, so this is data
// only: no wiring, no scheduling. The two terrain roles of ADR-022 are
// separate handles; they point at the same grid when the scenario declares
// the robots' map perfect ("robot_map": "truth").
struct TerrainScenario {
    std::string name;
    std::optional<std::uint64_t> seed;  // file-declared; CLI overrides
    std::uint64_t duration_ms = 0;      // run horizon, >= 1
    std::uint64_t step_ms = 100;        // motion integration step, >= 1
    std::shared_ptr<const map::ElevationGrid> truth_terrain;      // world truth
    std::shared_ptr<const map::ElevationGrid> robot_terrain_map;  // what robots carry
    std::vector<TerrainScenarioRobot> robots;  // declaration order = execution order
};

// Parses terrain-domain scenario JSON (ADR-023). The file must declare
// "domain": "terrain". Every object is strict: unknown fields, missing
// required fields and type mismatches throw std::invalid_argument with a
// deterministic message before any simulation state exists. Terrain sources:
//
//   {"synthetic": {north_west, south_east, rows, columns,
//                  base_height_m?, surface?, hills?}}
//   {"hgt": {"file": "<tile>.hgt", "surface": "surface_model"|"bare_earth"}}
//
// An hgt file path is relative to the scenario file's directory (or to
// `base_directory` for load_string) unless absolute. "robot_map" is "truth"
// (default) or another source, which must use the same vertical datum.
// A robot's optional "sensors" object enables any of "odometry", "gyro",
// "baro" and "rangefinder" (air platforms only), ADR-024. Its optional
// "estimator" object configures the grid estimator (ADR-025): a required
// "prior" {lat, lon, heading_deg} plus optional tuning fields; the height
// reference and mount height follow the robot's platform.
class TerrainScenarioLoader {
public:
    [[nodiscard]] static TerrainScenario load(const std::filesystem::path& file);
    [[nodiscard]] static TerrainScenario load_string(
        const std::string& json, const std::filesystem::path& base_directory = {});
};

}  // namespace fleet::scenario

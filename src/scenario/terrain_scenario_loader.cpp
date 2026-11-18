#include "fleet/scenario/terrain_scenario.hpp"

#include <cmath>
#include <cstdint>
#include <format>
#include <fstream>
#include <initializer_list>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#include "fleet/dem/hgt_reader.hpp"
#include "fleet/dem/synthetic_terrain.hpp"

namespace fleet::scenario {

namespace {

using json = nlohmann::json;

constexpr double kDegToRad = std::numbers::pi / 180.0;

[[nodiscard]] std::invalid_argument load_error(const std::string& message) {
    return std::invalid_argument("terrain scenario: " + message);
}

void require_object(const json& value, std::string_view where) {
    if (!value.is_object()) {
        throw load_error(std::format("{} must be an object", where));
    }
}

// Strictness: every key must be one the format defines.
void reject_unknown_fields(const json& object, std::string_view where,
                           std::initializer_list<std::string_view> allowed) {
    for (const auto& [key, value] : object.items()) {
        bool known = false;
        for (const std::string_view name : allowed) {
            known = known || key == name;
        }
        if (!known) {
            throw load_error(std::format("{}: unknown field '{}'", where, key));
        }
    }
}

[[nodiscard]] const json& require(const json& object, const char* key, std::string_view where) {
    const auto entry = object.find(key);
    if (entry == object.end()) {
        throw load_error(std::format("{}: missing field '{}'", where, key));
    }
    return *entry;
}

[[nodiscard]] std::string require_string(const json& object, const char* key,
                                         std::string_view where) {
    const json& value = require(object, key, where);
    if (!value.is_string()) {
        throw load_error(std::format("{}: '{}' must be a string", where, key));
    }
    return value.get<std::string>();
}

[[nodiscard]] double number_value(const json& value, const char* key, std::string_view where) {
    if (!value.is_number()) {
        throw load_error(std::format("{}: '{}' must be a number", where, key));
    }
    const double number = value.get<double>();
    if (!std::isfinite(number)) {
        throw load_error(std::format("{}: '{}' must be finite", where, key));
    }
    return number;
}

[[nodiscard]] double require_number(const json& object, const char* key, std::string_view where) {
    return number_value(require(object, key, where), key, where);
}

[[nodiscard]] std::optional<double> optional_number(const json& object, const char* key,
                                                    std::string_view where) {
    const auto entry = object.find(key);
    if (entry == object.end()) {
        return std::nullopt;
    }
    return number_value(*entry, key, where);
}

[[nodiscard]] std::uint64_t require_positive_ms(const json& object, const char* key,
                                                std::string_view where) {
    const json& value = require(object, key, where);
    if (!value.is_number_unsigned() || value.get<std::uint64_t>() == 0) {
        throw load_error(std::format("{}: '{}' must be a positive integer (ms)", where, key));
    }
    return value.get<std::uint64_t>();
}

[[nodiscard]] std::size_t require_count(const json& object, const char* key,
                                        std::string_view where) {
    const json& value = require(object, key, where);
    if (!value.is_number_unsigned()) {
        throw load_error(std::format("{}: '{}' must be a non-negative integer", where, key));
    }
    return value.get<std::size_t>();
}

// {"lat": d, "lon": d} plus any extra keys the caller allows.
[[nodiscard]] map::Wgs84Coordinate parse_coordinate(
    const json& value, std::string_view where,
    std::initializer_list<std::string_view> allowed = {"lat", "lon"}) {
    require_object(value, where);
    reject_unknown_fields(value, where, allowed);
    return map::Wgs84Coordinate{require_number(value, "lat", where),
                                require_number(value, "lon", where)};
}

[[nodiscard]] map::ElevationSurface parse_surface(const json& value, std::string_view where) {
    if (!value.is_string()) {
        throw load_error(std::format("{}: 'surface' must be a string", where));
    }
    const std::string name = value.get<std::string>();
    if (name == "bare_earth") {
        return map::ElevationSurface::BareEarth;
    }
    if (name == "surface_model") {
        return map::ElevationSurface::SurfaceModel;
    }
    throw load_error(std::format(
        "{}: unknown surface '{}' (supported: bare_earth, surface_model)", where, name));
}

[[nodiscard]] dem::SyntheticTerrainSpec parse_synthetic(const json& value,
                                                        const std::string& where) {
    require_object(value, where);
    reject_unknown_fields(value, where, {"north_west", "south_east", "rows", "columns",
                                         "base_height_m", "surface", "hills"});
    dem::SyntheticTerrainSpec spec;
    spec.layout.north_west = parse_coordinate(require(value, "north_west", where),
                                              where + ".north_west");
    spec.layout.south_east = parse_coordinate(require(value, "south_east", where),
                                              where + ".south_east");
    spec.layout.rows = require_count(value, "rows", where);
    spec.layout.columns = require_count(value, "columns", where);
    spec.base_height_m = optional_number(value, "base_height_m", where).value_or(0.0);
    if (const auto surface = value.find("surface"); surface != value.end()) {
        spec.surface = parse_surface(*surface, where);
    }
    if (const auto hills = value.find("hills"); hills != value.end()) {
        if (!hills->is_array()) {
            throw load_error(std::format("{}: 'hills' must be an array", where));
        }
        for (std::size_t index = 0; index < hills->size(); ++index) {
            const json& hill = (*hills)[index];
            const std::string hill_where = std::format("{}.hills[{}]", where, index);
            require_object(hill, hill_where);
            reject_unknown_fields(hill, hill_where, {"lat", "lon", "height_m", "radius_m"});
            spec.hills.push_back(dem::SyntheticHill{
                map::Wgs84Coordinate{require_number(hill, "lat", hill_where),
                                     require_number(hill, "lon", hill_where)},
                require_number(hill, "height_m", hill_where),
                require_number(hill, "radius_m", hill_where)});
        }
    }
    return spec;
}

// The single place a terrain source becomes a grid.
[[nodiscard]] std::shared_ptr<const map::ElevationGrid> load_source(
    const json& value, const std::string& where, const std::filesystem::path& base_directory) {
    require_object(value, where);
    if (value.size() != 1 || !(value.contains("synthetic") || value.contains("hgt"))) {
        throw load_error(std::format(
            "{}: a terrain source is {{\"synthetic\": {{...}}}} or {{\"hgt\": {{...}}}}", where));
    }
    try {
        if (value.contains("synthetic")) {
            const dem::SyntheticTerrainSpec spec =
                parse_synthetic(value.at("synthetic"), where + ".synthetic");
            return std::make_shared<const map::ElevationGrid>(dem::synthesize_terrain(spec));
        }
        const json& hgt = value.at("hgt");
        const std::string hgt_where = where + ".hgt";
        require_object(hgt, hgt_where);
        reject_unknown_fields(hgt, hgt_where, {"file", "surface"});
        std::filesystem::path file = require_string(hgt, "file", hgt_where);
        if (file.is_relative()) {
            file = base_directory / file;
        }
        return std::make_shared<const map::ElevationGrid>(
            dem::read_hgt(file, parse_surface(require(hgt, "surface", hgt_where), hgt_where)));
    } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        if (message.starts_with("terrain scenario: ")) {
            throw;  // already one of ours
        }
        throw load_error(std::format("{}: {}", where, message));
    } catch (const std::runtime_error& error) {
        throw load_error(std::format("{}: {}", where, error.what()));
    }
}

// One sensor block: every field optional with default 0 unless `required`.
[[nodiscard]] double sensor_number(const json& block, const char* key, std::string_view where,
                                   double scale = 1.0) {
    return optional_number(block, key, where).value_or(0.0) * scale;
}

// The robot's opt-in "sensors" object (ADR-024). Files use degrees; the
// sensor models use radians.
[[nodiscard]] localization::TerrainSensorConfig parse_sensors(const json& value,
                                                              const std::string& where,
                                                              world::TerrainPlatform platform) {
    require_object(value, where);
    reject_unknown_fields(value, where, {"odometry", "gyro", "baro", "rangefinder"});
    localization::TerrainSensorConfig sensors;
    if (const auto entry = value.find("odometry"); entry != value.end()) {
        const std::string block = where + ".odometry";
        require_object(*entry, block);
        reject_unknown_fields(*entry, block, {"scale_error", "noise_m_per_sqrt_m"});
        sensors.odometry = localization::OdometrySensorConfig{
            sensor_number(*entry, "scale_error", block),
            sensor_number(*entry, "noise_m_per_sqrt_m", block)};
    }
    if (const auto entry = value.find("gyro"); entry != value.end()) {
        const std::string block = where + ".gyro";
        require_object(*entry, block);
        reject_unknown_fields(*entry, block,
                              {"initial_bias_deg_per_s", "bias_walk_deg_per_s_per_sqrt_s",
                               "angle_random_walk_deg_per_sqrt_s"});
        sensors.gyro = localization::GyroSensorConfig{
            sensor_number(*entry, "initial_bias_deg_per_s", block, kDegToRad),
            sensor_number(*entry, "bias_walk_deg_per_s_per_sqrt_s", block, kDegToRad),
            sensor_number(*entry, "angle_random_walk_deg_per_sqrt_s", block, kDegToRad)};
    }
    if (const auto entry = value.find("baro"); entry != value.end()) {
        const std::string block = where + ".baro";
        require_object(*entry, block);
        reject_unknown_fields(*entry, block,
                              {"initial_bias_m", "bias_walk_m_per_sqrt_s", "noise_sigma_m"});
        sensors.baro = localization::BaroSensorConfig{
            sensor_number(*entry, "initial_bias_m", block),
            sensor_number(*entry, "bias_walk_m_per_sqrt_s", block),
            sensor_number(*entry, "noise_sigma_m", block)};
    }
    if (const auto entry = value.find("rangefinder"); entry != value.end()) {
        const std::string block = where + ".rangefinder";
        if (platform != world::TerrainPlatform::Air) {
            throw load_error(std::format(
                "{}: a downward rangefinder needs an air platform", block));
        }
        require_object(*entry, block);
        reject_unknown_fields(*entry, block, {"max_range_m", "noise_sigma_m", "dropout_ppm"});
        localization::RangefinderSensorConfig rangefinder;
        rangefinder.max_range_m = require_number(*entry, "max_range_m", block);
        rangefinder.noise_sigma_m = sensor_number(*entry, "noise_sigma_m", block);
        if (const auto dropout = entry->find("dropout_ppm"); dropout != entry->end()) {
            if (!dropout->is_number_unsigned() || dropout->get<std::uint64_t>() > 1'000'000) {
                throw load_error(std::format(
                    "{}: 'dropout_ppm' must be an integer within [0, 1000000]", block));
            }
            rangefinder.dropout_ppm = static_cast<std::uint32_t>(dropout->get<std::uint64_t>());
        }
        sensors.rangefinder = rangefinder;
    }
    try {
        sensors.validate();
    } catch (const std::invalid_argument& error) {
        throw load_error(std::format("{}: {}", where, error.what()));
    }
    return sensors;
}

[[nodiscard]] TerrainScenarioRobot parse_robot(const json& value, std::size_t index,
                                               std::set<std::string>& names,
                                               std::set<std::uint64_t>& ids) {
    const std::string fallback = std::format("robots[{}]", index);
    require_object(value, fallback);
    TerrainScenarioRobot robot;
    robot.name = require_string(value, "name", fallback);
    const std::string where = std::format("robot '{}'", robot.name);
    reject_unknown_fields(value, where,
                          {"name", "id", "platform", "speed_mps", "max_turn_rate_deg_per_s",
                           "arrival_radius_m", "mount_height_m", "altitude_m", "start",
                           "waypoints", "sensors"});
    if (!names.insert(robot.name).second) {
        throw load_error(std::format("duplicate robot name '{}'", robot.name));
    }
    const json& id = require(value, "id", where);
    if (!id.is_number_unsigned() || id.get<std::uint64_t>() > 255) {
        throw load_error(std::format("{}: 'id' must be an integer within [0, 255]", where));
    }
    if (!ids.insert(id.get<std::uint64_t>()).second) {
        throw load_error(std::format("duplicate robot id {}", id.get<std::uint64_t>()));
    }
    robot.id = common::RobotId{static_cast<std::uint8_t>(id.get<std::uint64_t>())};

    const std::string platform = require_string(value, "platform", where);
    if (platform == "ground") {
        robot.motion.platform = world::TerrainPlatform::Ground;
        if (value.contains("altitude_m")) {
            throw load_error(std::format(
                "{}: 'altitude_m' applies to air platforms; a ground robot follows the terrain",
                where));
        }
        robot.motion.mount_height_m = optional_number(value, "mount_height_m", where).value_or(0.0);
    } else if (platform == "air") {
        robot.motion.platform = world::TerrainPlatform::Air;
        if (value.contains("mount_height_m")) {
            throw load_error(std::format(
                "{}: 'mount_height_m' applies to ground platforms; use 'altitude_m'", where));
        }
        robot.motion.altitude_m = require_number(value, "altitude_m", where);
    } else {
        throw load_error(std::format(
            "{}: unknown platform '{}' (supported: ground, air)", where, platform));
    }
    robot.motion.speed_mps = require_number(value, "speed_mps", where);
    robot.motion.max_turn_rate_rad_per_s =
        require_number(value, "max_turn_rate_deg_per_s", where) * kDegToRad;
    robot.motion.arrival_radius_m = require_number(value, "arrival_radius_m", where);
    try {
        robot.motion.validate();
    } catch (const std::invalid_argument& error) {
        throw load_error(std::format("{}: {}", where, error.what()));
    }

    const json& start = require(value, "start", where);
    robot.start = parse_coordinate(start, where + " start", {"lat", "lon", "heading_deg"});
    robot.start_heading_rad =
        optional_number(start, "heading_deg", where + " start").value_or(0.0) * kDegToRad;

    const json& waypoints = require(value, "waypoints", where);
    if (!waypoints.is_array() || waypoints.empty()) {
        throw load_error(std::format("{}: 'waypoints' must be a non-empty array", where));
    }
    for (std::size_t point = 0; point < waypoints.size(); ++point) {
        robot.waypoints.push_back(
            parse_coordinate(waypoints[point], std::format("{} waypoints[{}]", where, point)));
    }
    if (const auto sensors = value.find("sensors"); sensors != value.end()) {
        robot.sensors = parse_sensors(*sensors, where + " sensors", robot.motion.platform);
    }
    return robot;
}

[[nodiscard]] TerrainScenario parse_terrain_scenario(const json& root,
                                                     const std::filesystem::path& base_directory) {
    require_object(root, "top-level value");
    reject_unknown_fields(root, "top-level",
                          {"name", "domain", "seed", "duration_ms", "motion", "terrain", "robots"});
    if (require_string(root, "domain", "top-level") != "terrain") {
        throw load_error("top-level: 'domain' must be \"terrain\"");
    }

    TerrainScenario scenario;
    scenario.name = require_string(root, "name", "top-level");
    if (const auto seed = root.find("seed"); seed != root.end()) {
        if (!seed->is_number_unsigned()) {
            throw load_error("'seed' must be a non-negative integer");
        }
        scenario.seed = seed->get<std::uint64_t>();
    }
    scenario.duration_ms = require_positive_ms(root, "duration_ms", "top-level");
    if (const auto motion = root.find("motion"); motion != root.end()) {
        require_object(*motion, "motion");
        reject_unknown_fields(*motion, "motion", {"step_ms"});
        scenario.step_ms = require_positive_ms(*motion, "step_ms", "motion");
    }

    const json& terrain = require(root, "terrain", "top-level");
    require_object(terrain, "terrain");
    reject_unknown_fields(terrain, "terrain", {"truth", "robot_map"});
    scenario.truth_terrain =
        load_source(require(terrain, "truth", "terrain"), "terrain.truth", base_directory);
    scenario.robot_terrain_map = scenario.truth_terrain;
    if (const auto map = terrain.find("robot_map"); map != terrain.end()) {
        if (map->is_string()) {
            if (map->get<std::string>() != "truth") {
                throw load_error("terrain.robot_map: must be \"truth\" or a terrain source");
            }
        } else {
            scenario.robot_terrain_map = load_source(*map, "terrain.robot_map", base_directory);
            if (scenario.robot_terrain_map->vertical_datum() !=
                scenario.truth_terrain->vertical_datum()) {
                throw load_error(
                    "terrain.robot_map: its vertical datum differs from the truth terrain's "
                    "(heights would not be comparable)");
            }
        }
    }

    const json& robots = require(root, "robots", "top-level");
    if (!robots.is_array() || robots.empty()) {
        throw load_error("'robots' must be a non-empty array");
    }
    std::set<std::string> names;
    std::set<std::uint64_t> ids;
    for (std::size_t index = 0; index < robots.size(); ++index) {
        scenario.robots.push_back(parse_robot(robots[index], index, names, ids));
    }
    return scenario;
}

}  // namespace

TerrainScenario TerrainScenarioLoader::load(const std::filesystem::path& file) {
    std::ifstream input{file};
    if (!input) {
        throw load_error(std::format("cannot open scenario file '{}'", file.string()));
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    json root;
    try {
        root = json::parse(buffer.str());
    } catch (const json::parse_error& error) {
        throw load_error(std::format("JSON parse error in '{}': {}", file.string(), error.what()));
    }
    return parse_terrain_scenario(root, file.parent_path());
}

TerrainScenario TerrainScenarioLoader::load_string(const std::string& json_text,
                                                   const std::filesystem::path& base_directory) {
    json root;
    try {
        root = json::parse(json_text);
    } catch (const json::parse_error& error) {
        throw load_error(std::format("JSON parse error: {}", error.what()));
    }
    return parse_terrain_scenario(root, base_directory);
}

}  // namespace fleet::scenario

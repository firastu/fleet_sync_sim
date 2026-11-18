#include "fleet/scenario/terrain_scenario.hpp"
#include "fleet/scenario/terrain_scenario_runner.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "fleet/dem/hgt_reader.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/trace.hpp"
#include "test_maps.hpp"

namespace {

using fleet::map::ElevationSurface;
using fleet::map::VerticalDatum;
using fleet::scenario::JsonlTraceSink;
using fleet::scenario::ScenarioDomain;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::TerrainScenario;
using fleet::scenario::TerrainScenarioLoader;
using fleet::scenario::TerrainScenarioRunner;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceSink;
using fleet::world::TerrainPlatform;

constexpr const char* kCommitted = "scenarios/terrain_hills.json";

class VectorTraceSink final : public TraceSink {
public:
    void record(const TraceEvent& event) override { events.push_back(event); }
    std::vector<TraceEvent> events;
};

[[nodiscard]] std::string error_message(const std::function<void()>& action) {
    try {
        action();
    } catch (const std::exception& error) {
        return error.what();
    }
    return {};
}

[[nodiscard]] std::string field(const TraceEvent& event, const std::string& key) {
    for (const auto& [name, value] : event.fields) {
        if (name == key) {
            return fleet::scenario::format_trace_value(value);
        }
    }
    return "<missing>";
}

// A small valid scenario as text, with hooks to break one part.
[[nodiscard]] std::string scenario_json(const std::string& robot_extra = "",
                                        const std::string& top_extra = "",
                                        const std::string& robot_map = "\"truth\"",
                                        const std::string& platform = "\"ground\"") {
    return std::string{R"({
  "name": "small", "domain": "terrain", "duration_ms": 60000,)"} +
           top_extra + R"(
  "motion": {"step_ms": 1000},
  "terrain": {
    "truth": {"synthetic": {"north_west": {"lat": 46.01, "lon": 7.0},
                            "south_east": {"lat": 46.0, "lon": 7.01},
                            "rows": 11, "columns": 11, "base_height_m": 500}},
    "robot_map": )" + robot_map + R"(
  },
  "robots": [{"name": "rover", "id": 1, "platform": )" + platform + R"(,)" + robot_extra + R"(
              "speed_mps": 2, "max_turn_rate_deg_per_s": 45, "arrival_radius_m": 10,
              "start": {"lat": 46.002, "lon": 7.002, "heading_deg": 90},
              "waypoints": [{"lat": 46.002, "lon": 7.006}]}]
})";
}

// Per-test scratch directory in the build tree, removed afterwards.
class ScratchDirectory {
public:
    ScratchDirectory()
        : path_{std::filesystem::absolute(
              std::filesystem::path{"terrain_scenario_test_tmp"} /
              ::testing::UnitTest::GetInstance()->current_test_info()->name())} {
        std::filesystem::remove_all(path_);
        std::filesystem::create_directories(path_);
    }
    ScratchDirectory(const ScratchDirectory&) = delete;
    ScratchDirectory& operator=(const ScratchDirectory&) = delete;
    ~ScratchDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

    // A flat 3-arc-second SRTM tile of 700 m heights.
    void write_flat_tile(const std::string& name) const {
        std::vector<char> bytes(fleet::dem::kHgt3ArcSecondBytes);
        for (std::size_t index = 0; index < bytes.size(); index += 2) {
            bytes[index] = static_cast<char>(700 >> 8);
            bytes[index + 1] = static_cast<char>(700 & 0xFF);
        }
        std::ofstream output{path_ / name, std::ios::binary};
        output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }

private:
    std::filesystem::path path_;
};

TEST(ScenarioDomainTest, FilesDeclareTheirDomain) {
    EXPECT_EQ(ScenarioLoader::domain_of("scenarios/station_partition.json"), ScenarioDomain::Graph);
    EXPECT_EQ(ScenarioLoader::domain_of("scenarios/gnss_outage.json"), ScenarioDomain::Graph);
    EXPECT_EQ(ScenarioLoader::domain_of(kCommitted), ScenarioDomain::Terrain);
}

TEST(ScenarioDomainTest, GraphLoaderRejectsTerrainScenariosByName) {
    const auto grid = fleet::testsupport::make_grid_map();
    EXPECT_NE(error_message([&] { (void)ScenarioLoader::load(grid.base, kCommitted); })
                  .find("terrain-domain scenario"),
              std::string::npos);
    EXPECT_NE(error_message([&] {
                  (void)ScenarioLoader::load_string(grid.base,
                                                    R"({"name": "x", "domain": "orbital"})");
              }).find("unknown domain 'orbital'"),
              std::string::npos);
}

TEST(TerrainScenarioLoaderTest, LoadsTheCommittedScenario) {
    const TerrainScenario scenario = TerrainScenarioLoader::load(kCommitted);
    EXPECT_EQ(scenario.name, "terrain_hills");
    EXPECT_EQ(scenario.seed, 1U);
    EXPECT_EQ(scenario.duration_ms, 600000U);
    EXPECT_EQ(scenario.step_ms, 1000U);
    ASSERT_NE(scenario.truth_terrain, nullptr);
    EXPECT_EQ(scenario.truth_terrain->vertical_datum(), VerticalDatum::Synthetic);
    EXPECT_EQ(scenario.truth_terrain->surface(), ElevationSurface::BareEarth);
    EXPECT_EQ(scenario.robot_terrain_map, scenario.truth_terrain);  // "robot_map": "truth"

    ASSERT_EQ(scenario.robots.size(), 2U);
    const auto& rover = scenario.robots[0];
    EXPECT_EQ(rover.name, "rover");
    EXPECT_EQ(rover.motion.platform, TerrainPlatform::Ground);
    EXPECT_EQ(rover.motion.mount_height_m, 0.5);
    EXPECT_DOUBLE_EQ(rover.motion.max_turn_rate_rad_per_s, 45.0 * std::numbers::pi / 180.0);
    EXPECT_DOUBLE_EQ(rover.start_heading_rad, std::numbers::pi / 2.0);
    EXPECT_EQ(rover.waypoints.size(), 3U);
    const auto& drone = scenario.robots[1];
    EXPECT_EQ(drone.motion.platform, TerrainPlatform::Air);
    EXPECT_EQ(drone.motion.altitude_m, 1100.0);
    EXPECT_EQ(drone.waypoints.size(), 4U);
}

TEST(TerrainScenarioLoaderTest, RejectsInvalidDescriptions) {
    const auto rejects = [](const std::string& json, const std::string& expected) {
        const std::string message =
            error_message([&] { (void)TerrainScenarioLoader::load_string(json); });
        EXPECT_NE(message.find(expected), std::string::npos)
            << "expected '" << expected << "', got '" << message << "'";
    };
    EXPECT_NO_THROW((void)TerrainScenarioLoader::load_string(scenario_json()));

    rejects(R"({"name": "x", "duration_ms": 1})", "missing field 'domain'");
    rejects(R"({"name": "x", "domain": "graph", "duration_ms": 1})", "'domain' must be");
    rejects(scenario_json("", R"( "events": [],)"), "unknown field 'events'");
    rejects(scenario_json(R"( "altitude_m": 900,)"), "'altitude_m' applies to air platforms");
    rejects(scenario_json("", "", "\"truth\"", "\"air\""), "missing field 'altitude_m'");
    rejects(scenario_json(R"( "altitude_m": 900, "mount_height_m": 1,)", "", "\"truth\"",
                          "\"air\""),
            "'mount_height_m' applies to ground platforms");
    rejects(scenario_json("", "", "\"truth\"", "\"boat\""), "unknown platform 'boat'");
    rejects(scenario_json(R"( "colour": "red",)"), "unknown field 'colour'");
    rejects(scenario_json("", "", "\"perfect\""), "must be \"truth\" or a terrain source");
    rejects(scenario_json("", "", R"({"synthetic": {}, "hgt": {}})"),
            "a terrain source is");
    rejects(scenario_json("", "", R"({"synthetic": {"north_west": {"lat": 1, "lon": 1},
             "south_east": {"lat": 0, "lon": 2}, "rows": 1, "columns": 2}})"),
            "at least two rows and two columns");

    rejects(R"({"name": "x", "domain": "terrain", "duration_ms": 1000,
                "terrain": {"truth": {"synthetic": {"north_west": {"lat": 46.01, "lon": 7.0},
                  "south_east": {"lat": 46.0, "lon": 7.01}, "rows": 2, "columns": 2}}},
                "robots": []})",
            "'robots' must be a non-empty array");
    // Arrival radius below the turning diameter (2 * 2 m/s / 45 deg/s ~ 5.1 m).
    std::string tight = scenario_json();
    const std::string radius = "\"arrival_radius_m\": 10";
    tight.replace(tight.find(radius), radius.size(), "\"arrival_radius_m\": 4");
    rejects(tight, "turning diameter");
}

TEST(TerrainScenarioLoaderTest, RejectsDuplicateRobots) {
    const std::string base = scenario_json();
    const std::size_t start = base.find("{\"name\": \"rover\"");
    const std::size_t end = base.rfind("]");
    const std::string entry = base.substr(start, end - start);
    std::string twice = base;
    twice.insert(end, ", " + entry);
    EXPECT_NE(error_message([&] { (void)TerrainScenarioLoader::load_string(twice); })
                  .find("duplicate robot name 'rover'"),
              std::string::npos);
    std::string renamed = entry;
    renamed.replace(renamed.find("\"rover\""), 7, "\"other\"");
    std::string same_id = base;
    same_id.insert(end, ", " + renamed);
    EXPECT_NE(error_message([&] { (void)TerrainScenarioLoader::load_string(same_id); })
                  .find("duplicate robot id 1"),
              std::string::npos);
}

TEST(TerrainScenarioLoaderTest, HgtSourcesResolveRelativeToTheScenarioAndCheckTheDatum) {
    const ScratchDirectory scratch;
    scratch.write_flat_tile("N46E007.hgt");

    std::string hgt_truth = scenario_json();
    const std::size_t truth_start = hgt_truth.find("{\"synthetic\"");
    const std::size_t truth_end = hgt_truth.find("\"robot_map\"");
    hgt_truth.replace(truth_start, truth_end - truth_start,
                      R"({"hgt": {"file": "N46E007.hgt", "surface": "surface_model"}},
    )");
    const TerrainScenario scenario = TerrainScenarioLoader::load_string(hgt_truth, scratch.path());
    EXPECT_EQ(scenario.truth_terrain->vertical_datum(), VerticalDatum::Egm96);
    EXPECT_EQ(scenario.truth_terrain->surface(), ElevationSurface::SurfaceModel);
    EXPECT_NEAR(scenario.truth_terrain->height_at({46.002, 7.002}).height_m.value(), 700.0, 1e-9);

    // Without the base directory the relative file is not found.
    EXPECT_NE(error_message([&] { (void)TerrainScenarioLoader::load_string(hgt_truth); })
                  .find("terrain.truth"),
              std::string::npos);

    // A synthetic truth with an EGM96 robot map: heights are not comparable.
    const std::string mixed = scenario_json(
        "", "", R"({"hgt": {"file": "N46E007.hgt", "surface": "surface_model"}})");
    EXPECT_NE(error_message([&] {
                  (void)TerrainScenarioLoader::load_string(mixed, scratch.path());
              }).find("vertical datum differs"),
              std::string::npos);
}

TEST(TerrainScenarioRunnerTest, RejectsRobotsThatCannotRun) {
    TerrainScenario outside = TerrainScenarioLoader::load_string(scenario_json());
    outside.robots[0].start = {47.0, 7.0};
    EXPECT_NE(error_message([&] { TerrainScenarioRunner runner{outside, 0}; })
                  .find("robot 'rover'"),
              std::string::npos);

    TerrainScenario long_step = TerrainScenarioLoader::load_string(scenario_json());
    long_step.step_ms = 10000;  // 20 m per step > 10 m arrival radius
    EXPECT_NE(error_message([&] { TerrainScenarioRunner runner{long_step, 0}; })
                  .find("more than arrival_radius_m"),
              std::string::npos);

    TerrainScenario no_time = TerrainScenarioLoader::load_string(scenario_json());
    no_time.duration_ms = 0;
    EXPECT_THROW(TerrainScenarioRunner(no_time, 0), std::invalid_argument);
}

TEST(TerrainScenarioRunnerTest, RunsTheCommittedScenarioToCompletion) {
    TerrainScenarioRunner runner{TerrainScenarioLoader::load(kCommitted), 1};
    VectorTraceSink sink;
    runner.add_sink(sink);
    const TerrainScenarioRunner::Result result = runner.run_to_completion();
    EXPECT_EQ(result.finished_at.value, 600000U);
    EXPECT_EQ(result.robot_count, 2U);

    ASSERT_GE(sink.events.size(), 5U);
    EXPECT_EQ(sink.events[0].type, "scenario");
    EXPECT_EQ(field(sink.events[0], "domain"), "terrain");
    EXPECT_EQ(sink.events[1].type, "seed");
    EXPECT_EQ(sink.events[2].type, "terrain");
    EXPECT_EQ(field(sink.events[2], "robot_map"), "truth");
    EXPECT_EQ(sink.events[3].type, "terrain_pose");
    EXPECT_EQ(field(sink.events[3], "robot"), "rover");
    EXPECT_EQ(sink.events[3].at.value, 0U);
    EXPECT_EQ(field(sink.events[4], "robot"), "drone");

    std::vector<std::string> reached;
    for (const TraceEvent& event : sink.events) {
        if (event.type == "waypoint_reached") {
            reached.push_back(field(event, "robot") + ":" + field(event, "index") + ":" +
                              field(event, "last"));
        }
        if (event.type == "terrain_pose") {
            EXPECT_EQ(event.at.value % 1000, 0U);  // the step cadence
            if (field(event, "robot") == "rover") {
                EXPECT_EQ(field(event, "agl_m"), "0.50");  // mount height
            }
        }
    }
    EXPECT_EQ(reached, (std::vector<std::string>{"drone:0:false", "rover:0:false",
                                                 "drone:1:false", "drone:2:false",
                                                 "rover:1:false", "drone:3:true",
                                                 "rover:2:true"}));
    EXPECT_TRUE(runner.truth_motion("rover").mission_complete());
    EXPECT_TRUE(runner.truth_motion("drone").mission_complete());

    // ADR-022 roles: with "robot_map": "truth" the robot carries the truth
    // grid's handle, but holds no truth state of its own.
    EXPECT_EQ(runner.robot("rover").mission().waypoints.size(), 3U);
    EXPECT_THROW((void)runner.robot("nobody"), std::invalid_argument);
}

TEST(TerrainScenarioRunnerTest, SeparateRobotMapIsADistinctGrid) {
    const std::string separate = scenario_json(
        "", "", R"({"synthetic": {"north_west": {"lat": 46.01, "lon": 7.0},
                                  "south_east": {"lat": 46.0, "lon": 7.01},
                                  "rows": 3, "columns": 3, "base_height_m": 480}})");
    const TerrainScenario scenario = TerrainScenarioLoader::load_string(separate);
    ASSERT_NE(scenario.robot_terrain_map, scenario.truth_terrain);
    TerrainScenarioRunner runner{scenario, 0};
    VectorTraceSink sink;
    runner.add_sink(sink);
    (void)runner.run_to_completion();
    EXPECT_EQ(field(sink.events[2], "robot_map"), "separate");
    EXPECT_EQ(&runner.robot("rover").terrain_map(), scenario.robot_terrain_map.get());
    EXPECT_NEAR(runner.robot("rover").terrain_map().height_at({46.002, 7.002}).height_m.value(),
                480.0, 1e-9);
    // Truth still drives the body: the truth terrain is 500 m.
    EXPECT_NEAR(runner.truth_motion("rover").state().terrain_height_m, 500.0, 1e-9);
}

TEST(TerrainScenarioRunnerTest, SameScenarioAndSeedGiveByteIdenticalTraces) {
    const auto trace_of = [] {
        TerrainScenarioRunner runner{TerrainScenarioLoader::load(kCommitted), 7};
        std::ostringstream out;
        JsonlTraceSink sink{out};
        runner.add_sink(sink);
        (void)runner.run_to_completion();
        return out.str();
    };
    const std::string first = trace_of();
    EXPECT_FALSE(first.empty());
    EXPECT_EQ(first, trace_of());
}

TEST(TerrainScenarioRunnerTest, LeavingTheTruthTerrainFailsTheRunNamingTheRobot) {
    std::string escaping = scenario_json();
    const std::string waypoint = "\"lon\": 7.006}";
    escaping.replace(escaping.find(waypoint), waypoint.size(), "\"lon\": 7.02}");
    const std::string duration = "\"duration_ms\": 60000";
    escaping.replace(escaping.find(duration), duration.size(), "\"duration_ms\": 600000");
    TerrainScenarioRunner runner{TerrainScenarioLoader::load_string(escaping), 0};
    const std::string message = error_message([&] { (void)runner.run_to_completion(); });
    EXPECT_NE(message.find("robot 'rover'"), std::string::npos) << message;
    EXPECT_NE(message.find("outside the truth terrain coverage"), std::string::npos) << message;
}

}  // namespace

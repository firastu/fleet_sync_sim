// Isaac stage 1 read-only replay export tests (ADR-020). CPU-only: nothing
// here needs Isaac Sim — the export is a pure native adapter output, and the
// bridge's scene-side conversion is tested separately in tools/isaac/tests.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "exporter.hpp"
#include "isaac_fixture_map.hpp"
#include "test_maps.hpp"

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

namespace {

using fleet::isaac::ExportOptions;
using fleet::isaac::ExportSummary;
using fleet::isaac::build_fixture_map;
using fleet::isaac::kFixtureOriginLatitude;
using fleet::isaac::kFixtureOriginLongitude;

const double kPiOverTwo = 1.5707963267948966;

const fleet::isaac::FixtureMap& fixture() {
    static const fleet::isaac::FixtureMap map = build_fixture_map();
    return map;
}

[[nodiscard]] std::string read_text(const std::filesystem::path& path) {
    std::ifstream input{path};
    if (!input.is_open()) {
        throw std::runtime_error("cannot open '" + path.string() + "'");
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

[[nodiscard]] std::filesystem::path fresh_dir(const std::string& name) {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

// The committed fixture scenario (also the exporter's documented input).
[[nodiscard]] std::string replay_scenario_json() {
    return read_text("scenarios/isaac_replay.json");
}

[[nodiscard]] fleet::scenario::Scenario load_json(const std::string& json) {
    return fleet::scenario::ScenarioLoader::load_string(fixture().base, json);
}

struct ExportBasics {
    nlohmann::json manifest;
    std::vector<nlohmann::json> snapshots;
};

[[nodiscard]] ExportBasics parse_export(const std::filesystem::path& dir) {
    ExportBasics result;
    result.manifest = nlohmann::json::parse(read_text(dir / "manifest.json"));
    std::istringstream lines{read_text(dir / "poses.jsonl")};
    std::string line;
    while (std::getline(lines, line)) {
        if (!line.empty()) {
            result.snapshots.push_back(nlohmann::json::parse(line));
        }
    }
    return result;
}

ExportOptions default_options(const std::string& dir_name, std::uint64_t seed = 17,
                              std::uint64_t period_ms = 100) {
    ExportOptions options;
    options.resolved_seed = seed;
    options.period_ms = period_ms;
    options.output_dir = fresh_dir(dir_name);
    options.scenario_json = replay_scenario_json();
    return options;
}

TEST(IsaacFixtureMapTest, FitsInsideFiftyMeterFootprint) {
    const fleet::map::MapGeometry* geometry = fixture().base.geometry();
    ASSERT_NE(geometry, nullptr);

    double farthest_m = 0.0;
    for (const fleet::map::Node& node : fixture().base.graph().nodes()) {
        const fleet::map::Wgs84Coordinate* position = geometry->node_position(node.id);
        ASSERT_NE(position, nullptr);
        const double dlat_deg = position->latitude_deg - kFixtureOriginLatitude;
        const double dlon_deg = position->longitude_deg - kFixtureOriginLongitude;
        // Upper bound via meters-per-degree (R = 6371008.8 m, the pinned
        // conversion-layer constant); the exact footprint contract is the
        // conversion layer's great-circle check.
        constexpr double kMetersPerDegree = 111195.08023406387;
        const double bound_m = std::hypot(dlat_deg * kMetersPerDegree,
                                          dlon_deg * kMetersPerDegree);
        farthest_m = std::max(farthest_m, bound_m);
    }
    EXPECT_GT(farthest_m, 20.0);   // the fixture is really 2D, not a point
    EXPECT_LT(farthest_m, 50.0);   // inside the declared footprint
}

TEST(IsaacFixtureMapTest, MapHashIsStableAcrossBuilds) {
    const fleet::isaac::FixtureMap first = build_fixture_map();
    const fleet::isaac::FixtureMap second = build_fixture_map();
    EXPECT_EQ(fleet::isaac::map_hash(first.base), fleet::isaac::map_hash(second.base));
}

TEST(IsaacExportTest, ProducesCompleteManifestAndBoundedStream) {
    const fleet::scenario::Scenario scenario = load_json(replay_scenario_json());
    ExportOptions options = default_options("fleet_isaac_export_basic");
    const std::filesystem::path dir = options.output_dir;
    const ExportSummary summary =
        fleet::isaac::export_replay_run(fixture().base, scenario, options);

    EXPECT_EQ(summary.first_tick_ms, 0U);
    EXPECT_EQ(summary.last_tick_ms, 6000U);
    EXPECT_EQ(summary.snapshot_count, 61U);
    EXPECT_EQ(summary.robot_count, 1U);
    EXPECT_GT(summary.observed_extent_m, 20.0);
    EXPECT_LE(summary.observed_extent_m, 50.0);

    const ExportBasics exported = parse_export(dir);
    ASSERT_EQ(exported.snapshots.size(), 61U);

    const nlohmann::json& manifest = exported.manifest;
    EXPECT_EQ(manifest["schema"], "fleet-isaac-replay/1");
    EXPECT_EQ(manifest["kind"], "readonly-replay-export");
    EXPECT_EQ(manifest["scenario_name"], "isaac_replay");
    EXPECT_EQ(manifest["resolved_seed"], "17");
    EXPECT_TRUE(manifest["scenario_hash"].get<std::string>().starts_with("fnv1a64:"));
    EXPECT_TRUE(manifest["map_hash"].get<std::string>().starts_with("fnv1a64:"));
    EXPECT_EQ(manifest["origin"]["latitude_deg"], kFixtureOriginLatitude);
    EXPECT_EQ(manifest["origin"]["longitude_deg"], kFixtureOriginLongitude);
    EXPECT_EQ(manifest["scene_frame"]["axes"], "enu");
    EXPECT_EQ(manifest["scene_frame"]["translation_units"], "meters");
    EXPECT_EQ(manifest["scene_frame"]["z_up"], true);
    EXPECT_EQ(manifest["heading_convention"], "radians_clockwise_from_north");
    EXPECT_EQ(manifest["first_tick_ms"], "0");
    EXPECT_EQ(manifest["final_tick_ms"], "6000");
    EXPECT_EQ(manifest["snapshot_period_ms"], "100");
    EXPECT_EQ(manifest["snapshot_count"], 61);
    ASSERT_EQ(manifest["robots"].size(), 1U);
    EXPECT_EQ(manifest["robots"][0]["robot_id"], 1);
    EXPECT_EQ(manifest["robots"][0]["name"], "robot_a");

    EXPECT_EQ(exported.snapshots.front()["tick_ms"], "0");
    EXPECT_EQ(exported.snapshots.back()["tick_ms"], "6000");
}

// Snapshot truth is compared against the KNOWN fixture geometry at the exact
// represented ticks (the movement timing contract: A->B over ticks 0..2000,
// corner at B, B->C over 2000..4000, then at rest at C, ADR-010/018).
TEST(IsaacExportTest, SnapshotTruthMatchesKnownFixtureGeometry) {
    const fleet::scenario::Scenario scenario = load_json(replay_scenario_json());
    ExportOptions options = default_options("fleet_isaac_export_truth");
    (void)fleet::isaac::export_replay_run(fixture().base, scenario, options);
    const ExportBasics exported = parse_export(options.output_dir);

    ASSERT_EQ(exported.snapshots.size(), 61U);
    const auto truth_at = [&exported](std::size_t index) -> const nlohmann::json& {
        return exported.snapshots[index]["robots"][0]["truth"];
    };
    const auto belief_at = [&exported](std::size_t index) -> const nlohmann::json& {
        return exported.snapshots[index]["robots"][0]["belief"];
    };

    // Tick 0: departed A at fraction 0 (or at rest at A) — node A exactly,
    // facing north (the initial physical orientation).
    EXPECT_DOUBLE_EQ(truth_at(0)["latitude_deg"].get<double>(), 52.370);
    EXPECT_DOUBLE_EQ(truth_at(0)["longitude_deg"].get<double>(), 9.730);
    EXPECT_NEAR(truth_at(0)["heading_rad"].get<double>(), 0.0, 1e-12);

    // Tick 3000: midpoint of the due-east B->C segment. The heading is the
    // geodesic INITIAL bearing of the segment: at this latitude it sits a
    // couple of microradians under the planar pi/2 — that curvature is the
    // domain's real truth-pose arithmetic, not an error.
    EXPECT_NEAR(truth_at(30)["latitude_deg"].get<double>(), 52.3702, 1e-12);
    EXPECT_NEAR(truth_at(30)["longitude_deg"].get<double>(), (9.730 + 9.7303) / 2.0, 1e-12);
    EXPECT_NEAR(truth_at(30)["heading_rad"].get<double>(), kPiOverTwo, 1e-5);

    // Tick 6000: at rest at C, stationary orientation = the completed
    // traversal's final-segment bearing (east) — never a north reset.
    EXPECT_NEAR(truth_at(60)["latitude_deg"].get<double>(), 52.3702, 1e-12);
    EXPECT_NEAR(truth_at(60)["longitude_deg"].get<double>(), 9.7303, 1e-12);
    EXPECT_NEAR(truth_at(60)["heading_rad"].get<double>(), kPiOverTwo, 1e-5);

    // Belief: tick 0 already carries the tick-0 perfect fix (ADR-018
    // ordering: the sample runs at tick 0); a perfect fix at tick 6000
    // matches truth; no dead reckoning is configured.
    ASSERT_FALSE(belief_at(0).is_null());
    EXPECT_EQ(belief_at(0)["estimated_at_ms"], "0");
    EXPECT_FALSE(belief_at(0)["dead_reckoned"].get<bool>());
    ASSERT_FALSE(belief_at(60).is_null());
    EXPECT_EQ(belief_at(60)["estimated_at_ms"], "6000");
    EXPECT_NEAR(belief_at(60)["latitude_deg"].get<double>(),
                truth_at(60)["latitude_deg"].get<double>(), 1e-12);
    EXPECT_NEAR(belief_at(60)["longitude_deg"].get<double>(),
                truth_at(60)["longitude_deg"].get<double>(), 1e-12);
    EXPECT_NEAR(belief_at(60)["heading_rad"].get<double>(),
                truth_at(60)["heading_rad"].get<double>(), 1e-12);
}

TEST(IsaacExportTest, RobotsEmittedInAscendingNumericIdOrder) {
    const std::string json = R"json({
      "name": "isaac_order",
      "seed": 3,
      "movement": { "ms_per_cost_unit": 1000 },
      "duration_ms": 2000,
      "robots": [
        { "name": "robot_gamma", "id": 3, "endpoint": 3, "mission": { "start": "A", "goal": "B" } },
        { "name": "robot_alpha", "id": 1, "endpoint": 1, "mission": { "start": "B", "goal": "C" } },
        { "name": "robot_beta",  "id": 2, "endpoint": 2, "mission": { "start": "C", "goal": "B" } }
      ],
      "events": []
    })json";
    const fleet::scenario::Scenario scenario = load_json(json);

    ExportOptions options;
    options.resolved_seed = 3;
    options.period_ms = 500;
    options.output_dir = fresh_dir("fleet_isaac_export_order");
    options.scenario_json = json;
    (void)fleet::isaac::export_replay_run(fixture().base, scenario, options);
    const ExportBasics exported = parse_export(options.output_dir);

    ASSERT_EQ(exported.manifest["robots"].size(), 3U);
    EXPECT_EQ(exported.manifest["robots"][0]["robot_id"], 1);
    EXPECT_EQ(exported.manifest["robots"][0]["name"], "robot_alpha");
    EXPECT_EQ(exported.manifest["robots"][1]["robot_id"], 2);
    EXPECT_EQ(exported.manifest["robots"][2]["robot_id"], 3);

    for (const nlohmann::json& snapshot : exported.snapshots) {
        ASSERT_EQ(snapshot["robots"].size(), 3U);
        EXPECT_LT(snapshot["robots"][0]["robot_id"].get<int>(),
                  snapshot["robots"][1]["robot_id"].get<int>());
        EXPECT_LT(snapshot["robots"][1]["robot_id"].get<int>(),
                  snapshot["robots"][2]["robot_id"].get<int>());
        EXPECT_EQ(snapshot["robots"][0]["name"], "robot_alpha");
    }
}

TEST(IsaacExportTest, MissingGeometryFailsExplicitly) {
    const fleet::testsupport::GridMap grid = fleet::testsupport::make_grid_map();
    const std::string json = R"json({
      "name": "isaac_no_geometry",
      "seed": 1,
      "movement": { "ms_per_cost_unit": 1000 },
      "duration_ms": 1000,
      "robots": [
        { "name": "robot_a", "id": 1, "endpoint": 1, "mission": { "start": "A", "goal": "B" } }
      ],
      "events": []
    })json";
    const fleet::scenario::Scenario scenario =
        fleet::scenario::ScenarioLoader::load_string(grid.base, json);

    ExportOptions options;
    options.resolved_seed = 1;
    options.period_ms = 500;
    options.output_dir = fresh_dir("fleet_isaac_export_nogeometry");
    options.scenario_json = json;
    EXPECT_THROW(fleet::isaac::export_replay_run(grid.base, scenario, options),
                 std::invalid_argument);
}

TEST(IsaacExportTest, RepeatExportsAreByteIdentical) {
    const fleet::scenario::Scenario scenario = load_json(replay_scenario_json());
    const std::filesystem::path first_dir = fresh_dir("fleet_isaac_export_repeat1");
    const std::filesystem::path second_dir = fresh_dir("fleet_isaac_export_repeat2");

    ExportOptions first = default_options("fleet_isaac_export_repeat1");
    ExportOptions second = default_options("fleet_isaac_export_repeat2");
    (void)fleet::isaac::export_replay_run(fixture().base, scenario, first);
    (void)fleet::isaac::export_replay_run(fixture().base, scenario, second);

    EXPECT_EQ(read_text(first_dir / "manifest.json"), read_text(second_dir / "manifest.json"));
    EXPECT_EQ(read_text(first_dir / "poses.jsonl"), read_text(second_dir / "poses.jsonl"));
}

// The export must not change what a plain run would emit: the stepped run's
// trace is byte-identical to a one-shot run of the same scenario/seed.
TEST(IsaacExportTest, SteppedExportLeavesNormalTraceByteIdentical) {
    const std::string json = replay_scenario_json();
    const fleet::scenario::Scenario scenario = load_json(json);

    std::ostringstream plain_buffer;
    fleet::scenario::JsonlTraceSink plain_sink{plain_buffer};
    fleet::scenario::ScenarioRunner plain_runner{fixture().base, scenario, 17};
    plain_runner.add_sink(plain_sink);
    plain_runner.run_to_completion();

    std::ostringstream stepped_buffer;
    fleet::scenario::JsonlTraceSink stepped_sink{stepped_buffer};
    ExportOptions options = default_options("fleet_isaac_export_trace");
    options.sinks.push_back(&stepped_sink);
    (void)fleet::isaac::export_replay_run(fixture().base, scenario, options);

    EXPECT_EQ(stepped_buffer.str(), plain_buffer.str());
}

TEST(IsaacExportTest, BeliefIsNullWithoutLocalization) {
    const std::string json = R"json({
      "name": "isaac_no_gnss",
      "seed": 5,
      "movement": { "ms_per_cost_unit": 1000 },
      "duration_ms": 2000,
      "robots": [
        { "name": "robot_a", "id": 1, "endpoint": 1, "mission": { "start": "A", "goal": "C" } }
      ],
      "events": []
    })json";
    const fleet::scenario::Scenario scenario = load_json(json);

    ExportOptions options;
    options.resolved_seed = 5;
    options.period_ms = 500;
    options.output_dir = fresh_dir("fleet_isaac_export_nognss");
    options.scenario_json = json;
    (void)fleet::isaac::export_replay_run(fixture().base, scenario, options);

    const ExportBasics exported = parse_export(options.output_dir);
    ASSERT_EQ(exported.snapshots.size(), 5U);
    for (const nlohmann::json& snapshot : exported.snapshots) {
        // No estimate before the first fix: null belief, not a zero pose.
        EXPECT_TRUE(snapshot["robots"][0]["belief"].is_null());
        ASSERT_TRUE(snapshot["robots"][0]["truth"].is_object());
    }
}

TEST(IsaacExportTest, MisalignedOrZeroPeriodRejected) {
    const fleet::scenario::Scenario scenario = load_json(replay_scenario_json());
    EXPECT_THROW(fleet::isaac::export_replay_run(
                     fixture().base, scenario, default_options("fleet_isaac_export_p7", 17, 7)),
                 std::invalid_argument);
    EXPECT_THROW(fleet::isaac::export_replay_run(
                     fixture().base, scenario, default_options("fleet_isaac_export_p0", 17, 0)),
                 std::invalid_argument);
}

TEST(IsaacExportTest, FootprintViolationRejectedBeforeAnyRun) {
    const fleet::scenario::Scenario scenario = load_json(replay_scenario_json());
    ExportOptions options = default_options("fleet_isaac_export_small");
    options.footprint_radius_m = 10.0;  // node C sits ~30 m out
    EXPECT_THROW(fleet::isaac::export_replay_run(fixture().base, scenario, options),
                 std::invalid_argument);
    EXPECT_FALSE(std::filesystem::exists(options.output_dir / "poses.jsonl"));
}

TEST(ScenarioRunnerTruthPoseTest, RequiresBeginAndKnownRobot) {
    const fleet::scenario::Scenario scenario = load_json(replay_scenario_json());
    fleet::scenario::ScenarioRunner runner{fixture().base, scenario, 17};
    EXPECT_THROW((void)runner.truth_pose_for("robot_a"), std::logic_error);
    runner.run_until(fleet::common::Tick{0});
    EXPECT_THROW((void)runner.truth_pose_for("does_not_exist"), std::invalid_argument);
    const fleet::localization::GroundTruthPose pose = runner.truth_pose_for("robot_a");
    EXPECT_DOUBLE_EQ(pose.position.latitude_deg, 52.370);
    EXPECT_DOUBLE_EQ(pose.position.longitude_deg, 9.730);
    EXPECT_NEAR(pose.heading_rad, 0.0, 1e-12);
    EXPECT_EQ(pose.at.value, 0U);
}

}  // namespace

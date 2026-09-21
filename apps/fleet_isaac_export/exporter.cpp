#include "exporter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/localization/pose.hpp"
#include "fleet/scenario/scenario_runner.hpp"

namespace fleet::isaac {
namespace {

constexpr std::string_view kSchema = "fleet-isaac-replay/1";

// FNV-1a 64 (correlation hashes only, ADR-020).
constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

// Metadata-side earth radius: matches the pinned conversion-layer constant
// (tools/isaac/conversion.py, ADR-020). Used only for the footprint extent
// check/record, never for pose derivation.
constexpr double kEarthRadiusM = 6371008.8;
constexpr double kPi = 3.14159265358979323846;

void hash_bytes(std::uint64_t& hash, std::string_view bytes) {
    for (const char byte : bytes) {
        hash ^= static_cast<std::uint8_t>(byte);
        hash *= kFnvPrime;
    }
}

void hash_u64(std::uint64_t& hash, std::uint64_t value) {
    hash_bytes(hash, std::to_string(value));
    hash_bytes(hash, "|");
}

// Hexfloat: an exact, toolchain-independent textual form of a double.
void hash_double(std::uint64_t& hash, double value) {
    hash_bytes(hash, std::format("{:a}", value));
    hash_bytes(hash, "|");
}

[[nodiscard]] std::string hash_label(std::uint64_t value) {
    return std::format("fnv1a64:{:016x}", value);
}

[[nodiscard]] double to_radians(double degrees) {
    return degrees * kPi / 180.0;
}

// Great-circle distance from the origin (haversine). Bounded-fixture math:
// sub-millimeter spherical deviations are covered by the conversion layer's
// own footprint check.
[[nodiscard]] double distance_from_origin_m(double latitude_deg, double longitude_deg,
                                            double origin_latitude_deg,
                                            double origin_longitude_deg) {
    const double lat1 = to_radians(origin_latitude_deg);
    const double lat2 = to_radians(latitude_deg);
    const double dlat = to_radians(latitude_deg - origin_latitude_deg);
    const double dlon = to_radians(longitude_deg - origin_longitude_deg);
    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) *
                         std::sin(dlon / 2.0);
    return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(a)));
}

}  // namespace

std::uint64_t bytes_hash(std::string_view bytes) {
    std::uint64_t hash = kFnvOffsetBasis;
    hash_bytes(hash, bytes);
    return hash;
}

std::uint64_t map_hash(const map::BaseMap& base) {
    std::uint64_t hash = kFnvOffsetBasis;
    hash_bytes(hash, "fleet-map-v1;");

    const map::Graph& graph = base.graph();
    hash_u64(hash, graph.node_count());
    for (const map::Node& node : graph.nodes()) {
        hash_u64(hash, node.id.value());
        hash_bytes(hash, node.name);
        hash_bytes(hash, ";");
        hash_double(hash, node.position.x);
        hash_double(hash, node.position.y);
    }
    hash_u64(hash, graph.edge_count());
    for (const map::Edge& edge : graph.edges()) {
        hash_u64(hash, edge.id.value());
        hash_u64(hash, edge.a.value());
        hash_u64(hash, edge.b.value());
        hash_u64(hash, static_cast<std::uint64_t>(edge.direction));
        hash_double(hash, edge.base_cost);
    }
    hash_u64(hash, base.version().value());

    const map::MapGeometry* geometry = base.geometry();
    if (geometry == nullptr) {
        hash_bytes(hash, "no-geometry");
        return hash;
    }
    hash_u64(hash, static_cast<std::uint64_t>(geometry->crs()));
    for (std::size_t index = 0; index < geometry->node_count(); ++index) {
        const common::NodeId node{static_cast<std::uint32_t>(index)};
        const map::Wgs84Coordinate* position = geometry->node_position(node);
        if (position == nullptr) {
            hash_bytes(hash, "none;");
        } else {
            hash_double(hash, position->latitude_deg);
            hash_double(hash, position->longitude_deg);
        }
    }
    for (std::size_t index = 0; index < geometry->edge_count(); ++index) {
        const common::EdgeId edge{static_cast<std::uint32_t>(index)};
        const std::vector<map::Wgs84Coordinate>* polyline = geometry->edge_polyline(edge);
        if (polyline == nullptr) {
            hash_bytes(hash, "none;");
            continue;
        }
        hash_u64(hash, polyline->size());
        for (const map::Wgs84Coordinate& point : *polyline) {
            hash_double(hash, point.latitude_deg);
            hash_double(hash, point.longitude_deg);
        }
    }
    return hash;
}

ExportSummary export_replay_run(const map::BaseMap& base, const scenario::Scenario& scenario,
                                const ExportOptions& options) {
    const map::MapGeometry* geometry = base.geometry();
    if (geometry == nullptr) {
        throw std::invalid_argument(
            "isaac export: the map carries no geographic geometry (truth pose is derived "
            "from it)");
    }
    if (!scenario.duration_ms.has_value()) {
        throw std::invalid_argument(
            "isaac export: the scenario declares no duration_ms horizon");
    }
    if (options.period_ms == 0) {
        throw std::invalid_argument("isaac export: snapshot period must be >= 1 ms");
    }
    if (*scenario.duration_ms % options.period_ms != 0) {
        throw std::invalid_argument(std::format(
            "isaac export: duration_ms {} is not a multiple of the snapshot period {} ms",
            *scenario.duration_ms, options.period_ms));
    }
    if (!std::isfinite(options.footprint_radius_m) || options.footprint_radius_m <= 0.0) {
        throw std::invalid_argument(
            "isaac export: footprint radius must be finite and positive");
    }

    // Up-front footprint precondition over ALL node coordinates: fail before
    // any run or output exists. A missing coordinate is a failure — a zero
    // coordinate is never manufactured (ADR-020).
    for (const map::Node& node : base.graph().nodes()) {
        const map::Wgs84Coordinate* position = geometry->node_position(node.id);
        if (position == nullptr) {
            throw std::invalid_argument(std::format(
                "isaac export: map node '{}' has no geographic coordinate", node.name));
        }
        const double distance_m =
            distance_from_origin_m(position->latitude_deg, position->longitude_deg,
                                   options.origin_latitude_deg, options.origin_longitude_deg);
        if (distance_m > options.footprint_radius_m) {
            throw std::invalid_argument(
                std::format("isaac export: map node '{}' lies {:.3f} m from the origin, "
                            "beyond the {:.3f} m footprint",
                            node.name, distance_m, options.footprint_radius_m));
        }
    }

    // Ascending numeric RobotId order; equal ids keep declaration order
    // (ADR-020).
    std::vector<std::size_t> order(scenario.robots.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(),
                     [&scenario](std::size_t lhs, std::size_t rhs) {
                         return scenario.robots[lhs].id.value() <
                                scenario.robots[rhs].id.value();
                     });

    std::error_code error_code;
    std::filesystem::create_directories(options.output_dir, error_code);
    if (error_code) {
        throw std::runtime_error(
            std::format("isaac export: cannot create output directory '{}': {}",
                        options.output_dir.string(), error_code.message()));
    }

    // The run itself: normal public ScenarioRunner API, the caller's trace
    // sinks attached exactly like fleet_sim attaches its own (ADR-020).
    scenario::ScenarioRunner runner{base, scenario, options.resolved_seed};
    for (scenario::TraceSink* sink : options.sinks) {
        runner.add_sink(*sink);
    }
    runner.begin();

    nlohmann::ordered_json robots_json = nlohmann::ordered_json::array();
    for (const std::size_t index : order) {
        robots_json.push_back(nlohmann::ordered_json{
            {"robot_id", scenario.robots[index].id.value()},
            {"name", scenario.robots[index].name}});
    }

    const std::filesystem::path poses_path = options.output_dir / "poses.jsonl";
    std::ofstream poses_file(poses_path, std::ios::out | std::ios::trunc);
    if (!poses_file.is_open()) {
        throw std::runtime_error(
            std::format("isaac export: cannot write '{}'", poses_path.string()));
    }

    // Snapshot loop: tick = step * period, so the final horizon is reached
    // exactly (duration is a validated multiple of the period) and no
    // uint64 accumulation can wrap.
    const std::uint64_t final_tick = *scenario.duration_ms;
    const std::uint64_t step_count = final_tick / options.period_ms;
    std::uint64_t snapshot_count = 0;
    double observed_extent_m = 0.0;
    for (std::uint64_t step = 0; step <= step_count; ++step) {
        const std::uint64_t tick = step * options.period_ms;
        runner.run_until(common::Tick{tick});

        nlohmann::ordered_json line;
        line["schema"] = kSchema;
        line["tick_ms"] = std::to_string(tick);
        nlohmann::ordered_json entries = nlohmann::ordered_json::array();
        for (const std::size_t index : order) {
            const scenario::ScenarioRobot& declaration = scenario.robots[index];

            const localization::GroundTruthPose truth = runner.truth_pose_for(declaration.name);
            observed_extent_m =
                std::max(observed_extent_m,
                         distance_from_origin_m(truth.position.latitude_deg,
                                                truth.position.longitude_deg,
                                                options.origin_latitude_deg,
                                                options.origin_longitude_deg));

            nlohmann::ordered_json entry;
            entry["robot_id"] = declaration.id.value();
            entry["name"] = declaration.name;
            entry["truth"] = nlohmann::ordered_json{
                {"latitude_deg", truth.position.latitude_deg},
                {"longitude_deg", truth.position.longitude_deg},
                {"heading_rad", truth.heading_rad}};

            // Belief is the robot-local retained estimate read through the
            // existing read-only view: JSON null before the first fix — an
            // absent estimate is never a marker at the origin (ADR-020).
            const localization::LocalizationTracker& tracker =
                runner.robot(declaration.name).localization();
            if (tracker.estimate().has_value()) {
                const localization::LocalizationEstimate& estimate = *tracker.estimate();
                entry["belief"] = nlohmann::ordered_json{
                    {"latitude_deg", estimate.position.latitude_deg},
                    {"longitude_deg", estimate.position.longitude_deg},
                    {"heading_rad", estimate.heading_rad},
                    {"estimated_at_ms", std::to_string(estimate.estimated_at.value)},
                    {"dead_reckoned", tracker.dead_reckoned()}};
            } else {
                entry["belief"] = nullptr;
            }
            entries.push_back(std::move(entry));
        }
        line["robots"] = std::move(entries);
        poses_file << line.dump() << '\n';
        ++snapshot_count;
    }
    if (!poses_file.good()) {
        throw std::runtime_error("isaac export: failed while writing poses.jsonl");
    }
    poses_file.close();

    nlohmann::ordered_json manifest;
    manifest["schema"] = kSchema;
    manifest["kind"] = "readonly-replay-export";
    manifest["scenario_name"] = scenario.name;
    manifest["resolved_seed"] = std::to_string(options.resolved_seed);
    manifest["scenario_hash"] = hash_label(bytes_hash(options.scenario_json));
    manifest["map_hash"] = hash_label(map_hash(base));
    manifest["origin"] = nlohmann::ordered_json{
        {"latitude_deg", options.origin_latitude_deg},
        {"longitude_deg", options.origin_longitude_deg}};
    manifest["scene_frame"] = nlohmann::ordered_json{{"axes", "enu"},
                                                     {"translation_units", "meters"},
                                                     {"z_up", true}};
    manifest["heading_convention"] = "radians_clockwise_from_north";
    manifest["footprint_radius_m"] = options.footprint_radius_m;
    manifest["observed_extent_m"] = observed_extent_m;
    manifest["tick_unit_ms"] = 1;
    manifest["first_tick_ms"] = "0";
    manifest["final_tick_ms"] = std::to_string(final_tick);
    manifest["snapshot_period_ms"] = std::to_string(options.period_ms);
    manifest["snapshot_count"] = snapshot_count;
    manifest["robots"] = std::move(robots_json);

    const std::filesystem::path manifest_path = options.output_dir / "manifest.json";
    std::ofstream manifest_file(manifest_path, std::ios::out | std::ios::trunc);
    if (!manifest_file.is_open()) {
        throw std::runtime_error(
            std::format("isaac export: cannot write '{}'", manifest_path.string()));
    }
    manifest_file << manifest.dump(2) << '\n';
    if (!manifest_file.good()) {
        throw std::runtime_error("isaac export: failed while writing manifest.json");
    }

    ExportSummary summary;
    summary.first_tick_ms = 0;
    summary.last_tick_ms = final_tick;
    summary.snapshot_count = snapshot_count;
    summary.robot_count = scenario.robots.size();
    summary.observed_extent_m = observed_extent_m;
    summary.manifest_path = manifest_path;
    summary.poses_path = poses_path;
    return summary;
}

}  // namespace fleet::isaac

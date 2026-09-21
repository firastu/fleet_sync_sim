#include "fleet/scenario/scenario_loader.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/network/probability.hpp"

namespace fleet::scenario {

namespace {

using json = nlohmann::json;

[[nodiscard]] std::invalid_argument load_error(const std::string& message) {
    return std::invalid_argument("scenario: " + message);
}

[[nodiscard]] const json& require(const json& object, const char* key, const char* where) {
    const auto entry = object.find(key);
    if (entry == object.end()) {
        throw load_error(std::format("{}: missing field '{}'", where, key));
    }
    return *entry;
}

[[nodiscard]] std::uint64_t require_ms(const json& object, const char* key, const char* where) {
    const json& value = require(object, key, where);
    if (!value.is_number_unsigned()) {
        throw load_error(std::format("{}: '{}' must be a non-negative integer (ms)", where, key));
    }
    return value.get<std::uint64_t>();
}

// Small non-negative integer (ids, endpoints) that must fit a byte without
// silent truncation.
[[nodiscard]] std::uint64_t require_small_int(const json& object, const char* key,
                                              const char* where) {
    const json& value = require(object, key, where);
    if (!value.is_number_unsigned()) {
        throw load_error(std::format("{}: '{}' must be a non-negative integer", where, key));
    }
    const std::uint64_t number = value.get<std::uint64_t>();
    if (number > 255) {
        throw load_error(std::format("{}: '{}' must not exceed 255", where, key));
    }
    return number;
}

[[nodiscard]] std::string require_string(const json& object, const char* key, const char* where) {
    const json& value = require(object, key, where);
    if (!value.is_string()) {
        throw load_error(std::format("{}: '{}' must be a string", where, key));
    }
    return value.get<std::string>();
}

[[nodiscard]] bool require_bool(const json& object, const char* key, const char* where) {
    const json& value = require(object, key, where);
    if (!value.is_boolean()) {
        throw load_error(std::format("{}: '{}' must be a boolean", where, key));
    }
    return value.get<bool>();
}

[[nodiscard]] common::NodeId node_named(const map::Graph& graph, const std::string& name) {
    for (const map::Node& node : graph.nodes()) {
        if (node.name == name) {
            return node.id;
        }
    }
    throw load_error(std::format("unknown node '{}'", name));
}

[[nodiscard]] common::EdgeId edge_named(const map::Graph& graph, const std::string& name) {
    const auto separator = name.find('-');
    if (separator == std::string::npos) {
        throw load_error(std::format("invalid edge '{}': expected <node>-<node>", name));
    }
    const std::string first = name.substr(0, separator);
    const std::string second = name.substr(separator + 1);
    const auto edge = graph.edge_between(node_named(graph, first), node_named(graph, second));
    if (!edge.has_value()) {
        throw load_error(std::format("unknown edge '{}': nodes are not adjacent", name));
    }
    return *edge;
}

// Movement settings (ADR-010): presence of the "movement" object enables
// movement; both fields are optional with documented defaults.
[[nodiscard]] fleet::scenario::MovementSettings parse_movement(const json& root) {
    fleet::scenario::MovementSettings movement;
    const auto entry = root.find("movement");
    if (entry == root.end()) {
        return movement;  // absent = static fleet
    }
    if (!entry->is_object()) {
        throw load_error("'movement' must be an object");
    }
    movement.enabled = true;
    if (const auto per_cost = entry->find("ms_per_cost_unit"); per_cost != entry->end()) {
        if (!per_cost->is_number_unsigned() || per_cost->get<std::uint64_t>() == 0) {
            throw load_error("movement: 'ms_per_cost_unit' must be a positive integer");
        }
        movement.ms_per_cost_unit = per_cost->get<std::uint64_t>();
    }
    if (const auto retry = entry->find("retry_ms"); retry != entry->end()) {
        if (!retry->is_number_unsigned() || retry->get<std::uint64_t>() == 0) {
            throw load_error("movement: 'retry_ms' must be a positive integer");
        }
        movement.retry_ms = retry->get<std::uint64_t>();
    }
    return movement;
}

// Sensing settings (ADR-011): presence of the "sensing" object enables
// position-based sensing; "mode" names the observation model ("none" is
// the explicit off value). World evolution is INDEPENDENT of sensing
// configuration: set_world_edge_state is valid without any sensor, and
// robots simply remain unaware of what they cannot observe.
[[nodiscard]] fleet::scenario::SensingSettings parse_sensing(const json& root) {
    fleet::scenario::SensingSettings sensing;
    const auto entry = root.find("sensing");
    if (entry == root.end()) {
        return sensing;  // absent = no sensing
    }
    if (!entry->is_object()) {
        throw load_error("'sensing' must be an object");
    }
    const std::string mode = require_string(*entry, "mode", "sensing");
    if (mode == "none") {
        return sensing;  // explicitly off
    }
    if (mode != "perfect_local") {
        throw load_error(
            std::format("sensing: unknown mode '{}' (supported: none, perfect_local)",
                        mode));
    }
    sensing.enabled = true;
    return sensing;
}

// ONE parsing seam for GNSS model selection (#16, ADR-018): the same
// grammar serves the initial model and every set_gnss_model event —
//   "perfect" | "unavailable" | {"noisy": {"position_axis_sigma_m": d,
//                                          "heading_sigma_rad": d}}
// Noise fields are optional (default 0); semantic validation (finite,
// non-negative) happens where the model is constructed — the runner's
// single factory — so the grammar here stays purely syntactic.
[[nodiscard]] GnssModelSpec parse_gnss_model_spec(const json& model, const char* where) {
    GnssModelSpec spec;
    if (model.is_string()) {
        const std::string name = model.get<std::string>();
        if (name == "perfect") {
            spec.kind = GnssModelSpec::Kind::Perfect;
            return spec;
        }
        if (name == "unavailable") {
            spec.kind = GnssModelSpec::Kind::Unavailable;
            return spec;
        }
        throw load_error(std::format(
            "{}: unknown model '{}' (supported: perfect, unavailable, noisy)",
            where, name));
    }
    if (!model.is_object() || model.size() != 1 || !model.contains("noisy")) {
        throw load_error(std::format(
            "{}: model must be 'perfect', 'unavailable' or {{\"noisy\": {{...}}}}",
            where));
    }
    spec.kind = GnssModelSpec::Kind::Noisy;
    const json& noisy = model.at("noisy");
    if (!noisy.is_object()) {
        throw load_error(std::format("{}: 'noisy' must be an object", where));
    }
    if (const auto sigma = noisy.find("position_axis_sigma_m"); sigma != noisy.end()) {
        if (!sigma->is_number()) {
            throw load_error(
                std::format("{}: 'position_axis_sigma_m' must be a number", where));
        }
        spec.noise.position_axis_sigma_m = sigma->get<double>();
    }
    if (const auto sigma = noisy.find("heading_sigma_rad"); sigma != noisy.end()) {
        if (!sigma->is_number()) {
            throw load_error(
                std::format("{}: 'heading_sigma_rad' must be a number", where));
        }
        spec.noise.heading_sigma_rad = sigma->get<double>();
    }
    return spec;
}

// Localization settings (#16, ADR-018): presence of the "localization"
// object enables GNSS sampling. Opt-in — absent means no localization
// behavior at all, so pre-#16 scenarios (including the founding
// scenario) produce byte-identical traces.
[[nodiscard]] fleet::scenario::LocalizationSettings parse_localization(
    const json& root, const map::BaseMap& base) {
    fleet::scenario::LocalizationSettings localization;
    const auto entry = root.find("localization");
    if (entry == root.end()) {
        return localization;  // absent = localization off
    }
    if (!entry->is_object()) {
        throw load_error("'localization' must be an object");
    }
    localization.enabled = true;

    // Truth pose is DERIVED from map geometry (ADR-018) — a map without
    // a geographic side cannot host localization; fail clearly instead
    // of manufacturing coordinates.
    if (base.geometry() == nullptr) {
        throw load_error(
            "localization: the map carries no geographic geometry (node "
            "coordinates required to derive truth pose)");
    }

    const json& gnss = require(*entry, "gnss", "localization");
    if (!gnss.is_object()) {
        throw load_error("localization: 'gnss' must be an object");
    }
    if (const auto period = gnss.find("period_ms"); period != gnss.end()) {
        if (!period->is_number_unsigned() || period->get<std::uint64_t>() == 0) {
            throw load_error("localization: 'period_ms' must be a positive integer");
        }
        localization.gnss_period_ms = period->get<std::uint64_t>();
    }
    localization.initial_model = parse_gnss_model_spec(
        require(gnss, "initial_model", "localization"), "localization: initial_model");
    if (const auto propagation = entry->find("dead_reckoning"); propagation != entry->end()) {
        if (!propagation->is_object()) {
            throw load_error("localization: 'dead_reckoning' must be an object");
        }
        localization::DeadReckoningConfig config;
        for (const auto& [key, value] : propagation->items()) {
            if (!value.is_number()) {
                throw load_error("dead_reckoning: bias fields must be numbers");
            }
            if (key == "distance_scale_error") {
                config.distance_scale_error = value.get<double>();
            } else if (key == "heading_drift_rad_per_m") {
                config.heading_drift_rad_per_m = value.get<double>();
            } else {
                throw load_error(std::format("dead_reckoning: unknown field '{}'", key));
            }
        }
        config.validate();
        localization.dead_reckoning = config;
    }
    return localization;
}

[[nodiscard]] network::NetworkConfig parse_network(const json& root) {
    network::NetworkConfig config;
    const auto entry = root.find("network");
    if (entry == root.end()) {
        return config;  // defaults: fixed 80 ms latency, ideal link
    }
    if (!entry->is_object()) {
        throw load_error("'network' must be an object");
    }
    const std::uint64_t min_latency = require_ms(*entry, "min_latency_ms", "network");
    const std::uint64_t max_latency = require_ms(*entry, "max_latency_ms", "network");
    if (min_latency > max_latency) {
        throw load_error("network: min_latency_ms exceeds max_latency_ms");
    }
    config.min_latency = common::Tick{min_latency};
    config.max_latency = common::Tick{max_latency};

    if (const auto loss = entry->find("packet_loss_ppm"); loss != entry->end()) {
        if (!loss->is_number_unsigned() || loss->get<std::uint64_t>() > 1'000'000) {
            throw load_error("network: 'packet_loss_ppm' must be within [0, 1000000]");
        }
        config.packet_loss = network::Probability::from_parts_per_million(
            static_cast<std::uint32_t>(loss->get<std::uint64_t>()));
    }
    if (const auto duplication = entry->find("duplication_ppm"); duplication != entry->end()) {
        if (!duplication->is_number_unsigned() ||
            duplication->get<std::uint64_t>() > 1'000'000) {
            throw load_error("network: 'duplication_ppm' must be within [0, 1000000]");
        }
        config.duplication = network::Probability::from_parts_per_million(
            static_cast<std::uint32_t>(duplication->get<std::uint64_t>()));
    }
    return config;
}

}  // namespace

namespace {

void parse_events(const map::BaseMap& base, Scenario& scenario,
                  const std::map<std::string, network::EndpointId>& endpoints_by_name,
                  const json& events) {
    const auto endpoint_of = [&](const std::string& participant) {
        const auto found = endpoints_by_name.find(participant);
        if (found == endpoints_by_name.end()) {
            throw load_error(std::format("unknown participant '{}'", participant));
        }
        return found->second;
    };
    const auto robot_named = [&](const std::string& name) {
        for (const ScenarioRobot& robot : scenario.robots) {
            if (robot.name == name) {
                return robot.id;
            }
        }
        throw load_error(std::format("unknown robot '{}'", name));
    };

    for (const json& entry : events) {
        const std::uint64_t at = require_ms(entry, "at_ms", "event");
        const std::string action = require_string(entry, "action", "event");
        if (action == "set_link_state") {
            scenario.events.push_back(ScenarioEvent{
                common::Tick{at},
                SetLinkAction{.from = endpoint_of(require_string(entry, "from", "event")),
                              .to = endpoint_of(require_string(entry, "to", "event")),
                              .up = require_bool(entry, "up", "event")}});
        } else if (action == "observe_edge") {
            const std::string state = require_string(entry, "state", "event");
            map::EdgeStatus status{};
            if (state == "open") {
                status = map::EdgeStatus::Open;
            } else if (state == "blocked") {
                status = map::EdgeStatus::Blocked;
            } else {
                throw load_error(std::format(
                    "event: invalid state '{}' (expected 'open' or 'blocked')", state));
            }
            double confidence = 1.0;
            if (const auto field = entry.find("confidence"); field != entry.end()) {
                if (!field->is_number()) {
                    throw load_error("event: 'confidence' must be a number");
                }
                confidence = field->get<double>();
            }
            scenario.events.push_back(ScenarioEvent{
                common::Tick{at},
                ObserveEdgeAction{.robot = robot_named(require_string(entry, "robot", "event")),
                                  .edge = edge_named(base.graph(),
                                                     require_string(entry, "edge", "event")),
                                  .status = status,
                                  .confidence = confidence}});
        } else if (action == "resynchronize") {
            scenario.events.push_back(ScenarioEvent{
                common::Tick{at},
                ResynchronizeAction{
                    .robot = robot_named(require_string(entry, "robot", "event"))}});
        } else if (action == "set_world_edge_state") {
            // Valid regardless of sensing: the world evolves
            // independently of whether anyone can observe it (ADR-011);
            // without a sensor, robots simply remain unaware.
            const std::string state = require_string(entry, "state", "event");
            map::EdgeStatus status{};
            if (state == "open") {
                status = map::EdgeStatus::Open;
            } else if (state == "blocked") {
                status = map::EdgeStatus::Blocked;
            } else {
                throw load_error(std::format(
                    "event: invalid state '{}' (expected 'open' or 'blocked')", state));
            }
            scenario.events.push_back(ScenarioEvent{
                common::Tick{at},
                SetWorldEdgeStateAction{
                    .edge = edge_named(base.graph(),
                                       require_string(entry, "edge", "event")),
                    .status = status}});
        } else if (action == "set_gnss_model") {
            // Scenario policy (ADR-016/018): an outage is WHICH model is
            // active, never a special case elsewhere. The action is only
            // meaningful in a run that actually samples GNSS.
            if (!scenario.localization.enabled) {
                throw load_error(
                    "event: 'set_gnss_model' requires a 'localization' block");
            }
            scenario.events.push_back(ScenarioEvent{
                common::Tick{at},
                SetGnssModelAction{
                    .robot = robot_named(require_string(entry, "robot", "event")),
                    .model = parse_gnss_model_spec(
                        require(entry, "model", "event"), "event: model")}});
        } else {
            throw load_error(std::format(
                "unknown action '{}' (supported: set_link_state, observe_edge, "
                "resynchronize, set_world_edge_state, set_gnss_model)",
                action));
        }
    }
}

[[nodiscard]] Scenario parse_scenario(const map::BaseMap& base, const json& root) {
    if (!root.is_object()) {
        throw load_error("top-level value must be an object");
    }

    Scenario scenario;
    scenario.name = require_string(root, "name", "top-level");
    if (const auto seed = root.find("seed"); seed != root.end()) {
        if (!seed->is_number_unsigned()) {
            throw load_error("'seed' must be a non-negative integer");
        }
        scenario.seed = seed->get<std::uint64_t>();
    }
    scenario.network = parse_network(root);
    scenario.movement = parse_movement(root);
    scenario.sensing = parse_sensing(root);
    scenario.localization = parse_localization(root, base);
    if (const auto duration = root.find("duration_ms"); duration != root.end()) {
        if (!duration->is_number_unsigned()) {
            throw load_error("'duration_ms' must be a non-negative integer (ms)");
        }
        scenario.duration_ms = duration->get<std::uint64_t>();
    }
    if (scenario.movement.enabled && !scenario.duration_ms.has_value()) {
        // Termination guarantee (ADR-010): a robot whose goal becomes
        // unreachable parks and retries forever; the horizon bounds the run.
        throw load_error("movement: 'movement' requires 'duration_ms' to bound the run");
    }
    if (scenario.localization.enabled && !scenario.duration_ms.has_value()) {
        // Same guarantee for the GNSS sampling chain (ADR-018): it
        // self-reschedules forever; without a horizon the run never ends.
        throw load_error(
            "localization: 'localization' requires 'duration_ms' to bound the run");
    }

    if (const auto station = root.find("station"); station != root.end()) {
        scenario.has_station = true;
        scenario.station_endpoint = network::EndpointId{static_cast<std::uint8_t>(
            require_small_int(*station, "endpoint", "station"))};
    }

    const json& robots = require(root, "robots", "top-level");
    if (!robots.is_array() || robots.empty()) {
        throw load_error("'robots' must be a non-empty array");
    }
    for (const json& entry : robots) {
        ScenarioRobot robot;
        robot.name = require_string(entry, "name", "robot");
        robot.id = common::RobotId{
            static_cast<std::uint8_t>(require_small_int(entry, "id", robot.name.c_str()))};
        robot.endpoint = network::EndpointId{
            static_cast<std::uint8_t>(require_small_int(entry, "endpoint", robot.name.c_str()))};
        const json& mission = require(entry, "mission", robot.name.c_str());
        robot.mission.start =
            node_named(base.graph(), require_string(mission, "start", "mission"));
        robot.mission.goal = node_named(base.graph(), require_string(mission, "goal", "mission"));
        scenario.robots.push_back(std::move(robot));
    }

    std::map<std::string, network::EndpointId> endpoints_by_name;
    std::map<common::RobotId, std::string> names_by_robot;
    for (const ScenarioRobot& robot : scenario.robots) {
        if (!endpoints_by_name.emplace(robot.name, robot.endpoint).second) {
            throw load_error(std::format("duplicate robot name '{}'", robot.name));
        }
        if (!names_by_robot.emplace(robot.id, robot.name).second) {
            throw load_error(std::format("duplicate robot id {}", robot.id.value()));
        }
        for (const ScenarioRobot& other : scenario.robots) {
            if (&robot != &other && robot.endpoint == other.endpoint) {
                throw load_error(std::format("robots '{}' and '{}' share an endpoint", robot.name,
                                             other.name));
            }
        }
        if (scenario.has_station && robot.endpoint == scenario.station_endpoint) {
            throw load_error(std::format("robot '{}' reuses the station endpoint", robot.name));
        }
    }
    if (scenario.has_station) {
        endpoints_by_name.emplace("station", scenario.station_endpoint);
    }

    const json& events = require(root, "events", "top-level");
    if (!events.is_array()) {
        throw load_error("'events' must be an array");
    }
    parse_events(base, scenario, endpoints_by_name, events);

    // Stable sort: equal-tick events keep file order, which becomes
    // execution order via ADR-005 enqueue order.
    std::stable_sort(scenario.events.begin(), scenario.events.end(),
                     [](const ScenarioEvent& a, const ScenarioEvent& b) { return a.at < b.at; });
    return scenario;
}

}  // namespace

Scenario ScenarioLoader::load(const map::BaseMap& base, const std::filesystem::path& file) {
    std::ifstream input{file};
    if (!input) {
        throw load_error(std::format("cannot open scenario file '{}'", file.string()));
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    try {
        return parse_scenario(base, json::parse(buffer.str()));
    } catch (const json::parse_error& error) {
        throw load_error(
            std::format("JSON parse error in '{}': {}", file.string(), error.what()));
    }
}

Scenario ScenarioLoader::load_string(const map::BaseMap& base, const std::string& json_text) {
    try {
        return parse_scenario(base, json::parse(json_text));
    } catch (const json::parse_error& error) {
        throw load_error(std::format("JSON parse error: {}", error.what()));
    }
}

}  // namespace fleet::scenario

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
        } else {
            throw load_error(std::format(
                "unknown action '{}' (supported: set_link_state, observe_edge, resynchronize)",
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

// fleet_console — interactive console over the ScenarioRunner (#13).
//
//   fleet_console --scenario <file> [--seed <uint64>]
//
// A thin REPL over the library's public APIs: time stepping
// (ScenarioRunner::run_until), event injection (inject), and state
// inspection (Robot/ControlStation accessors). The console owns no
// simulation semantics of its own; injected events run through the
// exact same effect path as loaded scenario events.
//
// Commands:
//   run <ms> | until <ms> | finish | status | robots | robot <name>
//   station | observe <robot> <A-B> <open|blocked> [conf]
//   world <A-B> <open|blocked> | link <from> <to> <up|down>
//   resync <robot> | help | quit

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "demo_map.hpp"

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/graph.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::NodeId;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::EdgeStatus;
using fleet::network::EndpointId;
using fleet::scenario::ObserveEdgeAction;
using fleet::scenario::ResynchronizeAction;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioEvent;
using fleet::scenario::ScenarioRunner;
using fleet::scenario::SetLinkAction;
using fleet::scenario::SetWorldEdgeStateAction;

class Console {
public:
    Console(const fleet::app::DemoMap& map, Scenario scenario, std::uint64_t seed)
        : scenario_{std::move(scenario)},
          runner_{map.base, scenario_, seed},
          map_{&map.base} {
        for (const auto& robot : scenario_.robots) {
            endpoints_[robot.name] = robot.endpoint;
        }
        if (scenario_.has_station) {
            endpoints_["station"] = scenario_.station_endpoint;
        }
    }

    void run() {
        runner_.add_sink(console_sink_);
        std::cout << std::format("fleet_console: scenario '{}' — 'help' for commands\n",
                                 scenario_.name);
        std::string line;
        while (std::cout << "> " && std::getline(std::cin, line)) {
            if (line == "quit") {
                break;
            }
            execute(line);
        }
    }

private:
    void execute(const std::string& line) {
        std::istringstream input{line};
        std::string command;
        input >> command;
        try {
            if (command.empty() || command == "help") {
                print_help();
            } else if (command == "run" || command == "until") {
                std::uint64_t tick = 0;
                if (!(input >> tick)) {
                    throw std::invalid_argument("expected a tick");
                }
                const Tick target =
                    command == "run" ? Tick{runner_.now().value + tick} : Tick{tick};
                runner_.run_until(target);
                std::cout << std::format("t={}\n", runner_.now().value);
            } else if (command == "finish") {
                const auto result = runner_.run_to_completion();
                std::cout << std::format("finished at t={}\n", result.finished_at.value);
            } else if (command == "status" || command == "robots") {
                print_robots();
            } else if (command == "robot") {
                std::string name;
                if (!(input >> name)) {
                    throw std::invalid_argument("expected a robot name");
                }
                print_robot(name);
            } else if (command == "station") {
                print_station();
            } else if (command == "observe") {
                inject_observe(input);
            } else if (command == "world") {
                inject_world(input);
            } else if (command == "link") {
                inject_link(input);
            } else if (command == "resync") {
                inject_resync(input);
            } else {
                std::cout << "unknown command (try 'help')\n";
            }
        } catch (const std::exception& error) {
            std::cout << std::format("error: {}\n", error.what());
        }
    }

    void print_help() {
        std::cout << "run <ms> | until <ms> | finish | status | robots | robot <name>\n"
                     "station | observe <robot> <edge> <open|blocked> [conf]\n"
                     "world <edge> <open|blocked> | link <from> <to> <up|down>\n"
                     "resync <robot> | help | quit\n";
    }

    [[nodiscard]] static std::string status_name(EdgeStatus status) {
        return status == EdgeStatus::Blocked ? "BLOCKED" : "OPEN";
    }

    void print_robots() {
        for (const auto& declaration : scenario_.robots) {
            const auto& robot = runner_.robot(declaration.name);
            const auto& state = robot.state();
            std::string where;
            if (state.mission_complete) {
                where = std::format("complete at {}", node_name(state.position));
            } else if (state.in_transit.has_value()) {
                where = std::format("{}->{} (arrives {})", node_name(state.in_transit->from),
                                    node_name(state.in_transit->to),
                                    state.in_transit->arrival.value);
            } else {
                where = std::format("at {}", node_name(state.position));
            }
            std::cout << std::format("{}: {} | known edges: {}\n", declaration.name, where,
                                     robot.overlay().tracked_count());
        }
    }

    void print_knowledge(const fleet::map::DynamicMapOverlay& overlay,
                         const std::string& title) {
        std::cout << title;
        const fleet::map::MapView view{*map_, overlay};
        for (const EdgeId edge : overlay.tracked_edges()) {
            const auto& state = *view.dynamic_state(edge);
            std::cout << std::format("  {} status={} observed_at={} source={} seq={}\n",
                                     edge_label(edge), status_name(state.status),
                                     state.observed_at.value, state.source.value(),
                                     state.source_sequence.value());
        }
    }

    void print_robot(const std::string& name) {
        const auto& robot = runner_.robot(name);  // throws for unknown names
        const auto& route = robot.current_route();
        std::cout << std::format("robot {} route: ", name);
        if (!route.found) {
            std::cout << "<no route>";
        } else {
            for (std::size_t i = 0; i < route.nodes.size(); ++i) {
                std::cout << (i > 0 ? "->" : "") << node_name(route.nodes[i]);
            }
            std::cout << std::format(" cost={:.2}", route.cost);
        }
        std::cout << '\n';
        print_knowledge(robot.overlay(), std::format("knowledge ({} edge(s)):\n",
                                                     robot.overlay().tracked_count()));
    }

    void print_station() {
        const auto* station = runner_.station();
        if (station == nullptr) {
            std::cout << "no station in this scenario\n";
            return;
        }
        print_knowledge(station->overlay(),
                        std::format("station knowledge ({} edge(s)):\n",
                                    station->overlay().tracked_count()));
    }

    void inject_observe(std::istringstream& input) {
        std::string robot;
        std::string edge_text;
        std::string state_text;
        double confidence = 1.0;
        if (!(input >> robot >> edge_text >> state_text)) {
            throw std::invalid_argument("usage: observe <robot> <A-B> <open|blocked> [conf]");
        }
        (void)(input >> confidence);
        runner_.inject(ScenarioEvent{runner_.now(),
                                     ObserveEdgeAction{robot_id(robot), edge_of(edge_text),
                                                       parse_status(state_text), confidence}});
    }

    void inject_world(std::istringstream& input) {
        std::string edge_text;
        std::string state_text;
        if (!(input >> edge_text >> state_text)) {
            throw std::invalid_argument("usage: world <A-B> <open|blocked>");
        }
        runner_.inject(ScenarioEvent{
            runner_.now(),
            SetWorldEdgeStateAction{edge_of(edge_text), parse_status(state_text)}});
    }

    void inject_link(std::istringstream& input) {
        std::string from;
        std::string to;
        std::string state_text;
        if (!(input >> from >> to >> state_text)) {
            throw std::invalid_argument("usage: link <from> <to> <up|down>");
        }
        const bool up = state_text == "up";
        if (!up && state_text != "down") {
            throw std::invalid_argument("link state must be 'up' or 'down'");
        }
        runner_.inject(ScenarioEvent{
            runner_.now(), SetLinkAction{endpoint_of(from), endpoint_of(to), up}});
    }

    void inject_resync(std::istringstream& input) {
        std::string robot;
        if (!(input >> robot)) {
            throw std::invalid_argument("usage: resync <robot>");
        }
        runner_.inject(ScenarioEvent{runner_.now(), ResynchronizeAction{robot_id(robot)}});
    }

    [[nodiscard]] static EdgeStatus parse_status(const std::string& text) {
        if (text == "open") {
            return EdgeStatus::Open;
        }
        if (text == "blocked") {
            return EdgeStatus::Blocked;
        }
        throw std::invalid_argument("state must be 'open' or 'blocked'");
    }

    [[nodiscard]] RobotId robot_id(const std::string& name) const {
        for (const auto& robot : scenario_.robots) {
            if (robot.name == name) {
                return RobotId{robot.id};
            }
        }
        throw std::invalid_argument("unknown robot '" + name + "'");
    }

    [[nodiscard]] EndpointId endpoint_of(const std::string& name) const {
        const auto found = endpoints_.find(name);
        if (found == endpoints_.end()) {
            throw std::invalid_argument("unknown participant '" + name + "'");
        }
        return found->second;
    }

    [[nodiscard]] EdgeId edge_of(const std::string& label) const {
        const auto separator = label.find('-');
        if (separator == std::string::npos) {
            throw std::invalid_argument("edge must be <node>-<node>");
        }
        const auto edge = map_->graph().edge_between(node_id(label.substr(0, separator)),
                                                     node_id(label.substr(separator + 1)));
        if (!edge.has_value()) {
            throw std::invalid_argument("no edge '" + label + "'");
        }
        return *edge;
    }

    [[nodiscard]] NodeId node_id(const std::string& name) const {
        for (const auto& node : map_->graph().nodes()) {
            if (node.name == name) {
                return node.id;
            }
        }
        throw std::invalid_argument("unknown node '" + name + "'");
    }

    [[nodiscard]] std::string node_name(NodeId id) const {
        return map_->graph().node(id).name;
    }

    [[nodiscard]] std::string edge_label(EdgeId edge) const {
        const auto& base_edge = map_->graph().edge(edge);
        return std::format("{}-{}", node_name(base_edge.a), node_name(base_edge.b));
    }

    Scenario scenario_;
    ScenarioRunner runner_;
    fleet::scenario::ConsoleTraceSink console_sink_{std::cout};
    std::map<std::string, EndpointId> endpoints_;
    const fleet::map::BaseMap* map_ = nullptr;
};

}  // namespace

int main(int argc, char* argv[]) {
    std::optional<std::filesystem::path> scenario_file;
    std::optional<std::uint64_t> seed;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--scenario" && i + 1 < argc) {
            scenario_file = argv[++i];
        } else if (argument == "--seed" && i + 1 < argc) {
            seed = std::stoull(argv[++i]);
        } else {
            std::cerr << "usage: fleet_console --scenario <file> [--seed <uint64>]\n";
            return 2;
        }
    }
    if (!scenario_file.has_value()) {
        std::cerr << "usage: fleet_console --scenario <file> [--seed <uint64>]\n";
        return 2;
    }

    try {
        const fleet::app::DemoMap map = fleet::app::build_demo_map();
        Scenario scenario = fleet::scenario::ScenarioLoader::load(map.base, *scenario_file);
        // Read the file seed before moving the scenario into the console
        // (argument evaluation order is unspecified).
        const std::uint64_t resolved_seed =
            fleet::scenario::resolve_seed(seed, scenario.seed);
        Console console{map, std::move(scenario), resolved_seed};
        console.run();
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << std::format("fleet_console: {}\n", error.what());
        return EXIT_FAILURE;
    }
}

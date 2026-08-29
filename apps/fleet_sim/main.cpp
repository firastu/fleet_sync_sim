// fleet_sim — CLI over the scenario runner (ADR-009).
//
//   fleet_sim                                     built-in founding scenario
//   fleet_sim --scenario <file>                    declarative scenario file
//   fleet_sim --seed <uint64>                      CLI seed (highest precedence)
//   fleet_sim --trace <file>                       JSONL structured trace
//
// Same scenario + same resolved seed => byte-identical trace. The console
// output and the JSONL trace are both formatters over the same TraceEvents;
// the console sink is always attached, the JSONL sink only with --trace.

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "demo_map.hpp"

#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

namespace {

// The founding demo (commit #2..#8) as declarative data: a station
// partition, one observation, one reroute, a reconnect and a resync.
// scenarios/station_partition.json describes the same run for the file
// path; this built-in copy keeps `fleet_sim` runnable with no arguments
// and no file-path dependence.
[[nodiscard]] fleet::scenario::Scenario make_founding_scenario(
    const fleet::app::DemoMap& demo) {
    using fleet::common::RobotId;
    using fleet::common::Tick;
    using fleet::map::EdgeStatus;
    using fleet::network::EndpointId;
    using fleet::scenario::ObserveEdgeAction;
    using fleet::scenario::ResynchronizeAction;
    using fleet::scenario::Scenario;
    using fleet::scenario::ScenarioEvent;
    using fleet::scenario::SetLinkAction;

    Scenario scenario;
    scenario.name = "station_partition";
    scenario.seed = 1;
    scenario.has_station = true;
    scenario.station_endpoint = EndpointId{3};
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {demo.node("A"), demo.node("D")}});
    scenario.robots.push_back(
        {"robot_b", RobotId{2}, EndpointId{2}, {demo.node("I"), demo.node("H")}});

    const auto link = [&](Tick at, EndpointId from, EndpointId to, bool up) {
        scenario.events.push_back(ScenarioEvent{at, SetLinkAction{from, to, up}});
    };
    // t = 2000: station partitioned (all four directed paths down).
    link(Tick{2000}, EndpointId{1}, EndpointId{3}, false);
    link(Tick{2000}, EndpointId{2}, EndpointId{3}, false);
    link(Tick{2000}, EndpointId{3}, EndpointId{1}, false);
    link(Tick{2000}, EndpointId{3}, EndpointId{2}, false);
    // t = 5000: robot_a observes F-G blocked.
    scenario.events.push_back(ScenarioEvent{
        Tick{5000},
        ObserveEdgeAction{RobotId{1}, *demo.base.graph().edge_between(demo.node("F"),
                                                                      demo.node("G")),
                          EdgeStatus::Blocked, 0.9}});
    // t = 8000: reconnect, then both robots resynchronize (file order).
    link(Tick{8000}, EndpointId{1}, EndpointId{3}, true);
    link(Tick{8000}, EndpointId{2}, EndpointId{3}, true);
    link(Tick{8000}, EndpointId{3}, EndpointId{1}, true);
    link(Tick{8000}, EndpointId{3}, EndpointId{2}, true);
    scenario.events.push_back(ScenarioEvent{Tick{8000}, ResynchronizeAction{RobotId{1}}});
    scenario.events.push_back(ScenarioEvent{Tick{8000}, ResynchronizeAction{RobotId{2}}});
    return scenario;
}

struct Options {
    std::optional<std::filesystem::path> scenario_file;
    std::optional<std::uint64_t> cli_seed;
    std::optional<std::filesystem::path> trace_file;
};

[[nodiscard]] std::runtime_error usage_error(const std::string& message) {
    return std::runtime_error(std::format(
        "{}\nusage: fleet_sim [--scenario <file>] [--seed <uint64>] [--trace <file>]",
        message));
}

// Strict uint64 parsing: digits only, no overflow, full-token consumption.
[[nodiscard]] std::uint64_t parse_seed(const std::string& text) {
    std::uint64_t value = 0;
    if (text.empty()) {
        throw usage_error("--seed expects an unsigned 64-bit integer");
    }
    for (const char digit : text) {
        if (digit < '0' || digit > '9') {
            throw usage_error(std::format(
                "--seed expects an unsigned 64-bit integer, got '{}'", text));
        }
        if (value > (std::numeric_limits<std::uint64_t>::max() - (digit - '0')) / 10) {
            throw usage_error(std::format("--seed value '{}' out of range", text));
        }
        value = value * 10 + static_cast<std::uint64_t>(digit - '0');
    }
    return value;
}

[[nodiscard]] Options parse_options(std::span<const char* const> args) {
    Options options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string argument = args[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= args.size()) {
                throw usage_error(std::format("{} expects a value", argument));
            }
            return args[++i];
        };
        if (argument == "--scenario") {
            options.scenario_file = value();
        } else if (argument == "--seed") {
            options.cli_seed = parse_seed(value());
        } else if (argument == "--trace") {
            options.trace_file = value();
        } else {
            throw usage_error(std::format("unknown option '{}'", argument));
        }
    }
    return options;
}

int run(const Options& options) {
    const fleet::app::DemoMap demo = fleet::app::build_demo_map();

    fleet::scenario::Scenario scenario =
        options.scenario_file.has_value()
            ? fleet::scenario::ScenarioLoader::load(demo.base, *options.scenario_file)
            : make_founding_scenario(demo);
    const std::string scenario_name = scenario.name;
    const std::uint64_t resolved_seed =
        fleet::scenario::resolve_seed(options.cli_seed, scenario.seed);

    std::optional<std::ofstream> trace_output;
    std::optional<fleet::scenario::JsonlTraceSink> jsonl_sink;
    if (options.trace_file.has_value()) {
        trace_output.emplace(*options.trace_file, std::ios::out | std::ios::trunc);
        if (!trace_output->is_open()) {
            throw std::runtime_error(
                std::format("cannot open trace file '{}'", options.trace_file->string()));
        }
        jsonl_sink.emplace(*trace_output);
    }

    fleet::scenario::ConsoleTraceSink console_sink{std::cout};
    fleet::scenario::ScenarioRunner runner{demo.base, std::move(scenario), resolved_seed};
    runner.add_sink(console_sink);
    if (jsonl_sink.has_value()) {
        runner.add_sink(*jsonl_sink);
    }
    const fleet::scenario::ScenarioRunner::Result result = runner.run_to_completion();

    std::cout << std::format("\nscenario '{}' finished at tick {} (resolved seed {})\n",
                             scenario_name, result.finished_at.value, result.resolved_seed);
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        return run(parse_options({argv + 1, argv + argc}));
    } catch (const std::exception& error) {
        std::cerr << std::format("fleet_sim: {}\n", error.what());
        return EXIT_FAILURE;
    }
}

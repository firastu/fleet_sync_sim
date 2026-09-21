// fleet_isaac_export — Isaac stage 1 read-only replay exporter (ADR-020).
//
//   fleet_isaac_export --scenario <file> --out <dir>
//                       [--seed <uint64>] [--period-ms <n>]
//                       [--footprint-m <m>] [--trace <file>]
//
// Loads the scenario against the built-in Isaac fixture map (nodes A, B, C
// around the pinned origin 52.370/9.730), runs it through the normal
// ScenarioRunner with stepped observation, and writes manifest.json +
// poses.jsonl into --out for tools/isaac/replay.py. CPU-only: this tool
// never contacts Isaac Sim, and the trace it emits is byte-identical to a
// plain fleet_sim run of the same scenario and resolved seed.

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>

#include "exporter.hpp"
#include "isaac_fixture_map.hpp"

#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/trace.hpp"

namespace {

struct Options {
    std::filesystem::path scenario_file;
    std::optional<std::uint64_t> cli_seed;
    std::filesystem::path out_dir;
    std::uint64_t period_ms = 100;
    double footprint_m = 50.0;
    std::optional<std::filesystem::path> trace_file;
};

class usage_error : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

[[nodiscard]] std::string usage() {
    return "usage: fleet_isaac_export --scenario <file> --out <dir> [--seed <uint64>] "
           "[--period-ms <n>] [--footprint-m <m>] [--trace <file>]";
}

// Strict uint64 parsing: digits only, no overflow, full-token consumption.
[[nodiscard]] std::uint64_t parse_uint64(const std::string& text, const std::string& option) {
    std::uint64_t value = 0;
    if (text.empty()) {
        throw usage_error(std::format("{} expects an unsigned 64-bit integer", option));
    }
    for (const char digit : text) {
        if (digit < '0' || digit > '9') {
            throw usage_error(
                std::format("{} expects an unsigned 64-bit integer, got '{}'", option, text));
        }
        if (value > (std::numeric_limits<std::uint64_t>::max() - (digit - '0')) / 10) {
            throw usage_error(std::format("{} value '{}' out of range", option, text));
        }
        value = value * 10 + static_cast<std::uint64_t>(digit - '0');
    }
    return value;
}

[[nodiscard]] double parse_double(const std::string& text, const std::string& option) {
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    } catch (const std::exception&) {
        throw usage_error(std::format("{} expects a finite number, got '{}'", option, text));
    }
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
        } else if (argument == "--out") {
            options.out_dir = value();
        } else if (argument == "--seed") {
            options.cli_seed = parse_uint64(value(), "--seed");
        } else if (argument == "--period-ms") {
            options.period_ms = parse_uint64(value(), "--period-ms");
        } else if (argument == "--footprint-m") {
            options.footprint_m = parse_double(value(), "--footprint-m");
        } else if (argument == "--trace") {
            options.trace_file = value();
        } else {
            throw usage_error(std::format("unknown option '{}'\n{}", argument, usage()));
        }
    }
    if (options.scenario_file.empty()) {
        throw usage_error(std::format("--scenario is required\n{}", usage()));
    }
    if (options.out_dir.empty()) {
        throw usage_error(std::format("--out is required\n{}", usage()));
    }
    return options;
}

[[nodiscard]] std::string read_file_bytes(const std::filesystem::path& path) {
    std::ifstream input{path, std::ios::in | std::ios::binary};
    if (!input.is_open()) {
        throw std::runtime_error(
            std::format("cannot open scenario file '{}'", path.string()));
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (input.bad()) {
        throw std::runtime_error(
            std::format("cannot read scenario file '{}'", path.string()));
    }
    return buffer.str();
}

int run(const Options& options) {
    const fleet::isaac::FixtureMap fixture = fleet::isaac::build_fixture_map();
    const std::string scenario_json = read_file_bytes(options.scenario_file);
    const fleet::scenario::Scenario scenario =
        fleet::scenario::ScenarioLoader::load_string(fixture.base, scenario_json);
    const std::string scenario_name = scenario.name;
    const std::uint64_t resolved_seed =
        fleet::scenario::resolve_seed(options.cli_seed, scenario.seed);

    std::optional<std::ofstream> trace_output;
    std::optional<fleet::scenario::JsonlTraceSink> jsonl_sink;
    if (options.trace_file.has_value()) {
        if (options.trace_file->has_parent_path()) {
            std::error_code error_code;
            std::filesystem::create_directories(options.trace_file->parent_path(), error_code);
        }
        trace_output.emplace(*options.trace_file, std::ios::out | std::ios::trunc);
        if (!trace_output->is_open()) {
            throw std::runtime_error(std::format("cannot open trace file '{}'",
                                                 options.trace_file->string()));
        }
        jsonl_sink.emplace(*trace_output);
    }

    fleet::scenario::ConsoleTraceSink console_sink{std::cout};

    fleet::isaac::ExportOptions export_options;
    export_options.resolved_seed = resolved_seed;
    export_options.period_ms = options.period_ms;
    export_options.footprint_radius_m = options.footprint_m;
    export_options.output_dir = options.out_dir;
    export_options.scenario_json = scenario_json;
    export_options.sinks.push_back(&console_sink);
    if (jsonl_sink.has_value()) {
        export_options.sinks.push_back(&*jsonl_sink);
    }

    const fleet::isaac::ExportSummary summary =
        fleet::isaac::export_replay_run(fixture.base, scenario, export_options);

    std::cout << std::format(
        "\nexport '{}' finished at tick {} (resolved seed {}): {} snapshots, {} robots, "
        "observed extent {:.3f} m\n",
        scenario_name, summary.last_tick_ms, resolved_seed, summary.snapshot_count,
        summary.robot_count, summary.observed_extent_m);
    std::cout << std::format("manifest: {}\nposes:    {}\n",
                             summary.manifest_path.string(), summary.poses_path.string());
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        return run(parse_options({argv + 1, argv + argc}));
    } catch (const std::exception& error) {
        std::cerr << std::format("fleet_isaac_export: {}\n", error.what());
        return EXIT_FAILURE;
    }
}

#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/network/probability.hpp"
#include "fleet/robot/robot.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::EdgeStatus;
using fleet::network::EndpointId;
using fleet::network::Probability;
using fleet::scenario::JsonlTraceSink;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::ScenarioRunner;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceSink;
using fleet::testsupport::make_grid_map;

// Captures every TraceEvent verbatim for ordering/content assertions.
class VectorTraceSink final : public TraceSink {
public:
    void record(const TraceEvent& event) override { events.push_back(event); }

    std::vector<TraceEvent> events;

    [[nodiscard]] std::size_t count(const std::string& source, const std::string& type) const {
        std::size_t total = 0;
        for (const TraceEvent& event : events) {
            if (event.source == source && event.type == type) {
                ++total;
            }
        }
        return total;
    }

    // First value of `key` on events matching (source, type), if present.
    [[nodiscard]] std::optional<std::string> first_value(const std::string& source,
                                                         const std::string& type,
                                                         const std::string& key) const {
        for (const TraceEvent& event : events) {
            if (event.source != source || event.type != type) {
                continue;
            }
            for (const auto& [field_key, field_value] : event.fields) {
                if (field_key == key) {
                    return fleet::scenario::format_trace_value(field_value);
                }
            }
        }
        return std::nullopt;
    }
};

// The JSONL bytes one (scenario, seed) pair produces.
[[nodiscard]] std::string run_to_jsonl(const Scenario& scenario, std::uint64_t seed) {
    const fleet::testsupport::GridMap grid = make_grid_map();
    std::ostringstream out;
    JsonlTraceSink sink{out};
    ScenarioRunner runner{grid.base, scenario, seed};
    runner.add_sink(sink);
    runner.run_to_completion();
    return out.str();
}

class ScenarioRunnerTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

// --- the founding scenario, declaratively, through the runner ----------------

TEST_F(ScenarioRunnerTest, FoundingScenarioReproducesPartitionAndConvergence) {
    // The real on-disk artifact (copied next to the test binary by CMake):
    // exactly what `fleet_sim --scenario scenarios/station_partition.json`
    // consumes.
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/station_partition.json");

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, scenario.seed.value_or(0)};
    runner.add_sink(trace);
    const ScenarioRunner::Result result = runner.run_to_completion();

    ASSERT_EQ(result.robot_count, 2U);
    ASSERT_TRUE(result.had_station);
    ASSERT_NE(runner.station(), nullptr);

    const EdgeId fg = edge("F", "G");

    // A observed F-G blocked and stayed on its own route (A-B-C-D).
    const fleet::robot::Robot& robot_a = runner.robot("robot_a");
    ASSERT_NE(robot_a.overlay().find(fg), nullptr);
    EXPECT_EQ(robot_a.overlay().find(fg)->status, EdgeStatus::Blocked);
    ASSERT_TRUE(robot_a.current_route().found);
    EXPECT_FALSE(robot_a.current_route().uses_edge(fg));
    EXPECT_DOUBLE_EQ(robot_a.current_route().cost, 3.0);

    // B received the delta over the fleet channel and rerouted around F-G.
    const fleet::robot::Robot& robot_b = runner.robot("robot_b");
    ASSERT_NE(robot_b.overlay().find(fg), nullptr);
    EXPECT_EQ(robot_b.overlay().find(fg)->status, EdgeStatus::Blocked);
    ASSERT_TRUE(robot_b.current_route().found);
    EXPECT_FALSE(robot_b.current_route().uses_edge(fg));

    // The station was partitioned at observation time, reconnected at 8000,
    // and converged with the fleet via resynchronization.
    ASSERT_NE(runner.station()->overlay().find(fg), nullptr);
    EXPECT_EQ(runner.station()->overlay().find(fg)->status, EdgeStatus::Blocked);
    EXPECT_EQ(runner.station()->overlay().tracked_count(), 1U);
    EXPECT_EQ(runner.station()->overlay().find(fg)->source.value(), 1);  // robot_a

    // Trace contract: resolved seed, initial routes, the observation, and
    // B's reroute.
    EXPECT_EQ(trace.count("world", "seed"), 1U);
    EXPECT_EQ(trace.count("robot_a", "route"), 1U);  // initial only: never replanned
    EXPECT_EQ(trace.count("robot_b", "route"), 2U);  // initial + reroute around F-G
    EXPECT_EQ(trace.count("robot_a", "observation"), 1U);
    EXPECT_EQ(trace.count("robot_a", "resynchronize"), 1U);
    EXPECT_EQ(trace.count("robot_b", "resynchronize"), 1U);
    EXPECT_EQ(trace.first_value("robot_a", "route", "nodes").value(), "A->B->C->D");
}

TEST_F(ScenarioRunnerTest, SameScenarioAndSeedProduceByteIdenticalTraces) {
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/station_partition.json");
    const std::string first = run_to_jsonl(scenario, 7);
    const std::string second = run_to_jsonl(scenario, 7);
    EXPECT_EQ(first, second);
    EXPECT_FALSE(first.empty());
}

TEST_F(ScenarioRunnerTest, DifferentSeedChangesTheDeterministicFaultSequence) {
    // A probabilistic scenario: ten observations, each sent to the station
    // over a 50% loss link. The resolved seed drives the fault sequence.
    Scenario scenario;
    scenario.name = "lossy";
    scenario.network.packet_loss = Probability::from_parts_per_million(500000);
    scenario.network.min_latency = Tick{10};
    scenario.network.max_latency = Tick{10};
    scenario.has_station = true;
    scenario.station_endpoint = EndpointId{2};
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    for (std::uint64_t observation = 0; observation < 10; ++observation) {
        scenario.events.push_back(fleet::scenario::ScenarioEvent{
            Tick{1000 + observation * 100},
            fleet::scenario::ObserveEdgeAction{
                RobotId{1}, edge("F", "G"),
                observation % 2 == 0 ? EdgeStatus::Blocked : EdgeStatus::Open, 0.9}});
    }

    const std::string seed_one_a = run_to_jsonl(scenario, 1);
    const std::string seed_one_b = run_to_jsonl(scenario, 1);
    const std::string seed_two = run_to_jsonl(scenario, 2);

    EXPECT_EQ(seed_one_a, seed_one_b);  // same seed: identical fault sequence
    EXPECT_NE(seed_one_a, seed_two);    // different seed: different faults
}

TEST_F(ScenarioRunnerTest, RunsAScenarioWithoutAStation) {
    Scenario scenario;
    scenario.name = "loner";
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{100},
        fleet::scenario::ObserveEdgeAction{RobotId{1}, edge("F", "G"),
                                           EdgeStatus::Blocked, 1.0}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    const ScenarioRunner::Result result = runner.run_to_completion();

    EXPECT_FALSE(result.had_station);
    EXPECT_EQ(runner.station(), nullptr);
    // Sole participant: the observation is applied locally, nobody to send to.
    EXPECT_EQ(trace.count("robot_a", "observation"), 1U);
    EXPECT_EQ(trace.count("robot_a", "send"), 0U);
    EXPECT_NE(runner.robot("robot_a").overlay().find(edge("F", "G")), nullptr);
}

TEST_F(ScenarioRunnerTest, RobotLookupThrowsForUnknownName) {
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/station_partition.json");
    ScenarioRunner runner{grid_.base, scenario, 1};
    EXPECT_THROW(static_cast<void>(runner.robot("ghost")), std::invalid_argument);
}

}  // namespace

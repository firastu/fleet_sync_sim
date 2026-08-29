#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/robot/robot.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::EdgeStatus;
using fleet::network::EndpointId;
using fleet::scenario::JsonlTraceSink;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::ScenarioRunner;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceSink;
using fleet::testsupport::make_grid_map;

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

    [[nodiscard]] std::optional<std::string> value_at(std::uint64_t at,
                                                      const std::string& source,
                                                      const std::string& type,
                                                      const std::string& key) const {
        for (const TraceEvent& event : events) {
            if (event.at.value == at && event.source == source && event.type == type) {
                for (const auto& [field_key, field_value] : event.fields) {
                    if (field_key == key) {
                        return fleet::scenario::format_trace_value(field_value);
                    }
                }
            }
        }
        return std::nullopt;
    }
};

class ScenarioSensingTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

TEST_F(ScenarioSensingTest, WorldSensingScenarioReroutesOnArrival) {
    // The physical path (ADR-011): the world blocks C-D at 1500, but the
    // robot (departing B, in transit on B-C) is NOT in range of C-D — it
    // learns the truth only when it ARRIVES at C at 2000 and reroutes
    // there. Contrast with scripted observe_edge (delivery_reroute),
    // which informs the robot instantly.
    const Scenario scenario = ScenarioLoader::load(grid_.base, "scenarios/world_sensing.json");

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, scenario.seed.value_or(0)};
    runner.add_sink(trace);
    const ScenarioRunner::Result result = runner.run_to_completion();

    EXPECT_EQ(result.finished_at, Tick{10000});
    const fleet::robot::Robot& robot = runner.robot("robot_a");
    EXPECT_EQ(robot.state().position, grid_.node("D"));
    EXPECT_TRUE(robot.state().mission_complete);

    // World change at 1500 is visible, but no robot observation happens
    // then; the sensor fires on arrival at C.
    EXPECT_EQ(trace.count("world", "world_edge"), 1U);
    EXPECT_EQ(trace.value_at(1500, "world", "world_edge", "edge").value(), "C-D");
    EXPECT_EQ(trace.value_at(1500, "robot_a", "observation", "origin"),
              std::nullopt);  // nothing sensed at 1500
    EXPECT_EQ(trace.value_at(2000, "robot_a", "observation", "origin").value(), "sensor");
    EXPECT_EQ(trace.value_at(2000, "robot_a", "observation", "edge").value(), "C-D");
    EXPECT_EQ(trace.value_at(2000, "robot_a", "observation", "replanned").value(), "true");

    // Detour from C: C-G, G-H, H-D; mission complete at 5000.
    EXPECT_EQ(trace.count("robot_a", "departure"), 5U);
    EXPECT_EQ(trace.value_at(2000, "robot_a", "departure", "to").value(), "G");
    EXPECT_EQ(trace.value_at(5000, "robot_a", "mission_complete", "goal").value(), "D");

    // The station learned the closure through the fleet channel (2080).
    ASSERT_NE(runner.station(), nullptr);
    ASSERT_NE(runner.station()->overlay().find(edge("C", "D")), nullptr);
    EXPECT_EQ(runner.station()->overlay().find(edge("C", "D"))->status, EdgeStatus::Blocked);
}

TEST_F(ScenarioSensingTest, StationaryRobotSensesAdjacentChangeImmediately) {
    // Static fleet, sensing on: a parked robot sees a change incident to
    // its node at the change tick and replans.
    Scenario scenario;
    scenario.name = "watcher";
    scenario.sensing.enabled = true;
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{1000},
        fleet::scenario::SetWorldEdgeStateAction{edge("A", "B"), EdgeStatus::Blocked}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_to_completion();

    EXPECT_EQ(trace.value_at(1000, "world", "world_edge", "edge").value(), "A-B");
    EXPECT_EQ(trace.value_at(1000, "robot_a", "observation", "origin").value(), "sensor");
    EXPECT_EQ(trace.value_at(1000, "robot_a", "observation", "replanned").value(), "true");
    const fleet::robot::Robot& robot = runner.robot("robot_a");
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_FALSE(robot.current_route().uses_edge(edge("A", "B")));
    EXPECT_DOUBLE_EQ(robot.current_route().cost, 5.0);  // best detour around A-B
}

TEST_F(ScenarioSensingTest, WorldCanChangeWithoutRobotLearning) {
    // The most important boundary, locked by test (ADR-011): the world
    // evolves independently of sensing. With no sensor configured, the
    // truth changes and the robot's belief simply diverges — truth !=
    // belief is a valid, useful state.
    Scenario scenario;
    scenario.name = "unaware";
    // No "sensing": world events are still valid; nobody observes them.
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("C"), grid_.node("D")}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{1000},
        fleet::scenario::SetWorldEdgeStateAction{edge("C", "D"), EdgeStatus::Blocked}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_to_completion();

    // The world change happened and is traced...
    EXPECT_EQ(trace.count("world", "world_edge"), 1U);
    // ...but no robot ever observed it.
    EXPECT_EQ(trace.count("robot_a", "observation"), 0U);

    // Truth is blocked; the robot's belief is still the base-map default
    // (untracked = open): it keeps routing through the closed edge.
    const fleet::robot::Robot& robot = runner.robot("robot_a");
    EXPECT_EQ(robot.overlay().find(edge("C", "D")), nullptr);
    ASSERT_TRUE(robot.current_route().found);
    EXPECT_TRUE(robot.current_route().uses_edge(edge("C", "D")));
}

TEST_F(ScenarioSensingTest, SensorGatesOnBeliefAcrossRepeatedChanges) {
    // Blocking an already-believed-blocked edge produces no observation;
    // reopening produces an OPEN observation (improvement => replan).
    Scenario scenario;
    scenario.name = "gated";
    scenario.sensing.enabled = true;
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("B")}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{1000},
        fleet::scenario::SetWorldEdgeStateAction{edge("A", "B"), EdgeStatus::Blocked}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{2000},
        fleet::scenario::SetWorldEdgeStateAction{edge("A", "B"), EdgeStatus::Blocked}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{3000},
        fleet::scenario::SetWorldEdgeStateAction{edge("A", "B"), EdgeStatus::Open}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_to_completion();

    EXPECT_EQ(trace.count("world", "world_edge"), 3U);
    EXPECT_EQ(trace.count("robot_a", "observation"), 2U);  // blocked@1000, open@3000
    EXPECT_EQ(trace.value_at(3000, "robot_a", "observation", "status").value(), "OPEN");
}

TEST_F(ScenarioSensingTest, MultipleRobotsSenseInDeclarationOrder) {
    Scenario scenario;
    scenario.name = "two_watchers";
    scenario.sensing.enabled = true;
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("C"), grid_.node("D")}});
    scenario.robots.push_back(
        {"robot_b", RobotId{2}, EndpointId{2}, {grid_.node("D"), grid_.node("A")}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{1000},
        fleet::scenario::SetWorldEdgeStateAction{edge("C", "D"), EdgeStatus::Blocked}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_to_completion();

    // Both robots are in range (C-D incident to C and to D); robot_a's
    // observation appears before robot_b's (declaration order, same tick).
    ASSERT_EQ(trace.count("robot_a", "observation"), 1U);
    ASSERT_EQ(trace.count("robot_b", "observation"), 1U);
    std::size_t robot_a_index = trace.events.size();
    std::size_t robot_b_index = trace.events.size();
    for (std::size_t i = 0; i < trace.events.size(); ++i) {
        if (trace.events[i].source == "robot_a" && trace.events[i].type == "observation") {
            robot_a_index = i;
        }
        if (trace.events[i].source == "robot_b" && trace.events[i].type == "observation") {
            robot_b_index = i;
        }
    }
    EXPECT_LT(robot_a_index, robot_b_index);
}

TEST_F(ScenarioSensingTest, SameSensingScenarioAndSeedProduceByteIdenticalTraces) {
    const Scenario scenario = ScenarioLoader::load(grid_.base, "scenarios/world_sensing.json");
    const fleet::testsupport::GridMap grid = make_grid_map();
    std::ostringstream first;
    JsonlTraceSink first_sink{first};
    ScenarioRunner first_runner{grid.base, scenario, 5};
    first_runner.add_sink(first_sink);
    first_runner.run_to_completion();

    std::ostringstream second;
    JsonlTraceSink second_sink{second};
    ScenarioRunner second_runner{grid.base, scenario, 5};
    second_runner.add_sink(second_sink);
    second_runner.run_to_completion();

    EXPECT_EQ(first.str(), second.str());
    EXPECT_FALSE(first.str().empty());
}

}  // namespace

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
using fleet::scenario::MovementSettings;
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

[[nodiscard]] std::string run_to_jsonl(const Scenario& scenario, std::uint64_t seed) {
    const fleet::testsupport::GridMap grid = make_grid_map();
    std::ostringstream out;
    JsonlTraceSink sink{out};
    ScenarioRunner runner{grid.base, scenario, seed};
    runner.add_sink(sink);
    runner.run_to_completion();
    return out.str();
}

class ScenarioMovementTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }

    [[nodiscard]] Scenario moving_robot_scenario() const {
        Scenario scenario;
        scenario.name = "mover";
        scenario.movement = MovementSettings{true, 1000, 1000};
        scenario.duration_ms = 10000;
        scenario.robots.push_back(
            {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
        return scenario;
    }
};

TEST_F(ScenarioMovementTest, MovementWalksTheRouteToCompletion) {
    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, moving_robot_scenario(), 0};
    runner.add_sink(trace);
    const ScenarioRunner::Result result = runner.run_to_completion();

    // Horizon semantics: the clock lands exactly on the duration.
    EXPECT_EQ(result.finished_at, Tick{10000});

    const fleet::robot::Robot& robot = runner.robot("robot_a");
    EXPECT_EQ(robot.state().position, grid_.node("D"));
    EXPECT_TRUE(robot.state().mission_complete);

    // Departures A->B@0, B->C@1000, C->D@2000; arrivals B@1000, C@2000;
    // mission complete at 3000. Knowledge never changed, so the only
    // "route" event is the initial one (movement replans are untraced).
    EXPECT_EQ(trace.count("robot_a", "departure"), 3U);
    EXPECT_EQ(trace.count("robot_a", "arrival"), 2U);
    EXPECT_EQ(trace.count("robot_a", "mission_complete"), 1U);
    EXPECT_EQ(trace.count("robot_a", "route"), 1U);
    EXPECT_EQ(trace.value_at(0, "robot_a", "departure", "to").value(), "B");
    EXPECT_EQ(trace.value_at(0, "robot_a", "departure", "arrival").value(), "1000");
    EXPECT_EQ(trace.value_at(1000, "robot_a", "arrival", "node").value(), "B");
    EXPECT_EQ(trace.value_at(3000, "robot_a", "mission_complete", "goal").value(), "D");
}

TEST_F(ScenarioMovementTest, MovementScenarioFileReroutesFromPosition) {
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/delivery_reroute.json");

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, scenario.seed.value_or(0)};
    runner.add_sink(trace);
    const ScenarioRunner::Result result = runner.run_to_completion();

    EXPECT_EQ(result.finished_at, Tick{10000});
    const fleet::robot::Robot& robot = runner.robot("robot_a");
    EXPECT_EQ(robot.state().position, grid_.node("D"));
    EXPECT_TRUE(robot.state().mission_complete);

    // Mid-transit observation at 1500 (robot on B->C): replan from C
    // around the blocked C-D; the robot walks the detour C->G->H->D.
    EXPECT_EQ(trace.value_at(1500, "robot_a", "observation", "replanned").value(), "true");
    EXPECT_EQ(trace.count("robot_a", "departure"), 5U);  // A-B, B-C, C-G, G-H, H-D
    EXPECT_EQ(trace.count("robot_a", "arrival"), 4U);    // B, C, G, H
    EXPECT_EQ(trace.count("robot_a", "mission_complete"), 1U);
    EXPECT_EQ(trace.value_at(5000, "robot_a", "mission_complete", "goal").value(), "D");

    // The station received the world change.
    ASSERT_NE(runner.station(), nullptr);
    ASSERT_NE(runner.station()->overlay().find(edge("C", "D")), nullptr);
    EXPECT_EQ(runner.station()->overlay().find(edge("C", "D"))->status, EdgeStatus::Blocked);
}

TEST_F(ScenarioMovementTest, RobotParksAndRetriesUntilRouteReturns) {
    Scenario scenario = moving_robot_scenario();
    scenario.name = "park_and_retry";
    // Robot departs A->B at 0 (arrival 1000). At 500 it learns D is
    // unreachable (both D edges blocked): replan from B finds no route,
    // so after arriving at B at 1000 it parks. Retries at 2000 and 3000
    // fail; at 3500 D-H reopens (improvement => replan; C-D stays
    // blocked), so the 4000 retry departs on the cost-4 detour
    // B->C->G->H->D (ADR-003 tie-break): departures at 4000/5000/6000/
    // 7000, mission complete at 8000.
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{500},
        fleet::scenario::ObserveEdgeAction{RobotId{1}, edge("C", "D"), EdgeStatus::Blocked,
                                           0.9}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{500},
        fleet::scenario::ObserveEdgeAction{RobotId{1}, edge("D", "H"), EdgeStatus::Blocked,
                                           0.9}});
    scenario.events.push_back(fleet::scenario::ScenarioEvent{
        Tick{3500},
        fleet::scenario::ObserveEdgeAction{RobotId{1}, edge("D", "H"), EdgeStatus::Open,
                                           0.9}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_to_completion();

    const fleet::robot::Robot& robot = runner.robot("robot_a");
    EXPECT_EQ(robot.state().position, grid_.node("D"));
    EXPECT_TRUE(robot.state().mission_complete);

    // Polling semantics: movement advances only at chain events, so the
    // departure after the 3500 reopening happens at the 4000 retry — not
    // at 3500, and not at 1000 when the route first vanished.
    EXPECT_EQ(trace.count("robot_a", "departure"), 5U);  // A-B + the B-C-G-H-D detour
    EXPECT_EQ(trace.value_at(0, "robot_a", "departure", "to").value(), "B");
    EXPECT_EQ(trace.value_at(4000, "robot_a", "departure", "to").value(), "C");
    EXPECT_EQ(trace.value_at(5000, "robot_a", "departure", "to").value(), "G");
    EXPECT_EQ(trace.value_at(6000, "robot_a", "departure", "to").value(), "H");
    EXPECT_EQ(trace.value_at(7000, "robot_a", "departure", "to").value(), "D");
    EXPECT_EQ(trace.value_at(8000, "robot_a", "mission_complete", "goal").value(), "D");
}

TEST_F(ScenarioMovementTest, RunnerRejectsZeroTimingSettingsWithoutLoader) {
    // Programmatically constructed scenarios bypass the loader; the
    // runner constructor is the choke point that still guarantees no
    // zero-time self-scheduling loops (ADR-010).
    Scenario broken = moving_robot_scenario();
    broken.movement.retry_ms = 0;
    EXPECT_THROW((ScenarioRunner{grid_.base, broken, 0}), std::invalid_argument);

    Scenario zero_speed = moving_robot_scenario();
    zero_speed.movement.ms_per_cost_unit = 0;
    EXPECT_THROW((ScenarioRunner{grid_.base, zero_speed, 0}), std::invalid_argument);
}

TEST_F(ScenarioMovementTest, MinimalSpeedAdvancesAtLeastOneTickPerTraversal) {
    // Base costs are strictly positive (graph invariant) and the minimal
    // speed is 1 ms per cost unit: an arrival is always strictly later
    // than its departure, even on the cheapest edge.
    Scenario scenario = moving_robot_scenario();
    scenario.movement.ms_per_cost_unit = 1;
    scenario.duration_ms = 5;

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_to_completion();

    EXPECT_EQ(trace.value_at(0, "robot_a", "departure", "arrival").value(), "1");
    EXPECT_EQ(trace.value_at(1, "robot_a", "arrival", "node").value(), "B");
}

TEST_F(ScenarioMovementTest, SameMovementScenarioAndSeedProduceByteIdenticalTraces) {
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/delivery_reroute.json");
    const std::string first = run_to_jsonl(scenario, 42);
    const std::string second = run_to_jsonl(scenario, 42);
    EXPECT_EQ(first, second);
    EXPECT_FALSE(first.empty());
}

}  // namespace

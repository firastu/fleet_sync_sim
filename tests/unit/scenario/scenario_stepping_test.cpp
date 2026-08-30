#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <stdexcept>
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
using fleet::scenario::ObserveEdgeAction;
using fleet::scenario::ResynchronizeAction;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioEvent;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::ScenarioRunner;
using fleet::scenario::SetLinkAction;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceSink;
using fleet::testsupport::make_grid_map;

class VectorTraceSink final : public TraceSink {
public:
    void record(const TraceEvent& event) override { events.push_back(event); }
    std::vector<TraceEvent> events;
};

class ScenarioSteppingTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] EdgeId edge(const char* a, const char* b) const {
        return grid_.base.graph().edge_between(grid_.node(a), grid_.node(b)).value();
    }
};

TEST_F(ScenarioSteppingTest, SteppedRunMatchesOneShotByteForByte) {
    // The stepping contract (#13): arbitrary run_until chunks execute
    // exactly the same events in the same order as run_to_completion.
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/world_sensing.json");

    std::ostringstream one_shot;
    {
        JsonlTraceSink sink{one_shot};
        ScenarioRunner runner{grid_.base, scenario, 3};
        runner.add_sink(sink);
        runner.run_to_completion();
    }
    std::ostringstream stepped;
    {
        JsonlTraceSink sink{stepped};
        ScenarioRunner runner{grid_.base, scenario, 3};
        runner.add_sink(sink);
        for (const std::uint64_t chunk : {100ULL, 400ULL, 1ULL, 2500ULL, 5999ULL}) {
            runner.run_until(Tick{runner.now().value + chunk});
        }
        runner.run_to_completion();
    }
    EXPECT_EQ(one_shot.str(), stepped.str());
}

TEST_F(ScenarioSteppingTest, RunUntilLandsExactlyOnTheHorizon) {
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/delivery_reroute.json");
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.begin();
    EXPECT_EQ(runner.now(), Tick{0});
    const auto result = runner.run_until(Tick{2500});
    EXPECT_EQ(result.finished_at, Tick{2500});
    EXPECT_EQ(runner.now(), Tick{2500});
    // Mid-run inspection: the robot is mid-edge (B->C arrives at 2000).
    EXPECT_TRUE(runner.robot("robot_a").state().in_transit.has_value());
}

TEST_F(ScenarioSteppingTest, RunUntilRejectsBackwardHorizon) {
    Scenario scenario;
    scenario.name = "mover";
    scenario.duration_ms = 10000;
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.run_until(Tick{1000});
    EXPECT_THROW(static_cast<void>(runner.run_until(Tick{999})), std::invalid_argument);
    // Same-tick stepping stays legal and deterministic (no-op advance).
    EXPECT_NO_THROW(static_cast<void>(runner.run_until(Tick{1000})));
    EXPECT_EQ(runner.now(), Tick{1000});
}

TEST_F(ScenarioSteppingTest, BeginIsIdempotent) {
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/station_partition.json");
    ScenarioRunner runner{grid_.base, scenario, 1};
    runner.begin();
    runner.begin();
    runner.run_to_completion();
    // Double begin must not double-schedule: exactly one tracked edge.
    EXPECT_EQ(runner.robot("robot_a").overlay().tracked_count(), 1U);
}

TEST_F(ScenarioSteppingTest, InjectObservationTakesEffectAtItsTick) {
    Scenario scenario;
    scenario.name = "injectable";
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);

    runner.run_until(Tick{100});
    runner.inject(ScenarioEvent{
        Tick{300}, ObserveEdgeAction{RobotId{1}, edge("F", "G"), EdgeStatus::Blocked, 0.9}});
    const auto result = runner.run_until(Tick{1000});

    EXPECT_EQ(result.finished_at, Tick{1000});
    EXPECT_NE(runner.robot("robot_a").overlay().find(edge("F", "G")), nullptr);
    bool observed_at_300 = false;
    for (const TraceEvent& event : trace.events) {
        observed_at_300 =
            observed_at_300 || (event.at == Tick{300} && event.type == "observation");
    }
    EXPECT_TRUE(observed_at_300);
}

TEST_F(ScenarioSteppingTest, InjectLinkAndResyncComposeWithLoadedEvents) {
    // Interactive partition + reconnect through inject(): the loaded
    // observation reaches the station at 580; the operator then cuts
    // the link and resynchronizes after a (hypothetical) reconnect.
    Scenario scenario;
    scenario.name = "interactive";
    scenario.has_station = true;
    scenario.station_endpoint = EndpointId{2};
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    scenario.events.push_back(ScenarioEvent{
        Tick{500},
        ObserveEdgeAction{RobotId{1}, edge("B", "C"), EdgeStatus::Blocked, 0.9}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_until(Tick{1000});
    ASSERT_NE(runner.station(), nullptr);
    ASSERT_NE(runner.station()->overlay().find(edge("B", "C")), nullptr);

    runner.inject(
        ScenarioEvent{Tick{1200}, SetLinkAction{EndpointId{1}, EndpointId{2}, false}});
    runner.inject(ScenarioEvent{Tick{1500}, ResynchronizeAction{RobotId{1}}});
    runner.run_until(Tick{2000});

    // Observation + link_state + resynchronize all traced.
    EXPECT_GE(trace.events.size(), 3U);
    const fleet::robot::Robot& robot = runner.robot("robot_a");
    EXPECT_NE(robot.overlay().find(edge("B", "C")), nullptr);
}

TEST_F(ScenarioSteppingTest, InjectedEventMatchesLoadedEventExactly) {
    // The console-injection boundary: an event injected through the
    // public inject() API has EXACTLY the same effect (trace bytes and
    // state) as the identical event loaded from a scenario file. The
    // console has no second execution path.
    const ObserveEdgeAction action{RobotId{1}, edge("B", "C"), EdgeStatus::Blocked, 0.9};

    Scenario loaded;
    loaded.name = "same";
    loaded.has_station = true;
    loaded.station_endpoint = EndpointId{2};
    loaded.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    loaded.events.push_back(ScenarioEvent{Tick{500}, action});

    Scenario injected;
    injected.name = "same";
    injected.has_station = true;
    injected.station_endpoint = EndpointId{2};
    injected.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});

    std::ostringstream loaded_trace;
    {
        fleet::scenario::JsonlTraceSink sink{loaded_trace};
        ScenarioRunner runner{grid_.base, loaded, 0};
        runner.add_sink(sink);
        runner.run_to_completion();
    }
    std::ostringstream injected_trace;
    {
        fleet::scenario::JsonlTraceSink sink{injected_trace};
        ScenarioRunner runner{grid_.base, injected, 0};
        runner.add_sink(sink);
        runner.begin();
        runner.inject(ScenarioEvent{Tick{500}, action});
        runner.run_to_completion();
    }
    EXPECT_EQ(loaded_trace.str(), injected_trace.str());
}

TEST_F(ScenarioSteppingTest, InjectRejectsPastTicksAndMissingBegin) {
    Scenario scenario;
    scenario.name = "guard";
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});
    ScenarioRunner fresh{grid_.base, scenario, 0};
    EXPECT_THROW(
        static_cast<void>(
            fresh.inject(ScenarioEvent{Tick{0}, ResynchronizeAction{RobotId{1}}})),
        std::logic_error);

    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.run_until(Tick{1000});
    EXPECT_THROW(
        static_cast<void>(
            runner.inject(ScenarioEvent{Tick{999}, ResynchronizeAction{RobotId{1}}})),
        std::invalid_argument);
    // Same tick is legal (schedules after already-queued events, ADR-005).
    EXPECT_NO_THROW(static_cast<void>(runner.inject(
        ScenarioEvent{Tick{1000}, ResynchronizeAction{RobotId{1}}})));
}

TEST_F(ScenarioSteppingTest, FinishAfterPassingHorizonIsStable) {
    // duration=5000, operator steps to 6000, then finishes: no throw,
    // no retry drain, no additional observable trace after the snapshot.
    Scenario scenario;
    scenario.name = "extended";
    scenario.movement.enabled = true;
    scenario.duration_ms = 5000;
    scenario.robots.push_back(
        {"robot_a", RobotId{1}, EndpointId{1}, {grid_.node("A"), grid_.node("D")}});

    VectorTraceSink trace;
    ScenarioRunner runner{grid_.base, scenario, 0};
    runner.add_sink(trace);
    runner.run_until(Tick{6000});
    const std::size_t events_at_6000 = trace.events.size();

    const auto result = runner.run_to_completion();
    EXPECT_EQ(result.finished_at, Tick{6000});
    EXPECT_EQ(runner.now(), Tick{6000});
    EXPECT_EQ(trace.events.size(), events_at_6000);  // nothing drained
}

}  // namespace

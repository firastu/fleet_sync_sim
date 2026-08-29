#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/trace.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

#include "fleet/common/ids.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::Tick;
using fleet::network::EndpointId;
using fleet::scenario::JsonlTraceSink;
using fleet::scenario::ConsoleTraceSink;
using fleet::scenario::ObserveEdgeAction;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::SetLinkAction;
using fleet::scenario::TraceEvent;
using fleet::scenario::resolve_seed;
using fleet::testsupport::make_grid_map;

class ScenarioTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};

    [[nodiscard]] Scenario load(const std::string& json) const {
        return ScenarioLoader::load_string(grid_.base, json);
    }

    // Minimal valid scenario; each test mutates one aspect of it.
    [[nodiscard]] static std::string minimal_json() {
        return R"json({
          "name": "minimal",
          "robots": [
            {"name": "robot_a", "id": 1, "endpoint": 7,
             "mission": {"start": "A", "goal": "D"}}
          ],
          "events": [
            {"at_ms": 100, "action": "observe_edge", "robot": "robot_a",
             "edge": "F-G", "state": "blocked"}
          ]
        })json";
    }
};

TEST_F(ScenarioTest, ResolveSeedPrecedenceIsCliOverFileOverDefault) {
    EXPECT_EQ(resolve_seed(std::uint64_t{5}, std::uint64_t{9}), std::uint64_t{5});
    EXPECT_EQ(resolve_seed(std::uint64_t{5}, std::nullopt), std::uint64_t{5});
    EXPECT_EQ(resolve_seed(std::nullopt, std::uint64_t{9}), std::uint64_t{9});
    EXPECT_EQ(resolve_seed(std::nullopt, std::nullopt), std::uint64_t{0});
}

TEST_F(ScenarioTest, LoadsMinimalScenarioIntoTypedData) {
    const Scenario scenario = load(minimal_json());

    ASSERT_EQ(scenario.name, "minimal");
    EXPECT_FALSE(scenario.seed.has_value());
    EXPECT_FALSE(scenario.has_station);
    ASSERT_EQ(scenario.robots.size(), 1U);
    EXPECT_EQ(scenario.robots[0].name, "robot_a");
    EXPECT_EQ(scenario.robots[0].id.value(), 1);
    EXPECT_EQ(scenario.robots[0].endpoint.value(), 7);
    EXPECT_EQ(scenario.robots[0].mission.start, grid_.node("A"));
    EXPECT_EQ(scenario.robots[0].mission.goal, grid_.node("D"));
    // Network defaults: fixed 80 ms latency, ideal link.
    EXPECT_EQ(scenario.network.min_latency, Tick{80});
    EXPECT_EQ(scenario.network.max_latency, Tick{80});

    ASSERT_EQ(scenario.events.size(), 1U);
    const auto* observation =
        std::get_if<fleet::scenario::ObserveEdgeAction>(&scenario.events[0].action);
    ASSERT_NE(observation, nullptr);
    EXPECT_EQ(scenario.events[0].at, Tick{100});
    EXPECT_EQ(observation->edge,
              *grid_.base.graph().edge_between(grid_.node("F"), grid_.node("G")));
    EXPECT_EQ(observation->status, fleet::map::EdgeStatus::Blocked);
    EXPECT_DOUBLE_EQ(observation->confidence, 1.0);  // omitted -> default
}

TEST_F(ScenarioTest, LoadsFoundingScenarioFileFromDisk) {
    // The on-disk artifact is part of the contract (CMake copies scenarios/
    // next to the test binary); this is the file `fleet_sim --scenario`
    // consumes.
    const Scenario scenario =
        ScenarioLoader::load(grid_.base, "scenarios/station_partition.json");

    EXPECT_EQ(scenario.name, "station_partition");
    EXPECT_EQ(scenario.seed.value(), std::uint64_t{1});
    ASSERT_TRUE(scenario.has_station);
    EXPECT_EQ(scenario.station_endpoint.value(), 3);
    ASSERT_EQ(scenario.robots.size(), 2U);
    EXPECT_EQ(scenario.robots[0].name, "robot_a");
    EXPECT_EQ(scenario.robots[1].name, "robot_b");
    // 4 cuts + 1 observation + 4 restores + 2 resynchronizations.
    ASSERT_EQ(scenario.events.size(), 11U);
}

TEST_F(ScenarioTest, ResolvesParticipantNamesAndSortsEventsStably) {
    const Scenario scenario = load(R"json({
      "name": "order",
      "seed": 42,
      "station": {"endpoint": 3},
      "robots": [
        {"name": "robot_a", "id": 1, "endpoint": 1,
         "mission": {"start": "A", "goal": "D"}}
      ],
      "events": [
        {"at_ms": 500, "action": "observe_edge", "robot": "robot_a",
         "edge": "F-G", "state": "blocked", "confidence": 0.75},
        {"at_ms": 100, "action": "set_link_state", "from": "robot_a",
         "to": "station", "up": false},
        {"at_ms": 100, "action": "set_link_state", "from": "station",
         "to": "robot_a", "up": false}
      ]
    })json");

    EXPECT_EQ(scenario.seed.value(), std::uint64_t{42});
    ASSERT_EQ(scenario.events.size(), 3U);
    // Sorted by tick; the two tick-100 events keep file order (their `to`
    // endpoints distinguish them), the tick-500 event is last.
    const auto* first = std::get_if<SetLinkAction>(&scenario.events[0].action);
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first->from.value(), 1);
    EXPECT_EQ(first->to.value(), 3);
    EXPECT_FALSE(first->up);
    const auto* second = std::get_if<SetLinkAction>(&scenario.events[1].action);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(second->from.value(), 3);
    EXPECT_EQ(second->to.value(), 1);
    const auto* observation =
        std::get_if<ObserveEdgeAction>(&scenario.events[2].action);
    ASSERT_NE(observation, nullptr);
    EXPECT_DOUBLE_EQ(observation->confidence, 0.75);
}

}  // namespace

// --- invalid input fails clearly and deterministically ----------------------

TEST_F(ScenarioTest, RejectsMalformedJson) {
    EXPECT_THROW(load("{ not json"), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsUnknownAction) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": 1, "action": "teleport", "robot": "r"}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsMissingField) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsNegativeTick) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": -1, "action": "resynchronize", "robot": "r"}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsUnknownMissionNode) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "Q"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsNonAdjacentEdgeName) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": 1, "action": "observe_edge", "robot": "r",
                  "edge": "A-L", "state": "blocked"}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsMalformedEdgeName) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": 1, "action": "observe_edge", "robot": "r",
                  "edge": "FG", "state": "blocked"}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsInvalidEdgeState) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": 1, "action": "observe_edge", "robot": "r",
                  "edge": "F-G", "state": "flooded"}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsUnknownEventRobot) {
    const std::string json = R"json({
      "name": "x", "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": 1, "action": "resynchronize", "robot": "ghost"}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsUnknownLinkParticipant) {
    const std::string json = R"json({
      "name": "x",
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": [{"at_ms": 1, "action": "set_link_state", "from": "r",
                  "to": "station", "up": false}]
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsInvertedLatencyRange) {
    const std::string json = R"json({
      "name": "x",
      "network": {"min_latency_ms": 200, "max_latency_ms": 100},
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsOutOfRangePartsPerMillion) {
    const std::string json = R"json({
      "name": "x",
      "network": {"packet_loss_ppm": 1000001},
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsDuplicateRobotName) {
    const std::string json = R"json({
      "name": "x",
      "robots": [
        {"name": "r", "id": 1, "endpoint": 1, "mission": {"start": "A", "goal": "D"}},
        {"name": "r", "id": 2, "endpoint": 2, "mission": {"start": "A", "goal": "D"}}
      ],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsSharedEndpointBetweenRobots) {
    const std::string json = R"json({
      "name": "x",
      "robots": [
        {"name": "r1", "id": 1, "endpoint": 4, "mission": {"start": "A", "goal": "D"}},
        {"name": "r2", "id": 2, "endpoint": 4, "mission": {"start": "A", "goal": "D"}}
      ],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsRobotReusingStationEndpoint) {
    const std::string json = R"json({
      "name": "x", "station": {"endpoint": 4},
      "robots": [{"name": "r", "id": 1, "endpoint": 4,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsIdBeyondByteRange) {
    const std::string json = R"json({
      "name": "x",
      "robots": [{"name": "r", "id": 300, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsEmptyRobotList) {
    const std::string json = R"json({"name": "x", "robots": [], "events": []})json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsUnreadableFile) {
    EXPECT_THROW(ScenarioLoader::load(grid_.base, "scenarios/does_not_exist.json"),
                 std::invalid_argument);
}

TEST_F(ScenarioTest, ParsesMovementAndDuration) {
    const Scenario scenario = load(R"json({
      "name": "moving",
      "movement": {"ms_per_cost_unit": 250, "retry_ms": 500},
      "duration_ms": 60000,
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json");

    ASSERT_TRUE(scenario.movement.enabled);
    EXPECT_EQ(scenario.movement.ms_per_cost_unit, std::uint64_t{250});
    EXPECT_EQ(scenario.movement.retry_ms, std::uint64_t{500});
    ASSERT_TRUE(scenario.duration_ms.has_value());
    EXPECT_EQ(*scenario.duration_ms, std::uint64_t{60000});
}

TEST_F(ScenarioTest, AbsentMovementMeansStaticFleet) {
    const Scenario scenario = load(minimal_json());
    EXPECT_FALSE(scenario.movement.enabled);
    EXPECT_FALSE(scenario.duration_ms.has_value());
    // Defaults are documented, not guessed.
    EXPECT_EQ(scenario.movement.ms_per_cost_unit, std::uint64_t{1000});
    EXPECT_EQ(scenario.movement.retry_ms, std::uint64_t{1000});
}

TEST_F(ScenarioTest, RejectsMovementWithoutDuration) {
    const std::string json = R"json({
      "name": "x",
      "movement": {"ms_per_cost_unit": 1000},
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsZeroMovementSettings) {
    const std::string json = R"json({
      "name": "x",
      "movement": {"ms_per_cost_unit": 0},
      "duration_ms": 1000,
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsZeroRetryCadence) {
    // Zero retry would self-schedule at the same tick forever (ADR-010).
    const std::string json = R"json({
      "name": "x",
      "movement": {"retry_ms": 0},
      "duration_ms": 1000,
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

TEST_F(ScenarioTest, RejectsNegativeDuration) {
    const std::string json = R"json({
      "name": "x",
      "duration_ms": -5,
      "robots": [{"name": "r", "id": 1, "endpoint": 1,
        "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(json), std::invalid_argument);
}

// --- trace sinks: deterministic formatting over the same events -------------

TEST(TraceSinkTest, ConsoleSinkFormatsOneLinePerEvent) {
    std::ostringstream out;
    ConsoleTraceSink sink{out};
    sink.record(TraceEvent{
        Tick{5000}, "robot_a", "observation",
        {{"edge", std::string{"F-G"}}, {"count", std::int64_t{2}},
         {"cost", 4.0}, {"replanned", false}}});
    EXPECT_EQ(out.str(),
              "[5000][robot_a] observation edge=F-G count=2 cost=4.00 replanned=false\n");
}

TEST(TraceSinkTest, JsonlSinkEmitsStableJsonObjectPerLine) {
    std::ostringstream out;
    JsonlTraceSink sink{out};
    sink.record(TraceEvent{
        Tick{5000}, "robot_a", "observation",
        {{"edge", std::string{"F-G"}}, {"count", std::int64_t{2}},
         {"cost", 4.0}, {"replanned", false}}});
    EXPECT_EQ(out.str(),
              "{\"t\":5000,\"source\":\"robot_a\",\"type\":\"observation\","
              "\"edge\":\"F-G\",\"count\":2,\"cost\":4.00,\"replanned\":false}\n");
}

TEST(TraceSinkTest, JsonlSinkEscapesStrings) {
    std::ostringstream out;
    JsonlTraceSink sink{out};
    sink.record(TraceEvent{
        Tick{0}, "world", "scenario", {{"name", std::string{"quote\" back\\slash"}}}});
    EXPECT_EQ(out.str(),
              "{\"t\":0,\"source\":\"world\",\"type\":\"scenario\","
              "\"name\":\"quote\\\" back\\\\slash\"}\n");
}

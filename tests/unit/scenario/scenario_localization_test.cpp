#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/localization/gnss_model.hpp"
#include "fleet/map/geometry.hpp"
#include "fleet/robot/robot.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::localization::GnssNoiseConfig;
using fleet::scenario::GnssModelSpec;
using fleet::scenario::JsonlTraceSink;
using fleet::scenario::LocalizationSettings;
using fleet::scenario::MovementSettings;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioEvent;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::ScenarioRunner;
using fleet::scenario::SetGnssModelAction;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceSink;
using fleet::testsupport::GridMap;
using fleet::testsupport::make_grid_map;
using fleet::testsupport::make_grid_map_with_geometry;

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

    [[nodiscard]] std::vector<const TraceEvent*> where(const std::string& source,
                                                       const std::string& type) const {
        std::vector<const TraceEvent*> found;
        for (const TraceEvent& event : events) {
            if (event.source == source && event.type == type) {
                found.push_back(&event);
            }
        }
        return found;
    }

    [[nodiscard]] std::vector<const TraceEvent*> all_of_type(const std::string& type) const {
        std::vector<const TraceEvent*> found;
        for (const TraceEvent& event : events) {
            if (event.type == type) {
                found.push_back(&event);
            }
        }
        return found;
    }

    [[nodiscard]] std::size_t count_type(const std::string& type) const {
        std::size_t total = 0;
        for (const TraceEvent& event : events) {
            if (event.type == type) {
                ++total;
            }
        }
        return total;
    }

};

[[nodiscard]] std::optional<std::string> field(const TraceEvent& event,
                                               const std::string& key) {
    for (const auto& [field_key, value] : event.fields) {
        if (field_key == key) {
            return fleet::scenario::format_trace_value(value);
        }
    }
    return std::nullopt;
}

[[nodiscard]] bool has_field(const TraceEvent& event, const std::string& key) {
    for (const auto& [field_key, value] : event.fields) {
        if (field_key == key) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] GnssModelSpec perfect() {
    return GnssModelSpec{GnssModelSpec::Kind::Perfect, GnssNoiseConfig{}};
}

[[nodiscard]] GnssModelSpec unavailable() {
    return GnssModelSpec{GnssModelSpec::Kind::Unavailable, GnssNoiseConfig{}};
}

[[nodiscard]] GnssModelSpec noisy(double position_sigma, double heading_sigma) {
    return GnssModelSpec{GnssModelSpec::Kind::Noisy,
                         GnssNoiseConfig{position_sigma, heading_sigma}};
}

class ScenarioLocalizationTest : public ::testing::Test {
protected:
    const GridMap grid_{make_grid_map_with_geometry()};
    const GridMap bare_{make_grid_map()};

    // Base scenario: one robot A->D on the geometry-backed grid, with a
    // duration horizon. Movement and localization are opt-in per test.
    [[nodiscard]] Scenario make_scenario(
        bool movement, std::optional<LocalizationSettings> localization,
        std::vector<ScenarioEvent> events = {}, std::uint64_t duration_ms = 6000) const {
        Scenario scenario;
        scenario.name = "localization_test";
        if (movement) {
            scenario.movement = MovementSettings{true, 1000, 1000};
        }
        if (localization.has_value()) {
            scenario.localization = *localization;
        }
        scenario.duration_ms = duration_ms;
        scenario.robots.push_back(fleet::scenario::ScenarioRobot{
            "robot_a", RobotId{1}, fleet::network::EndpointId{1},
            fleet::robot::Mission{grid_.node("A"), grid_.node("D")}});
        scenario.events = std::move(events);
        return scenario;
    }

    [[nodiscard]] LocalizationSettings localization_settings(
        std::uint64_t period_ms, GnssModelSpec initial) const {
        LocalizationSettings settings;
        settings.enabled = true;
        settings.gnss_period_ms = period_ms;
        settings.initial_model = initial;
        return settings;
    }

    [[nodiscard]] ScenarioEvent switch_event(std::uint64_t at, GnssModelSpec model) const {
        return ScenarioEvent{Tick{at}, SetGnssModelAction{RobotId{1}, model}};
    }

    // Runs one scenario twice with identical inputs and returns both
    // JSONL traces for byte-identity comparison.
    [[nodiscard]] static std::string jsonl_trace(const fleet::map::BaseMap& base,
                                                 const Scenario& scenario,
                                                 std::uint64_t seed) {
        std::ostringstream out;
        JsonlTraceSink sink{out};
        ScenarioRunner runner{base, scenario, seed};
        runner.add_sink(sink);
        runner.run_to_completion();
        return out.str();
    }
};

TEST_F(ScenarioLocalizationTest, LocalizationIsOptInAndAddsNoEvents) {
    // Movement on, localization absent: the trace must not grow a single
    // localization event, and the robot holds no estimate.
    VectorTraceSink sink;
    ScenarioRunner runner(grid_.base, make_scenario(true, std::nullopt), 42);
    runner.add_sink(sink);
    runner.run_to_completion();
    EXPECT_EQ(sink.count_type("gnss_sample"), 0U);
    EXPECT_EQ(sink.count_type("gnss_model"), 0U);
    EXPECT_FALSE(runner.robot("robot_a").localization().estimate().has_value());
}

TEST_F(ScenarioLocalizationTest, PerfectGnssSamplesPeriodicallyFromTickZero) {
    // Static robot (no movement): truth is node A with the at-rest
    // heading contract (north). First sample at tick 0, then exactly
    // every period_ms — strictly later, never zero-time.
    VectorTraceSink sink;
    ScenarioRunner runner(
        grid_.base, make_scenario(false, localization_settings(1000, perfect())), 42);
    runner.add_sink(sink);
    runner.run_to_completion();  // duration 6000 -> ticks 0..6000 inclusive

    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 7U);  // 0,1000,2000,3000,4000,5000,6000
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_EQ(samples[i]->at, Tick{i * 1000}) << "sample " << i;
        EXPECT_EQ(field(*samples[i], "outcome"), "fix");
    }
    const auto& localization = runner.robot("robot_a").localization();
    ASSERT_TRUE(localization.estimate().has_value());
    EXPECT_EQ(localization.estimate()->estimated_at, Tick{6000});
    EXPECT_EQ(localization.estimate()->position, (fleet::map::Wgs84Coordinate{52.370, 9.730}));
    EXPECT_EQ(localization.estimate()->heading_rad, 0.0);
    EXPECT_EQ(localization.age_at(Tick{6000}), std::uint64_t{0});
}

TEST_F(ScenarioLocalizationTest, SameTickScriptedSwitchAppliesBeforeThatSample) {
    // Contract (ADR-018): a scripted switch enqueued at load precedes the
    // self-rescheduled sample of the same tick — the sample at 2000
    // measures with the NEW model. Locked by event order in the sink.
    VectorTraceSink sink;
    ScenarioRunner runner(grid_.base,
                          make_scenario(false, localization_settings(1000, perfect()),
                                        {switch_event(2000, unavailable())}),
                          42);
    runner.add_sink(sink);
    runner.run_to_completion();

    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 7U);
    EXPECT_EQ(field(*samples[0], "outcome"), "fix");
    EXPECT_EQ(field(*samples[1], "outcome"), "fix");
    EXPECT_EQ(field(*samples[2], "outcome"), "no_fix");  // switched BEFORE sampling
    EXPECT_EQ(field(*samples[3], "outcome"), "no_fix");

    // Relative order at tick 2000: the model switch is recorded before
    // the sample of the same tick (enqueue order — see ADR-018).
    std::size_t switch_index = sink.events.size();
    std::size_t sample_index = sink.events.size();
    for (std::size_t i = 0; i < sink.events.size(); ++i) {
        const TraceEvent& event = sink.events[i];
        if (event.at != Tick{2000}) {
            continue;
        }
        if (event.type == "gnss_model" && event.source == "robot_a") {
            switch_index = i;
        } else if (event.type == "gnss_sample" && event.source == "robot_a") {
            sample_index = i;
        }
    }
    ASSERT_LT(switch_index, sink.events.size());
    ASSERT_LT(sample_index, sink.events.size());
    EXPECT_LT(switch_index, sample_index);
}

TEST_F(ScenarioLocalizationTest, TickZeroScriptedSwitchPrecedesTickZeroSample) {
    VectorTraceSink sink;
    ScenarioRunner runner(
        grid_.base,
        make_scenario(false,
                      localization_settings(1000, perfect()),
                      {switch_event(0, unavailable())}),
        42);

    runner.add_sink(sink);
    runner.run_to_completion();

    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 7U);

    EXPECT_EQ(samples[0]->at, Tick{0});
    EXPECT_EQ(field(*samples[0], "outcome"), "no_fix");

    const auto model_events = sink.all_of_type("gnss_model");
    ASSERT_EQ(model_events.size(), 1U);
    EXPECT_EQ(model_events[0]->at, Tick{0});
    EXPECT_EQ(field(*model_events[0], "model"), "unavailable");

    // The model switch must appear in the trace before the same-tick sample.
    const auto& events = sink.events;

    const auto model_it = std::find_if(
        events.begin(), events.end(),
        [](const TraceEvent& event) {
            return event.at == Tick{0} && event.type == "gnss_model";
        });

    const auto sample_it = std::find_if(
        events.begin(), events.end(),
        [](const TraceEvent& event) {
            return event.at == Tick{0} && event.type == "gnss_sample";
        });

    ASSERT_NE(model_it, events.end());
    ASSERT_NE(sample_it, events.end());

    EXPECT_LT(
        std::distance(events.begin(), model_it),
        std::distance(events.begin(), sample_it));
}


TEST_F(ScenarioLocalizationTest,
       UnrelatedRobotGnssSamplingDoesNotPerturbRobotNoiseSequence) {
    constexpr std::uint64_t kSeed = 42;

    Scenario single_robot =
        make_scenario(false,
                      localization_settings(1000, noisy(2.5, 0.1)),
                      {});

    Scenario with_unrelated_robot = single_robot;

    // Keep robot_a completely unchanged. Add an unrelated robot with a
    // distinct stable RobotId and transport endpoint.
    fleet::scenario::ScenarioRobot robot_b =
        with_unrelated_robot.robots.front();

    robot_b.id = RobotId{2};
    robot_b.endpoint = fleet::network::EndpointId{2};
    robot_b.name = "robot_b";

    with_unrelated_robot.robots.push_back(robot_b);

    VectorTraceSink single_sink;
    ScenarioRunner single_runner(grid_.base, single_robot, kSeed);
    single_runner.add_sink(single_sink);
    single_runner.run_to_completion();

    VectorTraceSink multi_sink;
    ScenarioRunner multi_runner(grid_.base, with_unrelated_robot, kSeed);
    multi_runner.add_sink(multi_sink);
    multi_runner.run_to_completion();

    const auto single_samples =
        single_sink.where("robot_a", "gnss_sample");

    const auto multi_samples =
        multi_sink.where("robot_a", "gnss_sample");

    ASSERT_FALSE(single_samples.empty());
    ASSERT_EQ(single_samples.size(), multi_samples.size());

    for (std::size_t i = 0; i < single_samples.size(); ++i) {
        const TraceEvent& single = *single_samples[i];
        const TraceEvent& multi = *multi_samples[i];

        EXPECT_EQ(single.at, multi.at)
            << "sample index " << i;

        EXPECT_EQ(field(single, "outcome"), field(multi, "outcome"))
            << "sample index " << i;

        EXPECT_EQ(field(single, "estimated_at"),
                  field(multi, "estimated_at"))
            << "sample index " << i;

        EXPECT_EQ(field(single, "age"), field(multi, "age"))
            << "sample index " << i;

        EXPECT_EQ(field(single, "position"), field(multi, "position"))
            << "sample index " << i;

        EXPECT_EQ(field(single, "heading"), field(multi, "heading"))
            << "sample index " << i;
    }
}
TEST_F(ScenarioLocalizationTest, ArrivalDoesNotResetOrientationToNorth) {
    // Orientation truth (ADR-018): a robot completing A->B keeps facing
    // its arrival direction (east ~pi/2) at rest — arrival is never a
    // north reset. Fixes during transit and after arrival share the
    // final segment's bearing; only the explicit INITIAL orientation
    // (before any movement) is north.
    VectorTraceSink sink;
    Scenario scenario = make_scenario(true, localization_settings(1000, perfect()), {}, 3000);
    scenario.robots.front().mission.goal = grid_.node("B");  // arrive and rest
    ScenarioRunner runner(grid_.base, scenario, 42);
    runner.add_sink(sink);
    runner.run_to_completion();
    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 4U);  // 0, 1000, 2000, 3000
    // t=0: departed A->B (movement runs before sampling), heading east.
    const std::string transit_heading = field(*samples[0], "heading").value();
    EXPECT_NE(transit_heading, "0.000000");
    // t>=1000: at rest at B after the arrival at 1000 — the heading
    // CARRIES OVER (final-segment bearing), never resets to north.
    for (std::size_t i = 1; i < samples.size(); ++i) {
        EXPECT_EQ(field(*samples[i], "heading"), transit_heading) << "sample " << i;
        EXPECT_NE(field(*samples[i], "heading"), "0.000000") << "sample " << i;
    }
}

TEST_F(ScenarioLocalizationTest, ReverseTraversalRestOrientationIsWest) {
    // The same contract on a reverse traversal (B->A): the robot faces
    // WEST (~3*pi/2) while traveling and keeps facing west at rest.
    VectorTraceSink sink;
    Scenario scenario = make_scenario(true, localization_settings(1000, perfect()), {}, 3000);
    scenario.robots.front().mission.start = grid_.node("B");
    scenario.robots.front().mission.goal = grid_.node("A");
    ScenarioRunner runner(grid_.base, scenario, 42);
    runner.add_sink(sink);
    runner.run_to_completion();
    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 4U);
    const std::string transit_heading = field(*samples[0], "heading").value();
    // West is ~3*pi/2 = 4.712... — formatted with 6 decimals.
    EXPECT_NE(transit_heading, "0.000000");
    EXPECT_NE(transit_heading, "1.570783");  // not east
    for (std::size_t i = 1; i < samples.size(); ++i) {
        EXPECT_EQ(field(*samples[i], "heading"), transit_heading) << "sample " << i;
        EXPECT_NE(field(*samples[i], "heading"), "0.000000") << "sample " << i;
    }
}

TEST_F(ScenarioLocalizationTest, NoFixBeforeFirstFixTracesNoEstimateFields) {
    VectorTraceSink sink;
    ScenarioRunner runner(
        grid_.base, make_scenario(false, localization_settings(500, unavailable())), 42);
    runner.add_sink(sink);
    runner.run_to_completion();

    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 13U);
    for (const TraceEvent* sample : samples) {
        EXPECT_EQ(field(*sample, "outcome"), "no_fix");
        // No estimate exists and none is manufactured: the event carries
        // the outcome and NOTHING else.
        EXPECT_FALSE(has_field(*sample, "estimated_at"));
        EXPECT_FALSE(has_field(*sample, "age"));
        EXPECT_FALSE(has_field(*sample, "position"));
        EXPECT_FALSE(has_field(*sample, "heading"));
    }
    EXPECT_FALSE(runner.robot("robot_a").localization().estimate().has_value());
}


TEST_F(ScenarioLocalizationTest, OutageRetainsEstimateWhileTruthMovesOn) {
    // The #16 story, exactly the fixture scenario's shape: fixes at
    // 0/1000/2000 (A, B, C), outage from 2500, no-fix samples at
    // 3000..6000 while the robot completes its mission (truth reaches D
    // at 3000) — retained position and estimated_at FROZEN at C@2000,
    // age growing tick by tick.
    VectorTraceSink sink;
    ScenarioRunner runner(
        grid_.base,
        make_scenario(true, localization_settings(1000, perfect()),
                      {switch_event(2500, unavailable())}),
        42);
    runner.add_sink(sink);
    runner.run_to_completion();

    // Truth moved on: the mission completed at D at 3000.
    EXPECT_EQ(sink.count("robot_a", "mission_complete"), 1U);
    EXPECT_EQ(sink.count("robot_a", "arrival"), 2U);  // B at 1000, C at 2000

    // Retained estimate: frozen at C, 2000 — the belief the robot holds.
    const auto& localization = runner.robot("robot_a").localization();
    ASSERT_TRUE(localization.estimate().has_value());
    EXPECT_EQ(localization.estimate()->estimated_at, Tick{2000});
    EXPECT_EQ(localization.estimate()->position, (fleet::map::Wgs84Coordinate{52.370, 9.734}));
    EXPECT_EQ(localization.age_at(Tick{6000}), std::uint64_t{4000});

    // And the trace shows the staleness growing: age 1000 at 3000, 3000
    // at 5000 — with position and estimated_at pinned.
    const auto samples = sink.all_of_type("gnss_sample");
    ASSERT_EQ(samples.size(), 7U);
    EXPECT_EQ(field(*samples[3], "outcome"), "no_fix");
    EXPECT_EQ(field(*samples[3], "estimated_at"), "2000");
    EXPECT_EQ(field(*samples[3], "age"), "1000");
    EXPECT_EQ(field(*samples[5], "age"), "3000");
    EXPECT_EQ(field(*samples[3], "position"), field(*samples[5], "position"));
    // The estimate is C, not D where truth actually is.
    EXPECT_EQ(field(*samples[3], "position"), std::string("52.370000,9.734000"));
}

TEST_F(ScenarioLocalizationTest, SameScenarioAndSeedIsByteIdentical) {
    const Scenario scenario = make_scenario(true, localization_settings(1000, perfect()),
                                            {switch_event(2500, unavailable())});
    EXPECT_EQ(jsonl_trace(grid_.base, scenario, 42), jsonl_trace(grid_.base, scenario, 42));
}

TEST_F(ScenarioLocalizationTest, NoisyGnssIsDeterministicPerSeed) {
    const Scenario scenario = make_scenario(false, localization_settings(1000, noisy(3.0, 0.05)));
    EXPECT_EQ(jsonl_trace(grid_.base, scenario, 7), jsonl_trace(grid_.base, scenario, 7));
    // A different resolved seed produces genuinely different estimates.
    EXPECT_NE(jsonl_trace(grid_.base, scenario, 7), jsonl_trace(grid_.base, scenario, 9));
}

TEST_F(ScenarioLocalizationTest, ZeroNoiseNoisyModelMatchesPerfectPositions) {
    // ADR-017's zero-noise contract, through the runner path: sigma 0
    // consumes no randomness and reproduces the perfect model exactly.
    VectorTraceSink perfect_sink;
    ScenarioRunner perfect_runner(
        grid_.base, make_scenario(false, localization_settings(1000, perfect())), 42);
    perfect_runner.add_sink(perfect_sink);
    perfect_runner.run_to_completion();

    VectorTraceSink zero_noise_sink;
    ScenarioRunner zero_noise_runner(
        grid_.base, make_scenario(false, localization_settings(1000, noisy(0.0, 0.0))), 42);
    zero_noise_runner.add_sink(zero_noise_sink);
    zero_noise_runner.run_to_completion();

    const auto perfect_samples = perfect_sink.all_of_type("gnss_sample");
    const auto zero_samples = zero_noise_sink.all_of_type("gnss_sample");
    ASSERT_EQ(perfect_samples.size(), zero_samples.size());
    for (std::size_t i = 0; i < perfect_samples.size(); ++i) {
        EXPECT_EQ(field(*perfect_samples[i], "position"), field(*zero_samples[i], "position"))
            << "sample " << i;
        EXPECT_EQ(field(*perfect_samples[i], "heading"), field(*zero_samples[i], "heading"))
            << "sample " << i;
    }
}

TEST_F(ScenarioLocalizationTest, ModelSwitchingAddsNoHiddenRngDraws) {
    // Two runs of the same noisy tail (sigma > 0 from tick 4000) with
    // DIFFERENT histories made only of zero-consumption models. If
    // switching or idle sampling consumed hidden draws, the noisy
    // estimates would diverge; the contract says they are identical.
    const Scenario alternating = make_scenario(
        false, localization_settings(1000, perfect()),
        {switch_event(1000, unavailable()), switch_event(2000, perfect()),
         switch_event(3000, unavailable()), switch_event(4000, noisy(2.5, 0.1))});
    const Scenario steady_outage =
        make_scenario(false, localization_settings(1000, unavailable()),
                      {switch_event(4000, noisy(2.5, 0.1))});

    VectorTraceSink alternating_sink;
    ScenarioRunner alternating_runner(grid_.base, alternating, 42);
    alternating_runner.add_sink(alternating_sink);
    alternating_runner.run_to_completion();

    VectorTraceSink steady_sink;
    ScenarioRunner steady_runner(grid_.base, steady_outage, 42);
    steady_runner.add_sink(steady_sink);
    steady_runner.run_to_completion();

    const auto alternating_samples = alternating_sink.all_of_type("gnss_sample");
    const auto steady_samples = steady_sink.all_of_type("gnss_sample");
    ASSERT_EQ(alternating_samples.size(), 7U);
    ASSERT_EQ(steady_samples.size(), 7U);
    for (std::size_t i = 4; i < 7; ++i) {  // ticks 4000, 5000, 6000: the noisy tail
        EXPECT_EQ(field(*alternating_samples[i], "position"),
                  field(*steady_samples[i], "position"))
            << "sample " << i;
        EXPECT_EQ(field(*alternating_samples[i], "heading"),
                  field(*steady_samples[i], "heading"))
            << "sample " << i;
    }
}

TEST_F(ScenarioLocalizationTest, RunnerRejectsLocalizationWithoutMapGeometry) {
    const Scenario scenario = make_scenario(false, localization_settings(1000, perfect()));
    EXPECT_THROW(ScenarioRunner(bare_.base, scenario, 42), std::invalid_argument);
}

TEST_F(ScenarioLocalizationTest, RunnerRejectsZeroSamplingPeriod) {
    const Scenario scenario = make_scenario(false, localization_settings(0, perfect()));
    EXPECT_THROW(ScenarioRunner(grid_.base, scenario, 42), std::invalid_argument);
}

TEST_F(ScenarioLocalizationTest, RunnerRejectsInvalidNoiseConfigurationEagerly) {
    const Scenario scenario =
        make_scenario(false, localization_settings(1000, noisy(-1.0, 0.0)));
    EXPECT_THROW(ScenarioRunner(grid_.base, scenario, 42), std::invalid_argument);
}

class LocalizationLoaderTest : public ::testing::Test {
protected:
    const GridMap grid_{make_grid_map_with_geometry()};
    const GridMap bare_{make_grid_map()};

    [[nodiscard]] Scenario load(const GridMap& map, const std::string& json) const {
        return ScenarioLoader::load_string(map.base, json);
    }

    [[nodiscard]] std::string localization_json(const std::string& body) const {
        return "{\n"
               "  \"name\": \"loc\",\n"
               "  \"duration_ms\": 5000,\n"
               "  \"localization\": {\n"
               "    \"gnss\": {\n"
               "      \"period_ms\": 750,\n" +
               body +
               "      \"initial_model\": \"perfect\"\n"
               "    }\n"
               "  },\n"
               "  \"robots\": [\n"
               "    {\"name\": \"robot_a\", \"id\": 1, \"endpoint\": 1,"
               " \"mission\": {\"start\": \"A\", \"goal\": \"D\"}}\n"
               "  ],\n"
               "  \"events\": []\n"
               "}";
    }
};

TEST_F(LocalizationLoaderTest, ParsesSettingsAndModelGrammar) {
    const Scenario scenario = load(
        grid_, localization_json("").substr(0, std::string::npos));
    EXPECT_TRUE(scenario.localization.enabled);
    EXPECT_EQ(scenario.localization.gnss_period_ms, 750U);
    EXPECT_EQ(scenario.localization.initial_model.kind, GnssModelSpec::Kind::Perfect);

    const Scenario noisy = load(grid_, R"json({
      "name": "loc_noisy",
      "duration_ms": 5000,
      "localization": {"gnss": {"period_ms": 500, "initial_model":
        {"noisy": {"position_axis_sigma_m": 2.5, "heading_sigma_rad": 0.1}}}},
      "robots": [{"name": "robot_a", "id": 1, "endpoint": 1,
                  "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json");
    EXPECT_EQ(noisy.localization.initial_model.kind, GnssModelSpec::Kind::Noisy);
    EXPECT_DOUBLE_EQ(noisy.localization.initial_model.noise.position_axis_sigma_m, 2.5);
    EXPECT_DOUBLE_EQ(noisy.localization.initial_model.noise.heading_sigma_rad, 0.1);
}

TEST_F(LocalizationLoaderTest, ParsesSetGnssModelEvent) {
    const Scenario scenario = load(grid_, R"json({
      "name": "loc_switch",
      "duration_ms": 5000,
      "localization": {"gnss": {"period_ms": 500, "initial_model": "unavailable"}},
      "robots": [{"name": "robot_a", "id": 1, "endpoint": 1,
                  "mission": {"start": "A", "goal": "D"}}],
      "events": [
        {"at_ms": 2000, "action": "set_gnss_model", "robot": "robot_a",
         "model": {"noisy": {"position_axis_sigma_m": 1.0}}}
      ]
    })json");
    ASSERT_EQ(scenario.events.size(), 1U);
    const SetGnssModelAction* action =
        std::get_if<SetGnssModelAction>(&scenario.events[0].action);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->robot, RobotId{1});
    EXPECT_EQ(action->model.kind, GnssModelSpec::Kind::Noisy);
    EXPECT_DOUBLE_EQ(action->model.noise.position_axis_sigma_m, 1.0);
    EXPECT_DOUBLE_EQ(action->model.noise.heading_sigma_rad, 0.0);
}

TEST_F(LocalizationLoaderTest, RejectsMissingDurationHorizon) {
    EXPECT_THROW(load(grid_, R"json({
      "name": "loc",
      "localization": {"gnss": {"period_ms": 500, "initial_model": "perfect"}},
      "robots": [{"name": "robot_a", "id": 1, "endpoint": 1,
                  "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json"), std::invalid_argument);
}

TEST_F(LocalizationLoaderTest, RejectsMapWithoutGeometry) {
    EXPECT_THROW(load(bare_, localization_json("")), std::invalid_argument);
}

TEST_F(LocalizationLoaderTest, RejectsZeroPeriod) {
    const std::string json = R"json({
      "name": "loc",
      "duration_ms": 5000,
      "localization": {"gnss": {"period_ms": 0, "initial_model": "perfect"}},
      "robots": [{"name": "robot_a", "id": 1, "endpoint": 1,
                  "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(grid_, json), std::invalid_argument);
}

TEST_F(LocalizationLoaderTest, RejectsUnknownModelName) {
    const std::string json = R"json({
      "name": "loc",
      "duration_ms": 5000,
      "localization": {"gnss": {"period_ms": 500, "initial_model": "rtk"}},
      "robots": [{"name": "robot_a", "id": 1, "endpoint": 1,
                  "mission": {"start": "A", "goal": "D"}}],
      "events": []
    })json";
    EXPECT_THROW(load(grid_, json), std::invalid_argument);
}

TEST_F(LocalizationLoaderTest, RejectsSetGnssModelWithoutLocalizationBlock) {
    const std::string json = R"json({
      "name": "loc",
      "duration_ms": 5000,
      "robots": [{"name": "robot_a", "id": 1, "endpoint": 1,
                  "mission": {"start": "A", "goal": "D"}}],
      "events": [
        {"at_ms": 100, "action": "set_gnss_model", "robot": "robot_a",
         "model": "unavailable"}
      ]
    })json";
    EXPECT_THROW(load(grid_, json), std::invalid_argument);
}

}  // namespace

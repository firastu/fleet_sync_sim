// Scenario-level reacquisition tests (#18, ADR-021): a returning fix is an
// observable TRANSITION — gnss_sample gains fields only on that one sample,
// and every existing localization fixture stays byte-unchanged.

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "test_maps.hpp"

#include "fleet/common/time.hpp"
#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/scenario_loader.hpp"
#include "fleet/scenario/scenario_runner.hpp"
#include "fleet/scenario/trace.hpp"

namespace {

using fleet::common::Tick;
using fleet::scenario::Scenario;
using fleet::scenario::ScenarioLoader;
using fleet::scenario::ScenarioRunner;
using fleet::scenario::TraceEvent;
using fleet::scenario::TraceSink;

class CapturingSink final : public TraceSink {
  public:
    void record(const TraceEvent& event) override { events_.push_back(event); }
    [[nodiscard]] const std::vector<TraceEvent>& events() const noexcept { return events_; }

  private:
    std::vector<TraceEvent> events_;
};

const fleet::testsupport::GridMap& grid() {
    static const fleet::testsupport::GridMap map =
        fleet::testsupport::make_grid_map_with_geometry();
    return map;
}

[[nodiscard]] CapturingSink run(const Scenario& scenario, std::uint64_t seed) {
    CapturingSink sink;
    ScenarioRunner runner{grid().base, scenario, seed};
    runner.add_sink(sink);
    runner.run_to_completion();
    return sink;
}

[[nodiscard]] std::string jsonl_trace(const Scenario& scenario, std::uint64_t seed) {
    std::ostringstream out;
    fleet::scenario::JsonlTraceSink sink{out};
    ScenarioRunner runner{grid().base, scenario, seed};
    runner.add_sink(sink);
    runner.run_to_completion();
    return out.str();
}

[[nodiscard]] bool has_field(const TraceEvent& event, const std::string& name) {
    for (const auto& [key, value] : event.fields) {
        if (key == name) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::string field_string(const TraceEvent& event, const std::string& name) {
    for (const auto& [key, value] : event.fields) {
        if (key == name) {
            return std::get<std::string>(value);
        }
    }
    return {};
}

[[nodiscard]] std::int64_t field_int(const TraceEvent& event, const std::string& name) {
    for (const auto& [key, value] : event.fields) {
        if (key == name) {
            return std::get<std::int64_t>(value);
        }
    }
    return -1;
}

// The committed fixture: perfect GNSS, an outage at 750, dead-reckoned
// drift while moving, perfect again from 2750 — exactly one reacquisition.
TEST(ScenarioReacquisitionTest, FixtureEmitsOneDriftedReacquisitionAtRestoreTick) {
    const Scenario scenario =
        ScenarioLoader::load(grid().base, "scenarios/gnss_reacquisition.json");
    const CapturingSink sink = run(scenario, 23);

    const TraceEvent* reacquisition = nullptr;
    int no_fix_samples = 0;
    int fix_samples = 0;
    for (const TraceEvent& event : sink.events()) {
        if (event.type != "gnss_sample") {
            continue;
        }
        if (has_field(event, "reacquired")) {
            ASSERT_EQ(reacquisition, nullptr);  // exactly one, ever
            reacquisition = &event;
        }
        if (field_string(event, "outcome") == "no_fix") {
            ++no_fix_samples;
        } else {
            ++fix_samples;
        }
    }
    ASSERT_NE(reacquisition, nullptr);

    // Last fix before the outage: 500. Restore switch at 2750 precedes the
    // same-tick sample (ADR-018), so the reacquisition runs exactly there.
    EXPECT_EQ(reacquisition->at, Tick{2750});
    EXPECT_EQ(field_string(*reacquisition, "prior_source"), "dead_reckoning");
    EXPECT_EQ(field_int(*reacquisition, "since_last_fix_ms"), 2250);
    const double correction = std::stod(field_string(*reacquisition, "correction_m"));
    EXPECT_GT(correction, 5.0);   // 5% scale bias over hundreds of meters
    EXPECT_LT(correction, 100.0);
    // Outage samples at 750..2500 (250 ms period): eight misses.
    EXPECT_EQ(no_fix_samples, 8);
    EXPECT_EQ(fix_samples, 9);    // 0..500 plus 2750..4000: five + four
}

// The #16 contrast: no dead reckoning — the prior FROZE, so the correction
// is the full distance traveled since the last fix, and the prior source
// is reported as "gnss".
TEST(ScenarioReacquisitionTest, FrozenOutageReacquisitionReportsMotionSizedCorrection) {
    const std::string json = R"json({
      "name": "gnss_frozen_return",
      "seed": 5,
      "movement": { "ms_per_cost_unit": 1000 },
      "localization": {
        "gnss": { "period_ms": 250, "initial_model": "perfect" }
      },
      "duration_ms": 2500,
      "robots": [
        { "name": "robot_a", "id": 1, "endpoint": 1, "mission": { "start": "A", "goal": "D" } }
      ],
      "events": [
        { "at_ms": 750, "action": "set_gnss_model", "robot": "robot_a", "model": "unavailable" },
        { "at_ms": 1750, "action": "set_gnss_model", "robot": "robot_a", "model": "perfect" }
      ]
    })json";
    const Scenario scenario = ScenarioLoader::load_string(grid().base, json);
    const CapturingSink sink = run(scenario, 5);

    const TraceEvent* reacquisition = nullptr;
    for (const TraceEvent& event : sink.events()) {
        if (event.type == "gnss_sample" && has_field(event, "reacquired")) {
            ASSERT_EQ(reacquisition, nullptr);
            reacquisition = &event;
        }
    }
    ASSERT_NE(reacquisition, nullptr);
    EXPECT_EQ(reacquisition->at, Tick{1750});
    EXPECT_EQ(field_string(*reacquisition, "prior_source"), "gnss");  // frozen, not drifted
    EXPECT_EQ(field_int(*reacquisition, "since_last_fix_ms"), 1250);

    // Frozen belief at the 500 ms fix (half of A->B, ~68 m east); truth at
    // 1750 is 1.25 grid edges further east (~170 m): the correction is the
    // full motion since the last fix, due east.
    const double correction = std::stod(field_string(*reacquisition, "correction_m"));
    EXPECT_GT(correction, 150.0);
    EXPECT_LT(correction, 190.0);
    const double bearing = std::stod(field_string(*reacquisition, "correction_heading_rad"));
    EXPECT_NEAR(bearing, 1.5707963, 1e-3);
}

// The non-regression promise of ADR-021: fixtures that never restore a
// fix-producing model must not contain a single reacquisition field.
TEST(ScenarioReacquisitionTest, ExistingLocalizationFixturesNeverReacquire) {
    for (const char* file : {"scenarios/gnss_outage.json", "scenarios/dead_reckoning.json"}) {
        SCOPED_TRACE(file);
        const Scenario scenario = ScenarioLoader::load(grid().base, file);
        const CapturingSink sink = run(scenario, 17);
        for (const TraceEvent& event : sink.events()) {
            if (event.type == "gnss_sample") {
                EXPECT_FALSE(has_field(event, "reacquired"));
                EXPECT_FALSE(has_field(event, "correction_m"));
            }
        }
    }
}

TEST(ScenarioReacquisitionTest, ReacquisitionTraceIsByteDeterministic) {
    const Scenario scenario =
        ScenarioLoader::load(grid().base, "scenarios/gnss_reacquisition.json");
    EXPECT_EQ(jsonl_trace(scenario, 23), jsonl_trace(scenario, 23));
}

}  // namespace

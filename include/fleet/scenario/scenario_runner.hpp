#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/localization/gnss_model.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/robot/robot.hpp"
#include "fleet/robot/robot_state.hpp"
#include "fleet/scenario/scenario.hpp"
#include "fleet/scenario/trace.hpp"
#include "fleet/station/control_station.hpp"
#include "fleet/world/observation_model.hpp"
#include "fleet/world/world.hpp"

namespace fleet::simulation {
class EventQueue;
class DeterministicRng;
}

namespace fleet::network {
class NetworkSimulator;
}

namespace fleet::scenario {

// Executes a Scenario by wiring existing public APIs together and
// scheduling the scenario's declared effects on the EventQueue (ADR-009).
// The runner is wiring + scheduling ONLY: every piece of behavior —
// reconciliation, replanning, transport faults, convergence — belongs to
// Robot / NetworkSimulator / ControlStation and is unchanged by running
// through this path.
//
// Determinism: a (scenario, resolved seed) pair produces a bit-identical
// event stream, therefore a bit-identical trace. Equal-tick scenario
// events execute in file order. Robot delta fan-out order is robot
// declaration order, then the station last.
//
// Wiring performed per run:
//   - one EventQueue and one NetworkSimulator(config, resolved seed);
//   - one Robot per ScenarioRobot, each with a sink that sends to every
//     OTHER participant (station last);
//   - one delivery handler per participant: receive, then a "route" trace
//     whenever the receiver's route object actually changed;
//   - one ControlStation when the scenario declares one;
//   - initial "scenario"/"seed"/"route" trace events at tick 0;
//   - each ScenarioEvent scheduled at its tick through the public APIs.
//
// Localization wiring (#16, ADR-018), when enabled:
//   - the initial GNSS model per robot is constructed EAGERLY in the
//     constructor (the single model factory — configuration errors fail
//     construction, not the first sample);
//   - ONE DeterministicRng PER ROBOT, derived from the resolved seed by
//     fully specified arithmetic (derive_stream_seed; stable RobotId,
//     fixed GNSS domain constant) — one robot's GNSS noise sequence is
//     independent of every other robot's presence and sampling;
//   - one sampling chain per robot: the first sample runs at tick 0,
//     and every subsequent sample runs exactly period_ms later
//     (strictly later — no zero-time self-scheduling);
//   - a sample derives GroundTruthPose on the simulation side from
//     movement state, map geometry, logical time, and runner-maintained
//     physical orientation. Initial physical orientation is north;
//     after a traversal completes, the final-segment bearing becomes the
//     stationary physical orientation;
//   - truth is measured through the ACTIVE GnssModel and only the
//     resulting optional LocalizationEstimate crosses into the robot's
//     LocalizationTracker (fix replaces, no-fix retains);
//   - each sample emits a "gnss_sample" trace event and reschedules the
//     same robot's next sample;
//   - set_gnss_model replaces the active model and causes NO immediate
//     sample or RNG consumption;
//   - loaded scripted set_gnss_model events are scheduled before the
//     initial GNSS chains. Therefore a scripted switch at tick T takes
//     effect before a GNSS sample scheduled for the same T, including
//     T = 0 (test-locked). This is a GNSS-specific same-tick guarantee,
//     not a universal priority ordering among all event classes.
//
// Lifetime: borrows `base` and every added sink; both must outlive the
// runner. Sinks are observation-only and never influence the run.
//
// Thread-safety: not synchronized (ADR-002).
class ScenarioRunner {
public:
    struct Result {
        std::uint64_t resolved_seed = 0;
        common::Tick finished_at{};
        std::size_t robot_count = 0;
        bool had_station = false;
    };

    ScenarioRunner(const map::BaseMap& base, Scenario scenario,
                   std::uint64_t resolved_seed);

    // Out-of-line: members are held by unique_ptr through forward declarations.
    ~ScenarioRunner();

    ScenarioRunner(const ScenarioRunner&) = delete;
    ScenarioRunner& operator=(const ScenarioRunner&) = delete;

    // Observation only; never affects the run. Must outlive the runner.
    void add_sink(TraceSink& sink);

    // --- interactive stepping (ADR-009 extension; #13) ---------------------
    //
    // run_to_completion() = begin() + run to the scenario horizon (or
    // queue exhaustion). Interactive callers instead: begin() once, then
    // run_until() in arbitrary chunks and inject() events at future
    // ticks. Stepping is semantically identical to one-shot execution:
    // the same events execute in the same (tick, enqueue) order, so a
    // stepped run produces a byte-identical trace.

    // Wires the world and schedules all scenario events. Idempotent:
    // the first call does the work, later calls are no-ops.
    void begin();

    // Advances to exactly `until` (ADR-005 horizon semantics: events
    // with tick <= until run, the clock lands on until). Throws
    // std::invalid_argument when until < now. Implicitly begin()s.
    Result run_until(common::Tick until);

    // Current logical time (implicitly begin()s; a run at t=0 is legal).
    common::Tick now();

    // Schedules one additional scenario event at its declared tick.
    // Requires begin() (the world must exist) and event.at >= now();
    // throws std::logic_error / std::invalid_argument otherwise. The
    // event executes through the same effect path as loaded events —
    // the console owns no semantics of its own.
    void inject(const ScenarioEvent& event);

    // Runs the queue to the scenario horizon (duration_ms) or to
    // exhaustion. Same behavior as before the stepping API existed.
    Result run_to_completion();

    // Post-run access for tests and callers. robot() throws
    // std::invalid_argument for an unknown name.
    [[nodiscard]] const robot::Robot& robot(std::string_view name) const;
    [[nodiscard]] std::optional<double> localization_position_error_m(std::string_view name) const;
    [[nodiscard]] const station::ControlStation* station() const noexcept;

private:
    void emit(TraceEvent event);
    void wire_world();
    void wire_delivery_handler(std::size_t index);
    void schedule_events();
    void apply_scenario_event(const ScenarioEvent& event);
    void advance_robot(std::size_t index);
    void sense_for(std::size_t index, common::Tick now);

    // --- localization wiring (#16, ADR-018) -------------------------------

    // The single place a GnssModelSpec becomes a model: constructing the
    // initial models and every set_gnss_model switch goes through here,
    // so configuration validation (e.g. NoisyGnss's finite non-negative
    // sigmas) fails in exactly one way.
    [[nodiscard]] static std::unique_ptr<localization::GnssModel>
    make_gnss_model(const GnssModelSpec& spec);

    // One GNSS sample for one robot:
    //
    // simulation truth
    //       ->
    // active GnssModel
    //       ->
    // optional LocalizationEstimate
    //       ->
    // robot-local LocalizationTracker
    //       ->
    // "gnss_sample" trace
    //       ->
    // reschedule at now + period_ms
    void sample_gnss(std::size_t index);

    // Starts the per-robot sampling chains (tick 0). Called from begin()
    // AFTER schedule_events(): loaded model switches precede same-tick
    // samples; other event classes retain enqueue order (ADR-018).
    void start_localization_chains();

    [[nodiscard]] std::size_t index_of_robot(std::string_view name) const;
    [[nodiscard]] std::size_t index_of_robot(common::RobotId id) const;
    [[nodiscard]] std::string robot_name_of(common::RobotId id) const;
    [[nodiscard]] std::string target_name(std::size_t target_index) const;
    [[nodiscard]] network::EndpointId target_endpoint(
        std::size_t target_index) const;
    [[nodiscard]] std::string edge_label(common::EdgeId edge) const;
    [[nodiscard]] std::string node_name(common::NodeId node) const;

    const map::BaseMap& base_;
    Scenario scenario_;
    std::uint64_t resolved_seed_;
    bool begun_ = false;

    std::vector<TraceSink*> sinks_;

    std::unique_ptr<simulation::EventQueue> queue_;
    std::unique_ptr<network::NetworkSimulator> network_;
    std::unique_ptr<world::World> world_;
    std::unique_ptr<world::ObservationModel> sensor_;  // set when sensing enabled
    std::vector<std::unique_ptr<robot::Robot>> robots_;
    std::unique_ptr<station::ControlStation> station_;

    // Localization (#16, ADR-018). When localization is enabled these
    // containers have one entry per declared robot, in declaration order.

    // ACTIVE GNSS model per robot. An outage is represented solely by
    // UnavailableGnss occupying the corresponding slot.
    std::vector<std::unique_ptr<localization::GnssModel>> gnss_models_;

    // ONE deterministic GNSS RNG PER ROBOT. Seeds are derived as:
    //
    //   derive_stream_seed(
    //       resolved_seed,
    //       kGnssStreamDomain,
    //       stable RobotId)
    //
    // Therefore GNSS sampling by unrelated robots cannot shift this
    // robot's noise sequence. Perfect, Unavailable, and zero-noise
    // models preserve their zero-consumption contracts.
    std::vector<std::unique_ptr<simulation::DeterministicRng>> gnss_rngs_;

    // SIMULATION-SIDE physical orientation truth per robot.
    //
    // Initial condition: north (0 rad).
    // While moving: truth_pose derives heading from travel geometry.
    // At arrival: the completed traversal's final-segment bearing is
    // retained here and becomes the stationary physical orientation.
    //
    // Deliberately NOT part of Robot/RobotState: autonomy must not gain
    // an oracle path to simulation-truth orientation.
    std::vector<double> rest_heading_rad_;
};

}  // namespace fleet::scenario
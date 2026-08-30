#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
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

    ScenarioRunner(const map::BaseMap& base, Scenario scenario, std::uint64_t resolved_seed);

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
    [[nodiscard]] const station::ControlStation* station() const noexcept;

private:
    void emit(TraceEvent event);
    void wire_world();
    void wire_delivery_handler(std::size_t index);
    void schedule_events();
    void apply_scenario_event(const ScenarioEvent& event);
    void advance_robot(std::size_t index);
    void sense_for(std::size_t index, common::Tick now);

    [[nodiscard]] std::size_t index_of_robot(std::string_view name) const;
    [[nodiscard]] std::size_t index_of_robot(common::RobotId id) const;
    [[nodiscard]] std::string robot_name_of(common::RobotId id) const;
    [[nodiscard]] std::string target_name(std::size_t target_index) const;
    [[nodiscard]] network::EndpointId target_endpoint(std::size_t target_index) const;
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
};

}  // namespace fleet::scenario

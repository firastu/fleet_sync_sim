#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

#include "fleet/common/time.hpp"
#include "fleet/robot/terrain_robot.hpp"
#include "fleet/scenario/terrain_scenario.hpp"
#include "fleet/scenario/trace.hpp"
#include "fleet/world/terrain_motion.hpp"

namespace fleet::simulation {
class EventQueue;
}

namespace fleet::scenario {

// Executes a TerrainScenario (ADR-023, M4 #20): orchestration and wiring
// for the terrain domain, as ScenarioRunner is for the road-graph domain.
// It owns no motion semantics — those belong to world::TerrainMotion — and
// shares the EventQueue, trace and seed contracts of ADR-005/009.
//
// Wiring per run:
//   - one EventQueue;
//   - per robot (declaration order): one world::TerrainMotion holding the
//     truth body on the truth terrain, and one robot::TerrainRobot holding
//     the robot's mission and its own terrain map handle (ADR-022 roles);
//   - tick-0 trace events: "scenario" (name, domain), "seed", "terrain"
//     (truth surface and datum, and whether robots carry the truth grid);
//   - one motion chain per robot: at tick 0 an initial "terrain_pose"; then
//     every step_ms (strictly later, ADR-005/010) one TerrainMotion step,
//     a "waypoint_reached" event per waypoint reached and a "terrain_pose".
//     The chain ends once the last waypoint is reached.
//
// Every robot's motion is validated in the constructor (configuration, step
// bound, start inside the truth terrain), so a bad scenario fails before the
// run. A body that later leaves the truth terrain fails the run with a
// std::runtime_error naming the robot — truth is never manufactured.
//
// Trace events carry SIMULATION TRUTH under source "world": in #20 robots
// have no belief to report yet. Positions and headings are 6-decimal
// strings, as in "gnss_sample".
//
// Determinism: a (scenario, resolved seed) pair produces a byte-identical
// trace. The terrain domain draws no randomness in #20; the resolved seed is
// recorded for the sensor streams that follow (#21).
//
// Thread-safety: not synchronized (ADR-002).
class TerrainScenarioRunner {
public:
    struct Result {
        std::uint64_t resolved_seed = 0;
        common::Tick finished_at{};
        std::size_t robot_count = 0;
    };

    // Throws std::invalid_argument for a scenario that cannot run: missing
    // terrain, duration_ms or step_ms of 0, no robots, duplicate names or
    // ids, or any robot whose motion TerrainMotion rejects.
    TerrainScenarioRunner(TerrainScenario scenario, std::uint64_t resolved_seed);
    ~TerrainScenarioRunner();

    TerrainScenarioRunner(const TerrainScenarioRunner&) = delete;
    TerrainScenarioRunner& operator=(const TerrainScenarioRunner&) = delete;

    // Observation only; never affects the run. Must outlive the runner.
    void add_sink(TraceSink& sink);

    // Schedules the run. Idempotent.
    void begin();

    // Advances to exactly `until` (ADR-005). Implicitly begin()s.
    Result run_until(common::Tick until);

    common::Tick now();

    // Runs to the duration_ms horizon.
    Result run_to_completion();

    // Post-run access. Both throw std::invalid_argument for an unknown name.
    [[nodiscard]] const robot::TerrainRobot& robot(std::string_view name) const;
    // Simulation truth, for tests and diagnostics only.
    [[nodiscard]] const world::TerrainMotion& truth_motion(std::string_view name) const;

private:
    void emit(TraceEvent event);
    void emit_pose(std::size_t index);
    void step_robot(std::size_t index);
    [[nodiscard]] std::size_t index_of(std::string_view name) const;

    TerrainScenario scenario_;
    std::uint64_t resolved_seed_;
    bool begun_ = false;
    std::vector<TraceSink*> sinks_;
    std::unique_ptr<simulation::EventQueue> queue_;
    std::vector<world::TerrainMotion> bodies_;  // truth, one per robot
    std::vector<robot::TerrainRobot> robots_;   // robot side, one per robot
};

}  // namespace fleet::scenario

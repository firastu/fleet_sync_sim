#include "fleet/scenario/terrain_scenario_runner.hpp"

#include <cstdint>
#include <format>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "fleet/simulation/event_queue.hpp"

namespace fleet::scenario {

namespace {

[[nodiscard]] std::string surface_name(map::ElevationSurface surface) {
    switch (surface) {
        case map::ElevationSurface::SurfaceModel:
            return "surface_model";
        case map::ElevationSurface::BareEarth:
            return "bare_earth";
    }
    return "?";
}

[[nodiscard]] std::string datum_name(map::VerticalDatum datum) {
    switch (datum) {
        case map::VerticalDatum::Egm96:
            return "egm96";
        case map::VerticalDatum::Synthetic:
            return "synthetic";
    }
    return "?";
}

}  // namespace

TerrainScenarioRunner::TerrainScenarioRunner(TerrainScenario scenario,
                                             std::uint64_t resolved_seed)
    : scenario_{std::move(scenario)}, resolved_seed_{resolved_seed} {
    if (scenario_.truth_terrain == nullptr || scenario_.robot_terrain_map == nullptr) {
        throw std::invalid_argument(
            "TerrainScenarioRunner: truth terrain and robot terrain map are required");
    }
    if (scenario_.duration_ms == 0 || scenario_.step_ms == 0) {
        throw std::invalid_argument(
            "TerrainScenarioRunner: duration_ms and step_ms must be >= 1");
    }
    if (scenario_.robots.empty()) {
        throw std::invalid_argument("TerrainScenarioRunner: at least one robot is required");
    }
    std::set<std::string> names;
    std::set<std::uint8_t> ids;
    for (const TerrainScenarioRobot& entry : scenario_.robots) {
        if (!names.insert(entry.name).second) {
            throw std::invalid_argument(
                std::format("TerrainScenarioRunner: duplicate robot name '{}'", entry.name));
        }
        if (!ids.insert(entry.id.value()).second) {
            throw std::invalid_argument(
                std::format("TerrainScenarioRunner: duplicate robot id {}", entry.id.value()));
        }
        // Eager validation: a bad robot fails construction, not the run.
        try {
            world::TerrainMotion::validate_step(entry.motion, scenario_.step_ms);
            bodies_.emplace_back(entry.motion, entry.waypoints, scenario_.truth_terrain,
                                 entry.start, entry.start_heading_rad, common::Tick{0});
            robots_.emplace_back(entry.id, robot::TerrainMission{entry.waypoints},
                                 scenario_.robot_terrain_map);
        } catch (const std::invalid_argument& error) {
            throw std::invalid_argument(
                std::format("TerrainScenarioRunner: robot '{}': {}", entry.name, error.what()));
        }
    }
}

TerrainScenarioRunner::~TerrainScenarioRunner() = default;

void TerrainScenarioRunner::add_sink(TraceSink& sink) {
    sinks_.push_back(&sink);
}

void TerrainScenarioRunner::emit(TraceEvent event) {
    for (TraceSink* sink : sinks_) {
        sink->record(event);
    }
}

std::size_t TerrainScenarioRunner::index_of(std::string_view name) const {
    for (std::size_t index = 0; index < scenario_.robots.size(); ++index) {
        if (scenario_.robots[index].name == name) {
            return index;
        }
    }
    throw std::invalid_argument(std::format("terrain scenario runner: unknown robot '{}'", name));
}

const robot::TerrainRobot& TerrainScenarioRunner::robot(std::string_view name) const {
    return robots_.at(index_of(name));
}

const world::TerrainMotion& TerrainScenarioRunner::truth_motion(std::string_view name) const {
    return bodies_.at(index_of(name));
}

void TerrainScenarioRunner::emit_pose(std::size_t index) {
    const world::TerrainBodyState& state = bodies_[index].state();
    emit(TraceEvent{
        state.at,
        "world",
        "terrain_pose",
        {{"robot", scenario_.robots[index].name},
         {"position", std::format("{:.6f},{:.6f}", state.position.latitude_deg,
                                  state.position.longitude_deg)},
         {"heading", std::format("{:.6f}", state.heading_rad)},
         {"altitude_m", state.altitude_m},
         {"terrain_m", state.terrain_height_m},
         {"agl_m", state.altitude_m - state.terrain_height_m},
         {"next_waypoint", static_cast<std::int64_t>(state.next_waypoint)}}});
}

void TerrainScenarioRunner::step_robot(std::size_t index) {
    world::TerrainMotion& body = bodies_[index];
    world::TerrainStepOutcome outcome;
    try {
        outcome = body.step(scenario_.step_ms);
    } catch (const std::runtime_error& error) {
        throw std::runtime_error(std::format("terrain scenario: robot '{}': {}",
                                             scenario_.robots[index].name, error.what()));
    }
    for (const std::size_t reached : outcome.reached_waypoints) {
        emit(TraceEvent{body.state().at,
                        "world",
                        "waypoint_reached",
                        {{"robot", scenario_.robots[index].name},
                         {"index", static_cast<std::int64_t>(reached)},
                         {"last", reached + 1 == body.waypoints().size()}}});
    }
    emit_pose(index);
    if (!body.mission_complete()) {
        // Strictly later (step_ms >= 1, constructor-enforced): no zero-time loop.
        queue_->schedule(body.state().at + scenario_.step_ms, [this, index] { step_robot(index); });
    }
}

void TerrainScenarioRunner::begin() {
    if (begun_) {
        return;
    }
    begun_ = true;
    queue_ = std::make_unique<simulation::EventQueue>();
    emit(TraceEvent{common::Tick{0},
                    "world",
                    "scenario",
                    {{"name", scenario_.name}, {"domain", std::string{"terrain"}}}});
    emit(TraceEvent{common::Tick{0},
                    "world",
                    "seed",
                    {{"value", static_cast<std::int64_t>(resolved_seed_)}}});
    emit(TraceEvent{
        common::Tick{0},
        "world",
        "terrain",
        {{"surface", surface_name(scenario_.truth_terrain->surface())},
         {"datum", datum_name(scenario_.truth_terrain->vertical_datum())},
         {"robot_map", std::string{scenario_.robot_terrain_map == scenario_.truth_terrain
                                       ? "truth"
                                       : "separate"}}}});
    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        queue_->schedule(common::Tick{0}, [this, index] {
            emit_pose(index);
            if (!bodies_[index].mission_complete()) {
                queue_->schedule(common::Tick{scenario_.step_ms},
                                 [this, index] { step_robot(index); });
            }
        });
    }
}

TerrainScenarioRunner::Result TerrainScenarioRunner::run_until(common::Tick until) {
    begin();
    queue_->run_until(until);  // throws std::invalid_argument if until < now
    return Result{resolved_seed_, queue_->clock().now(), robots_.size()};
}

common::Tick TerrainScenarioRunner::now() {
    begin();
    return queue_->clock().now();
}

TerrainScenarioRunner::Result TerrainScenarioRunner::run_to_completion() {
    begin();
    const common::Tick horizon{scenario_.duration_ms};
    if (horizon >= queue_->clock().now()) {
        queue_->run_until(horizon);
    }
    return Result{resolved_seed_, queue_->clock().now(), robots_.size()};
}

}  // namespace fleet::scenario

#pragma once

#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/robot/robot_state.hpp"
#include "fleet/world/world.hpp"

namespace fleet::world {

// What a sensor reports about one edge (ADR-011): the measured truth.
struct EdgeObservation {
    common::EdgeId edge{};
    map::EdgeStatus status{};
};

// The ONLY path from world truth to robot perception (ADR-011). An
// ObservationModel is a PURE PERCEPTION function of (World truth, robot
// state): it measures what is currently observable, without any knowledge
// of what the robot already believes. Belief comparison, filtering and
// suppression of unchanged facts are downstream concerns (the simulation
// wiring), so later models — noisy sensors, repeated measurements,
// confidence accumulation, state estimation — never have to un-learn a
// belief dependency. The wiring turns returned observations into
// Robot::observe() calls.
//
// Determinism contract: implementations return observations in ascending
// EdgeId order and are pure functions of their inputs.
class ObservationModel {
public:
    virtual ~ObservationModel() = default;

    // Everything currently observable for a robot in `state`.
    [[nodiscard]] virtual std::vector<EdgeObservation> sense(
        const World& world, const robot::RobotState& state) const = 0;
};

// Stage-0 sensor, deliberately artificial and deterministic (ADR-011):
// observable range = every edge incident to the node the robot occupies
// (while in transit, the occupied node is the traversal's departure node —
// the transit edge is incident to it and therefore covered; the robot
// learns what changed at the upcoming node when it arrives). Perfect
// accuracy, confidence 1.0. Reports the measured truth of ALL observable
// edges — including ones whose truth matches what the robot may already
// believe: suppression of unchanged facts is coordinator policy, not
// sensor behavior.
class PerfectLocalEdgeSensor final : public ObservationModel {
public:
    [[nodiscard]] std::vector<EdgeObservation> sense(
        const World& world, const robot::RobotState& state) const override;
};

}  // namespace fleet::world

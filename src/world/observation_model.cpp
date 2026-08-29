#include "fleet/world/observation_model.hpp"

#include <algorithm>

namespace fleet::world {

std::vector<EdgeObservation> PerfectLocalEdgeSensor::sense(
    const World& world, const robot::RobotState& state) const {
    std::vector<EdgeObservation> observations;
    for (const map::AdjacencyEntry& entry : world.base().graph().adjacency(state.position)) {
        observations.push_back(EdgeObservation{entry.edge, world.edge_state(entry.edge)});
    }
    // CSR adjacency order is edge-insertion order, not EdgeId order: sort
    // for the deterministic output contract.
    std::sort(observations.begin(), observations.end(),
              [](const EdgeObservation& a, const EdgeObservation& b) {
                  return a.edge < b.edge;
              });
    return observations;
}

}  // namespace fleet::world

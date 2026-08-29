#include "fleet/world/world.hpp"

#include <stdexcept>

namespace fleet::world {

World::World(const map::BaseMap& base)
    : base_{base}, truth_(base.graph().edge_count(), map::EdgeStatus::Open) {}

void World::set_edge_state(common::EdgeId edge, map::EdgeStatus status) {
    if (edge.value() >= truth_.size()) {
        throw std::invalid_argument("World: unknown edge");
    }
    truth_[edge.value()] = status;
}

map::EdgeStatus World::edge_state(common::EdgeId edge) const {
    if (edge.value() >= truth_.size()) {
        throw std::invalid_argument("World: unknown edge");
    }
    return truth_[edge.value()];
}

}  // namespace fleet::world

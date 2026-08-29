#include "fleet/map/dynamic_overlay.hpp"

#include <utility>

namespace fleet::map {

DynamicMapOverlay::DynamicMapOverlay(std::size_t edge_count) : states_(edge_count) {}

bool DynamicMapOverlay::apply(common::EdgeId edge, EdgeDynamicState state) {
    assert(edge.value() < states_.size());
    std::optional<EdgeDynamicState>& slot = states_[edge.value()];
    if (slot.has_value() && *slot == state) {
        return false;
    }
    if (!slot.has_value()) {
        ++tracked_count_;
    }
    slot = std::move(state);
    version_ = common::OverlayVersion{version_.value() + 1};
    return true;
}

const EdgeDynamicState* DynamicMapOverlay::find(common::EdgeId edge) const noexcept {
    if (edge.value() >= states_.size()) {
        return nullptr;
    }
    const std::optional<EdgeDynamicState>& slot = states_[edge.value()];
    return slot.has_value() ? &*slot : nullptr;
}

std::vector<common::EdgeId> DynamicMapOverlay::tracked_edges() const {
    std::vector<common::EdgeId> edges;
    edges.reserve(tracked_count_);
    for (std::size_t i = 0; i < states_.size(); ++i) {
        if (states_[i].has_value()) {
            edges.push_back(common::EdgeId{static_cast<std::uint32_t>(i)});
        }
    }
    return edges;
}

}  // namespace fleet::map

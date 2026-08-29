#include "fleet/map/map_view.hpp"

namespace fleet::map {

MapView::MapView(const BaseMap& base, const DynamicMapOverlay& overlay) noexcept
    : base_{&base}, overlay_{&overlay} {}

std::span<const AdjacencyEntry> MapView::adjacency(common::NodeId node) const noexcept {
    return base_->graph().adjacency(node);
}

bool MapView::is_blocked(common::EdgeId edge) const noexcept {
    const EdgeDynamicState* state = overlay_->find(edge);
    return state != nullptr && state->status == EdgeStatus::Blocked;
}

std::optional<double> MapView::traversal_cost(common::EdgeId edge) const noexcept {
    if (is_blocked(edge)) {
        return std::nullopt;
    }
    return base_->graph().edge(edge).base_cost;
}

const EdgeDynamicState* MapView::dynamic_state(common::EdgeId edge) const noexcept {
    return overlay_->find(edge);
}

}  // namespace fleet::map

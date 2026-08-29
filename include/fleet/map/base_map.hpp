#pragma once

#include "fleet/common/ids.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::map {

// The immutable baseline topology every participant starts from. All
// participants of a simulation run share the same MapVersion; a revision of
// the road network is a new BaseMap object, never in-place mutation
// (see ADR-001).
//
// Thread-safety: immutable after construction; safe to share without
// synchronization.
class BaseMap {
public:
    BaseMap(Graph graph, common::MapVersion version) noexcept;

    [[nodiscard]] const Graph& graph() const noexcept { return graph_; }
    [[nodiscard]] common::MapVersion version() const noexcept { return version_; }

private:
    Graph graph_;
    common::MapVersion version_{0};
};

}  // namespace fleet::map

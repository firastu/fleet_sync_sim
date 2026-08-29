#include "fleet/map/base_map.hpp"

#include <utility>

namespace fleet::map {

BaseMap::BaseMap(Graph graph, common::MapVersion version) noexcept
    : graph_{std::move(graph)}, version_{version} {}

}  // namespace fleet::map

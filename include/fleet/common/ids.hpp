#pragma once

#include <cstdint>

#include "fleet/common/strong_type.hpp"

namespace fleet::common {

// --- Tags (empty types used only to distinguish StrongType instances) -------

struct NodeIdTag {};
struct EdgeIdTag {};
struct RobotIdTag {};
struct MapVersionTag {};
struct OverlayVersionTag {};

// Dense index into a Graph's node table. Issued by Graph::Builder.
using NodeId = StrongType<NodeIdTag, std::uint32_t>;
// Dense index into a Graph's edge table. Issued by Graph::Builder.
using EdgeId = StrongType<EdgeIdTag, std::uint32_t>;
// Identifies a fleet member (robot or control station) within a simulation.
using RobotId = StrongType<RobotIdTag, std::uint8_t>;
// Revision of the immutable base map that all participants agreed on at start.
using MapVersion = StrongType<MapVersionTag, std::uint32_t>;
// Monotonic counter of state changes applied to a DynamicMapOverlay.
using OverlayVersion = StrongType<OverlayVersionTag, std::uint32_t>;

}  // namespace fleet::common

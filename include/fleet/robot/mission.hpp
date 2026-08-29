#pragma once

#include <compare>

#include "fleet/common/ids.hpp"

namespace fleet::robot {

// A mission: where the robot must go. Stage 0 has no movement yet; the
// mission defines the planning query and, later, the completion target.
struct Mission {
    common::NodeId start{};
    common::NodeId goal{};

    auto operator<=>(const Mission&) const = default;
};

}  // namespace fleet::robot

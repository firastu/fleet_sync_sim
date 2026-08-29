#pragma once

#include <cstdint>
#include <optional>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"

namespace fleet::robot {

// One committed edge traversal in progress (ADR-010). Created by
// begin_transit(), committed by complete_transit(); the traversal is
// physically committed — later knowledge changes never cancel it.
struct RobotTransit {
    common::EdgeId edge{};
    common::NodeId from{};
    common::NodeId to{};
    common::Tick arrival{};  // earliest tick complete_transit() may commit
};

// Where a robot is and whether its mission is done (ADR-010).
//
// Stage 0 motion is node-resident: `position` is the last node reached;
// while `in_transit` holds, the robot is physically on that edge and
// `position` still names the departure node until the arrival commits.
// There is deliberately no continuous (sub-edge) position — planning
// needs node-granular starts, and finer motion models arrive with the
// geospatial stages.
struct RobotState {
    common::NodeId position{};
    std::optional<RobotTransit> in_transit;
    bool mission_complete = false;
};

}  // namespace fleet::robot

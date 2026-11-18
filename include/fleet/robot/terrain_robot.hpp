#pragma once

#include <memory>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/elevation_grid.hpp"
#include "fleet/map/geometry.hpp"

namespace fleet::robot {

// Where a terrain-domain robot must go (ADR-023): an ordered waypoint list.
struct TerrainMission {
    std::vector<map::Wgs84Coordinate> waypoints;
};

// The robot-side participant of a terrain-domain scenario (ADR-023, M4 #20).
//
// It holds what the ROBOT has: its mission and its own terrain map handle
// (ADR-022's robot role). It never holds truth: no TerrainBodyState, no truth
// terrain, no fleet::world dependency (ADR-007/011). In #20 the body moves by
// scripted truth, so the robot has no behavior yet; sensors (#21), the
// estimator (#22) and guidance (#23) attach here.
//
// The terrain map may be the same immutable grid as the world's truth terrain
// (a perfect map) or a different one; the robot cannot tell, by design.
//
// Thread-safety: immutable after construction.
class TerrainRobot {
public:
    // Throws std::invalid_argument for a null map, an empty mission or a
    // waypoint outside WGS84 bounds.
    TerrainRobot(common::RobotId id, TerrainMission mission,
                 std::shared_ptr<const map::ElevationGrid> terrain_map);

    [[nodiscard]] common::RobotId id() const noexcept { return id_; }
    [[nodiscard]] const TerrainMission& mission() const noexcept { return mission_; }
    [[nodiscard]] const map::ElevationGrid& terrain_map() const noexcept { return *terrain_map_; }

private:
    common::RobotId id_;
    TerrainMission mission_;
    std::shared_ptr<const map::ElevationGrid> terrain_map_;
};

}  // namespace fleet::robot

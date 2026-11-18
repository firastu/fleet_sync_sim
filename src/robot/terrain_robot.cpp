#include "fleet/robot/terrain_robot.hpp"

#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

namespace fleet::robot {

TerrainRobot::TerrainRobot(common::RobotId id, TerrainMission mission,
                           std::shared_ptr<const map::ElevationGrid> terrain_map)
    : id_{id}, mission_{std::move(mission)}, terrain_map_{std::move(terrain_map)} {
    if (terrain_map_ == nullptr) {
        throw std::invalid_argument("TerrainRobot: a terrain map is required");
    }
    if (mission_.waypoints.empty()) {
        throw std::invalid_argument("TerrainRobot: the mission needs at least one waypoint");
    }
    for (std::size_t index = 0; index < mission_.waypoints.size(); ++index) {
        const map::Wgs84Coordinate& waypoint = mission_.waypoints[index];
        if (!std::isfinite(waypoint.latitude_deg) || !std::isfinite(waypoint.longitude_deg) ||
            std::abs(waypoint.latitude_deg) > 90.0 || std::abs(waypoint.longitude_deg) > 180.0) {
            throw std::invalid_argument(std::format(
                "TerrainRobot: waypoint {} must be finite and within WGS84 bounds", index));
        }
    }
}

}  // namespace fleet::robot

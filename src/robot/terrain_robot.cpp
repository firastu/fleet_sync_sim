#include "fleet/robot/terrain_robot.hpp"

#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

namespace fleet::robot {

TerrainRobot::TerrainRobot(common::RobotId id, TerrainMission mission,
                           std::shared_ptr<const map::ElevationGrid> terrain_map,
                           std::optional<localization::TerrainEstimatorConfig> estimator)
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
    if (estimator) {
        estimator_.emplace(*estimator, terrain_map_);
    }
}

const std::optional<localization::TerrainEstimate>& TerrainRobot::estimate() const noexcept {
    static const std::optional<localization::TerrainEstimate> kNone;
    return estimator_ ? estimator_->latest() : kNone;
}

void TerrainRobot::receive(const localization::TerrainSensorFrame& frame) {
    if (!(frame.at > frame.from)) {
        throw std::invalid_argument("TerrainRobot: a sensor frame must end after it starts");
    }
    if (last_frame_ && frame.from < last_frame_->at) {
        throw std::invalid_argument(
            "TerrainRobot: a sensor frame must not start before the previous one ended");
    }
    if (estimator_) {
        (void)estimator_->update(frame);  // throws before changing anything
    }
    last_frame_ = frame;
    ++frames_received_;
}

}  // namespace fleet::robot

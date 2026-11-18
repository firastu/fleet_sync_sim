#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/localization/terrain_estimator.hpp"
#include "fleet/localization/terrain_sensors.hpp"
#include "fleet/map/elevation_grid.hpp"
#include "fleet/map/geometry.hpp"

namespace fleet::robot {

// Where a terrain-domain robot must go (ADR-023): an ordered waypoint list.
struct TerrainMission {
    std::vector<map::Wgs84Coordinate> waypoints;
};

// The robot-side participant of a terrain-domain scenario (ADR-023, M4 #20).
//
// It holds what the ROBOT has: its mission, its own terrain map handle
// (ADR-022's robot role) and the sensor frames it receives (ADR-024). It never
// holds truth: no TerrainBodyState, no TerrainMotionTruth, no truth terrain,
// no fleet::world dependency (ADR-007/011). An optional terrain-referenced
// estimator (ADR-025) turns the frames into belief, using only the robot's
// own map. The body moves by scripted truth until guidance arrives in #23.
//
// The terrain map may be the same immutable grid as the world's truth terrain
// (a perfect map) or a different one; the robot cannot tell, by design.
//
// Thread-safety: not synchronized (ADR-002).
class TerrainRobot {
public:
    // Throws std::invalid_argument for a null map, an empty mission, a
    // waypoint outside WGS84 bounds, or an invalid estimator configuration.
    // The estimator, when configured, uses this robot's own terrain map.
    TerrainRobot(common::RobotId id, TerrainMission mission,
                 std::shared_ptr<const map::ElevationGrid> terrain_map,
                 std::optional<localization::TerrainEstimatorConfig> estimator = std::nullopt);

    [[nodiscard]] common::RobotId id() const noexcept { return id_; }
    [[nodiscard]] const TerrainMission& mission() const noexcept { return mission_; }
    [[nodiscard]] const map::ElevationGrid& terrain_map() const noexcept { return *terrain_map_; }

    // One sensor frame reaches the robot: measurements only (ADR-024). The
    // robot retains the latest frame, counts frames and, with an estimator,
    // updates its belief (ADR-025). Throws std::invalid_argument, changing
    // nothing, when the frame's interval does not start at or after the
    // previous frame's end or does not end after it starts, or when the
    // estimator rejects the frame.
    void receive(const localization::TerrainSensorFrame& frame);

    // The estimator's latest belief; nullopt without an estimator or before
    // the first frame.
    [[nodiscard]] const std::optional<localization::TerrainEstimate>& estimate() const noexcept;
    [[nodiscard]] bool has_estimator() const noexcept { return estimator_.has_value(); }

    // The latest frame; nullopt before the first.
    [[nodiscard]] const std::optional<localization::TerrainSensorFrame>& last_frame()
        const noexcept {
        return last_frame_;
    }
    [[nodiscard]] std::uint64_t frames_received() const noexcept { return frames_received_; }

private:
    common::RobotId id_;
    TerrainMission mission_;
    std::shared_ptr<const map::ElevationGrid> terrain_map_;
    std::optional<localization::TerrainSensorFrame> last_frame_;
    std::uint64_t frames_received_ = 0;
    std::optional<localization::TerrainGridEstimator> estimator_;
};

}  // namespace fleet::robot

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "fleet/common/time.hpp"
#include "fleet/map/elevation_grid.hpp"
#include "fleet/map/geometry.hpp"

namespace fleet::world {

// The largest horizontal displacement one integration step may take
// (ADR-023). Positions advance through localization::apply_en_displacement,
// whose local-tangent approximation is specified for meter-to-tens-of-meters
// displacements (ADR-017/022); this bound keeps every step inside it.
inline constexpr double kMaxTerrainStepDisplacementM = 50.0;

enum class TerrainPlatform : std::uint8_t {
    Ground,  // drives on the truth terrain surface
    Air,     // flies at a constant altitude above it
};

// Kinematic limits of one terrain-domain body (ADR-023). Deliberately
// kinematic: no physics, wind, slip or acceleration.
struct TerrainMotionConfig {
    TerrainPlatform platform = TerrainPlatform::Ground;
    double speed_mps = 1.0;                // > 0, constant
    double max_turn_rate_rad_per_s = 0.5;  // > 0
    // > 0 and at least the turning diameter 2 * speed / turn rate, so a
    // waypoint inside the body's turning circles counts as reached instead of
    // being orbited forever.
    double arrival_radius_m = 10.0;
    double mount_height_m = 0.0;  // ground only: body height above the surface, >= 0
    double altitude_m = 0.0;      // air only: constant altitude in the terrain datum

    // Throws std::invalid_argument naming the offending field. Ground bodies
    // must leave altitude_m at 0; air bodies must leave mount_height_m at 0.
    void validate() const;

    [[nodiscard]] double turning_diameter_m() const noexcept {
        return 2.0 * speed_mps / max_turn_rate_rad_per_s;
    }
};

// Where a terrain-domain body physically is: SIMULATION TRUTH (ADR-011/023).
// Never visible to robot autonomy; it reaches robots only through sensor
// models (M4 #21).
struct TerrainBodyState {
    map::Wgs84Coordinate position;
    double heading_rad = 0.0;       // clockwise from north, [0, 2*pi)
    double altitude_m = 0.0;        // body altitude in the terrain datum
    double terrain_height_m = 0.0;  // truth terrain directly below
    std::size_t next_waypoint = 0;  // == waypoint count once the mission is complete
    common::Tick at{};
};

// What one step did: observability, and the true motion increment that
// sensor models measure (ADR-024).
struct TerrainStepOutcome {
    std::vector<std::size_t> reached_waypoints;  // reached in this step, ascending
    bool mission_completed = false;              // the last waypoint was reached in this step
    double distance_m = 0.0;  // horizontal distance moved (0 when parked)
    double turn_rad = 0.0;    // heading change applied, clockwise positive (0 when parked)
};

// Scripted truth motion of one body along a waypoint list (ADR-023, M4 #20).
//
// Each step of dt = step_ms:
//   1. consecutive waypoints already within arrival_radius_m are reached;
//   2. once the last waypoint is reached the body stays where it is
//      (ground: parked; air: hovering) and only time advances;
//   3. otherwise the heading turns toward the next waypoint (initial great-
//      circle bearing) by at most max_turn_rate * dt; a target exactly
//      behind turns clockwise;
//   4. the body moves speed * dt along the new heading through
//      localization::apply_en_displacement, anchored at its current
//      position (ADR-022: no scenario-global projection);
//   5. the truth terrain below is looked up: a ground body's altitude is
//      terrain + mount_height_m; an air body keeps altitude_m, which must
//      stay strictly above the terrain;
//   6. step 1 repeats at the new position.
//
// Steering uses TRUE position: this is a script for simulation truth, not
// autonomy. Belief-driven guidance replaces it in #23.
//
// Failure is explicit, never manufactured (ADR-022): leaving truth terrain
// coverage, reaching a NODATA area, or an air body at or below the terrain
// surface throws std::runtime_error naming the position, and the state is
// left exactly as before the step (strong guarantee).
//
// Deterministic for a given toolchain: fixed operation order, libm
// trigonometry (the ADR-014/017 reproducibility class), no randomness.
//
// Thread-safety: not synchronized (ADR-002).
class TerrainMotion {
public:
    // Throws std::invalid_argument for an invalid config, a null terrain, an
    // empty waypoint list, an invalid coordinate or a non-finite heading, and
    // when the start is outside truth terrain coverage, on NODATA, or (air)
    // not strictly above the terrain.
    TerrainMotion(TerrainMotionConfig config, std::vector<map::Wgs84Coordinate> waypoints,
                  std::shared_ptr<const map::ElevationGrid> truth_terrain,
                  map::Wgs84Coordinate start, double start_heading_rad, common::Tick start_at);

    // Advances by step_ms ticks (one tick = one millisecond). Throws
    // std::invalid_argument when validate_step rejects step_ms, and
    // std::runtime_error as described above. Near the poles,
    // apply_en_displacement's explicit domain errors (std::invalid_argument)
    // propagate unchanged.
    TerrainStepOutcome step(std::uint64_t step_ms);

    // The step contract, exposed so orchestration can reject a bad step
    // before the run starts: step_ms >= 1, and speed * dt must not exceed
    // kMaxTerrainStepDisplacementM or arrival_radius_m (a step must not jump
    // over the arrival circle). Throws std::invalid_argument.
    static void validate_step(const TerrainMotionConfig& config, std::uint64_t step_ms);

    [[nodiscard]] const TerrainBodyState& state() const noexcept { return state_; }
    [[nodiscard]] bool mission_complete() const noexcept {
        return state_.next_waypoint == waypoints_.size();
    }
    [[nodiscard]] const TerrainMotionConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<map::Wgs84Coordinate>& waypoints() const noexcept {
        return waypoints_;
    }

private:
    TerrainMotionConfig config_;
    std::vector<map::Wgs84Coordinate> waypoints_;
    std::shared_ptr<const map::ElevationGrid> truth_terrain_;
    TerrainBodyState state_;
};

}  // namespace fleet::world

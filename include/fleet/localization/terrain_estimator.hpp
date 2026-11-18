#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "fleet/localization/pose.hpp"
#include "fleet/localization/terrain_sensors.hpp"
#include "fleet/map/elevation_grid.hpp"
#include "fleet/map/geometry.hpp"

namespace fleet::localization {

// How a body's height measurement relates to the terrain below it (ADR-025).
enum class TerrainHeightReference : std::uint8_t {
    Ground,  // terrain = baro altitude - mount height
    Air,     // terrain surface = baro altitude - rangefinder range
};

// Configuration of the terrain-referenced grid estimator (ADR-025). These are
// the ESTIMATOR's assumptions about its sensors and motion, which need not
// match the simulated sensors' true parameters.
struct TerrainEstimatorConfig {
    // Explicit prior: the robot starts somewhere in a square of half-width
    // half_width_m around prior_center, uniformly likely, heading
    // prior_heading_rad (known at start, e.g. from a compass at power-on).
    map::Wgs84Coordinate prior_center;
    double prior_heading_rad = 0.0;
    double half_width_m = 300.0;  // > 0, <= 2000, a whole number of cells
    double cell_size_m = 10.0;    // > 0; at most 401 cells per side
    // Position uncertainty added per meter travelled (odometry noise and
    // heading drift), as a random walk: sigma = value * sqrt(distance).
    double position_noise_m_per_sqrt_m = 0.5;  // >= 0
    // One height measurement's error: baro and range noise plus map error.
    double height_sigma_m = 2.0;  // > 0
    double baro_bias_sigma_m = 10.0;            // prior bias spread, > 0 (mean 0)
    double baro_bias_walk_m_per_sqrt_s = 0.05;  // >= 0
    TerrainHeightReference reference = TerrainHeightReference::Ground;
    double mount_height_m = 0.0;  // Ground only, >= 0

    // Throws std::invalid_argument naming the offending field.
    void validate() const;
};

// The estimator's belief after one frame (ADR-025): the first uncertainty
// representation in FleetSyncSim. BELIEF only: no truth.
struct TerrainEstimate {
    // Probability-weighted mean position, integrated heading, and the time
    // the estimate refers to (the frame's end).
    LocalizationEstimate estimate;
    map::Wgs84Coordinate most_likely;  // the highest-probability cell
    double spread_m = 0.0;         // radial RMS distance of the probability about the mean
    double effective_cells = 0.0;  // 1 / sum(p^2): 1 = one cell, N = N equal cells
    double baro_bias_m = 0.0;      // probability-weighted mean baro bias estimate
    bool measured = false;         // this frame carried a height measurement
    std::uint64_t resets = 0;      // times all probability was lost and the grid reset
};

// Terrain-referenced point-mass (grid) filter with per-cell barometer-bias
// estimation (ADR-025, M4 #22). Deterministic: no randomness at all.
//
// State: a square grid of cells in a local east/north frame around a moving
// centre. Each cell holds a probability and a scalar Kalman estimate (mean,
// variance) of the barometer bias GIVEN that the robot is in that cell — the
// bias is linear-Gaussian once position is fixed, so it is marginalized per
// cell instead of adding a grid dimension.
//
// Per frame, in this order:
//   1. heading += measured gyro turn;
//   2. the grid centre moves by the measured odometry distance along the new
//      heading (localization::apply_en_displacement): probability moves with
//      the cells, so dead reckoning itself adds no numerical smearing;
//   3. position noise accumulates as variance (value^2 * distance); once it
//      reaches (cell_size / 2)^2 the grid is convolved with a Gaussian of that
//      variance (separable, truncated at 3 sigma, bias moments carried along)
//      and the accumulator resets — diffusion below the cell size is never
//      silently dropped;
//   4. every cell's bias variance grows by walk^2 * dt;
//   5. with a height measurement z (Ground: baro - mount height; Air: baro -
//      range, only on a rangefinder Return), each cell with probability > 0
//      and a map height h is weighted by N(z - h - bias_mean; 0, bias_var +
//      height_sigma^2) and its bias Kalman-updated. A cell where the robot's
//      map has no height (outside coverage or NODATA) gets probability 0:
//      the robot's map defines where it can be;
//   6. probabilities are normalized. If none is left, the estimator is lost:
//      the grid resets to the uniform prior around its current centre with
//      the prior bias, and `resets` counts it;
//   7. when the mean drifts more than a quarter half-width from the centre,
//      the grid re-centres by whole cells toward it (probability leaving the
//      grid is dropped, entering cells are empty);
//   8. the estimate is computed.
//
// Cell positions are east/north offsets from the centre through
// apply_en_displacement, so the grid half-width is bounded (<= 2 km) to keep
// that local approximation accurate.
//
// Heading is NOT estimated: it is integrated from the gyro, so gyro bias
// becomes along-track direction error that position_noise must cover.
//
// Thread-safety: not synchronized (ADR-002).
class TerrainGridEstimator {
public:
    // Throws std::invalid_argument for an invalid config or a null map.
    TerrainGridEstimator(TerrainEstimatorConfig config,
                         std::shared_ptr<const map::ElevationGrid> map);

    // Processes one sensor frame. Throws std::invalid_argument, changing
    // nothing, when the frame lacks odometry, gyro or baro, lacks the
    // rangefinder field in Air mode, or does not start at or after the
    // previous frame's end.
    const TerrainEstimate& update(const TerrainSensorFrame& frame);

    // nullopt before the first frame.
    [[nodiscard]] const std::optional<TerrainEstimate>& latest() const noexcept {
        return latest_;
    }
    [[nodiscard]] std::size_t cells_per_side() const noexcept { return side_; }
    [[nodiscard]] const TerrainEstimatorConfig& config() const noexcept { return config_; }

private:
    void reset_to_prior();
    void diffuse(double variance_m2);
    void measure(double terrain_height_m);
    void recentre_if_needed();
    [[nodiscard]] map::Wgs84Coordinate cell_position(std::size_t east_index,
                                                     std::size_t north_index) const;
    [[nodiscard]] TerrainEstimate summarize(const TerrainSensorFrame& frame, bool measured) const;

    TerrainEstimatorConfig config_;
    std::shared_ptr<const map::ElevationGrid> map_;
    std::size_t side_ = 0;    // cells per side (odd)
    std::size_t centre_ = 0;  // index of the centre cell on each axis
    map::Wgs84Coordinate centre_position_;
    double heading_rad_ = 0.0;
    double pending_variance_m2_ = 0.0;
    std::uint64_t resets_ = 0;
    std::vector<double> probability_;  // row-major: north index * side + east index
    std::vector<double> bias_mean_m_;
    std::vector<double> bias_variance_m2_;
    std::optional<TerrainEstimate> latest_;
};

}  // namespace fleet::localization

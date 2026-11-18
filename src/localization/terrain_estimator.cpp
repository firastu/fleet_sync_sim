#include "fleet/localization/terrain_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

namespace fleet::localization {

namespace {

constexpr double kMaxHalfWidthM = 2000.0;
constexpr std::size_t kMaxCellsPerSide = 401;

void require_finite(double value, const char* field) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::format("terrain estimator: {} must be finite", field));
    }
}

void require_positive(double value, const char* field) {
    require_finite(value, field);
    if (!(value > 0.0)) {
        throw std::invalid_argument(std::format("terrain estimator: {} must be > 0", field));
    }
}

void require_non_negative(double value, const char* field) {
    require_finite(value, field);
    if (value < 0.0) {
        throw std::invalid_argument(std::format("terrain estimator: {} must be >= 0", field));
    }
}

// Cells from the centre to the edge; validate() guarantees a whole number.
[[nodiscard]] std::size_t half_cells(const TerrainEstimatorConfig& config) {
    return static_cast<std::size_t>(std::llround(config.half_width_m / config.cell_size_m));
}

// One separable pass of the convolution along one axis. `along_east` selects
// the axis; contributions from outside the grid are absent (mass that would
// come from or go past the edge is dropped).
void convolve_axis(const std::vector<double>& input, std::vector<double>& output,
                   std::size_t side, const std::vector<double>& kernel, bool along_east) {
    const auto radius = static_cast<std::ptrdiff_t>(kernel.size() / 2);
    const auto extent = static_cast<std::ptrdiff_t>(side);
    for (std::ptrdiff_t north = 0; north < extent; ++north) {
        for (std::ptrdiff_t east = 0; east < extent; ++east) {
            double sum = 0.0;
            for (std::ptrdiff_t offset = -radius; offset <= radius; ++offset) {
                const std::ptrdiff_t source_east = along_east ? east + offset : east;
                const std::ptrdiff_t source_north = along_east ? north : north + offset;
                if (source_east < 0 || source_east >= extent || source_north < 0 ||
                    source_north >= extent) {
                    continue;
                }
                sum += kernel[static_cast<std::size_t>(offset + radius)] *
                       input[static_cast<std::size_t>(source_north * extent + source_east)];
            }
            output[static_cast<std::size_t>(north * extent + east)] = sum;
        }
    }
}

}  // namespace

void TerrainEstimatorConfig::validate() const {
    require_finite(prior_center.latitude_deg, "prior latitude");
    require_finite(prior_center.longitude_deg, "prior longitude");
    if (std::abs(prior_center.latitude_deg) > 90.0 ||
        std::abs(prior_center.longitude_deg) > 180.0) {
        throw std::invalid_argument("terrain estimator: prior centre must be within WGS84 bounds");
    }
    require_finite(prior_heading_rad, "prior heading");
    require_positive(half_width_m, "half_width_m");
    require_positive(cell_size_m, "cell_size_m");
    if (half_width_m > kMaxHalfWidthM) {
        throw std::invalid_argument(std::format(
            "terrain estimator: half_width_m must not exceed {:.0f} m (local-tangent cell "
            "offsets)",
            kMaxHalfWidthM));
    }
    const double cells = half_width_m / cell_size_m;
    if (std::abs(cells - std::round(cells)) > 1e-9 * cells || std::round(cells) < 1.0) {
        throw std::invalid_argument(
            "terrain estimator: half_width_m must be a whole, non-zero number of cells");
    }
    if (2 * half_cells(*this) + 1 > kMaxCellsPerSide) {
        throw std::invalid_argument(std::format(
            "terrain estimator: at most {} cells per side (2 * half_width / cell_size + 1)",
            kMaxCellsPerSide));
    }
    require_non_negative(position_noise_m_per_sqrt_m, "position_noise_m_per_sqrt_m");
    require_positive(height_sigma_m, "height_sigma_m");
    require_positive(baro_bias_sigma_m, "baro_bias_sigma_m");
    require_non_negative(baro_bias_walk_m_per_sqrt_s, "baro_bias_walk_m_per_sqrt_s");
    require_non_negative(mount_height_m, "mount_height_m");
    if (reference == TerrainHeightReference::Air && mount_height_m != 0.0) {
        throw std::invalid_argument(
            "terrain estimator: mount_height_m applies to the Ground reference only");
    }
}

TerrainGridEstimator::TerrainGridEstimator(TerrainEstimatorConfig config,
                                           std::shared_ptr<const map::ElevationGrid> map)
    : config_{std::move(config)}, map_{std::move(map)} {
    config_.validate();
    if (map_ == nullptr) {
        throw std::invalid_argument("terrain estimator: a terrain map is required");
    }
    centre_ = half_cells(config_);
    side_ = 2 * centre_ + 1;
    centre_position_ = config_.prior_center;
    heading_rad_ = normalize_heading(config_.prior_heading_rad);
    probability_.assign(side_ * side_, 0.0);
    bias_mean_m_.assign(side_ * side_, 0.0);
    bias_variance_m2_.assign(side_ * side_, 0.0);
    reset_to_prior();
}

void TerrainGridEstimator::reset_to_prior() {
    const double uniform = 1.0 / static_cast<double>(side_ * side_);
    std::fill(probability_.begin(), probability_.end(), uniform);
    std::fill(bias_mean_m_.begin(), bias_mean_m_.end(), 0.0);
    std::fill(bias_variance_m2_.begin(), bias_variance_m2_.end(),
              config_.baro_bias_sigma_m * config_.baro_bias_sigma_m);
    pending_variance_m2_ = 0.0;
}

map::Wgs84Coordinate TerrainGridEstimator::cell_position(std::size_t east_index,
                                                         std::size_t north_index) const {
    const double east_m =
        (static_cast<double>(east_index) - static_cast<double>(centre_)) * config_.cell_size_m;
    const double north_m =
        (static_cast<double>(north_index) - static_cast<double>(centre_)) * config_.cell_size_m;
    return apply_en_displacement(centre_position_, east_m, north_m);
}

void TerrainGridEstimator::diffuse(double variance_m2) {
    const double sigma_cells = std::sqrt(variance_m2) / config_.cell_size_m;
    const auto radius = static_cast<std::size_t>(std::ceil(3.0 * sigma_cells));
    std::vector<double> kernel(2 * radius + 1);
    double kernel_sum = 0.0;
    for (std::size_t index = 0; index < kernel.size(); ++index) {
        const double offset = static_cast<double>(index) - static_cast<double>(radius);
        kernel[index] = std::exp(-(offset * offset) / (2.0 * sigma_cells * sigma_cells));
        kernel_sum += kernel[index];
    }
    for (double& weight : kernel) {
        weight /= kernel_sum;
    }

    // Moment matching: convolving probability, probability * bias mean and
    // probability * (bias variance + mean^2) with the same kernel yields the
    // exact mixture moments of each destination cell's bias.
    const std::size_t cells = side_ * side_;
    std::vector<double> first_moment(cells);
    std::vector<double> second_moment(cells);
    for (std::size_t index = 0; index < cells; ++index) {
        first_moment[index] = probability_[index] * bias_mean_m_[index];
        second_moment[index] = probability_[index] * (bias_variance_m2_[index] +
                                                      bias_mean_m_[index] * bias_mean_m_[index]);
    }
    std::vector<double> scratch(cells);
    for (std::vector<double>* field : {&probability_, &first_moment, &second_moment}) {
        convolve_axis(*field, scratch, side_, kernel, true);
        convolve_axis(scratch, *field, side_, kernel, false);
    }
    for (std::size_t index = 0; index < cells; ++index) {
        if (probability_[index] > 0.0) {
            bias_mean_m_[index] = first_moment[index] / probability_[index];
            bias_variance_m2_[index] = std::max(
                0.0, second_moment[index] / probability_[index] -
                         bias_mean_m_[index] * bias_mean_m_[index]);
        }
    }
}

void TerrainGridEstimator::measure(double terrain_height_m) {
    const double measurement_variance = config_.height_sigma_m * config_.height_sigma_m;
    for (std::size_t north = 0; north < side_; ++north) {
        for (std::size_t east = 0; east < side_; ++east) {
            const std::size_t index = north * side_ + east;
            if (probability_[index] <= 0.0) {
                continue;
            }
            const map::ElevationLookup lookup = map_->height_at(cell_position(east, north));
            if (lookup.status != map::ElevationStatus::Ok) {
                probability_[index] = 0.0;  // the robot's map says it cannot be here
                continue;
            }
            const double innovation = terrain_height_m - *lookup.height_m - bias_mean_m_[index];
            const double innovation_variance = bias_variance_m2_[index] + measurement_variance;
            probability_[index] *=
                std::exp(-(innovation * innovation) / (2.0 * innovation_variance)) /
                std::sqrt(innovation_variance);
            const double gain = bias_variance_m2_[index] / innovation_variance;
            bias_mean_m_[index] += gain * innovation;
            bias_variance_m2_[index] -= gain * bias_variance_m2_[index];
        }
    }
}

void TerrainGridEstimator::recentre_if_needed() {
    double mean_east = 0.0;
    double mean_north = 0.0;
    for (std::size_t north = 0; north < side_; ++north) {
        for (std::size_t east = 0; east < side_; ++east) {
            const double probability = probability_[north * side_ + east];
            mean_east += probability * (static_cast<double>(east) - static_cast<double>(centre_));
            mean_north +=
                probability * (static_cast<double>(north) - static_cast<double>(centre_));
        }
    }
    const double limit_cells = static_cast<double>(centre_) / 4.0;
    if (std::abs(mean_east) <= limit_cells && std::abs(mean_north) <= limit_cells) {
        return;
    }
    const long long shift_east = std::llround(mean_east);
    const long long shift_north = std::llround(mean_north);

    const std::size_t cells = side_ * side_;
    std::vector<double> probability(cells, 0.0);
    std::vector<double> bias_mean(cells, 0.0);
    std::vector<double> bias_variance(cells,
                                      config_.baro_bias_sigma_m * config_.baro_bias_sigma_m);
    const auto extent = static_cast<long long>(side_);
    for (long long north = 0; north < extent; ++north) {
        for (long long east = 0; east < extent; ++east) {
            const long long source_east = east + shift_east;
            const long long source_north = north + shift_north;
            if (source_east < 0 || source_east >= extent || source_north < 0 ||
                source_north >= extent) {
                continue;  // entering cells start empty
            }
            const auto target = static_cast<std::size_t>(north * extent + east);
            const auto source = static_cast<std::size_t>(source_north * extent + source_east);
            probability[target] = probability_[source];
            bias_mean[target] = bias_mean_m_[source];
            bias_variance[target] = bias_variance_m2_[source];
        }
    }
    probability_ = std::move(probability);
    bias_mean_m_ = std::move(bias_mean);
    bias_variance_m2_ = std::move(bias_variance);
    centre_position_ = apply_en_displacement(
        centre_position_, static_cast<double>(shift_east) * config_.cell_size_m,
        static_cast<double>(shift_north) * config_.cell_size_m);

    double total = 0.0;
    for (const double probability_value : probability_) {
        total += probability_value;
    }
    if (total > 0.0) {
        for (double& probability_value : probability_) {
            probability_value /= total;
        }
    } else {
        reset_to_prior();
        ++resets_;
    }
}

TerrainEstimate TerrainGridEstimator::summarize(const TerrainSensorFrame& frame,
                                                bool measured) const {
    double mean_east = 0.0;
    double mean_north = 0.0;
    double second = 0.0;
    double sum_squares = 0.0;
    double bias = 0.0;
    std::size_t best = 0;
    for (std::size_t north = 0; north < side_; ++north) {
        for (std::size_t east = 0; east < side_; ++east) {
            const std::size_t index = north * side_ + east;
            const double probability = probability_[index];
            const double east_m =
                (static_cast<double>(east) - static_cast<double>(centre_)) * config_.cell_size_m;
            const double north_m =
                (static_cast<double>(north) - static_cast<double>(centre_)) * config_.cell_size_m;
            mean_east += probability * east_m;
            mean_north += probability * north_m;
            second += probability * (east_m * east_m + north_m * north_m);
            sum_squares += probability * probability;
            bias += probability * bias_mean_m_[index];
            if (probability > probability_[best]) {
                best = index;  // first maximum in index order wins ties
            }
        }
    }
    TerrainEstimate result;
    result.estimate = LocalizationEstimate{
        apply_en_displacement(centre_position_, mean_east, mean_north), heading_rad_, frame.at};
    result.most_likely = cell_position(best % side_, best / side_);
    result.spread_m =
        std::sqrt(std::max(0.0, second - mean_east * mean_east - mean_north * mean_north));
    result.effective_cells = sum_squares > 0.0 ? 1.0 / sum_squares : 0.0;
    result.baro_bias_m = bias;
    result.measured = measured;
    result.resets = resets_;
    return result;
}

const TerrainEstimate& TerrainGridEstimator::update(const TerrainSensorFrame& frame) {
    // Validate everything before any change.
    if (!(frame.at > frame.from)) {
        throw std::invalid_argument("terrain estimator: a frame must end after it starts");
    }
    if (latest_ && frame.from < latest_->estimate.estimated_at) {
        throw std::invalid_argument(
            "terrain estimator: a frame must not start before the previous one ended");
    }
    if (!frame.odometry_distance_m || !frame.gyro_turn_rad || !frame.baro_altitude_m) {
        throw std::invalid_argument(
            "terrain estimator: frames need odometry, gyro and baro measurements");
    }
    if (config_.reference == TerrainHeightReference::Air && !frame.range) {
        throw std::invalid_argument(
            "terrain estimator: the Air reference needs a rangefinder reading");
    }
    const double distance = *frame.odometry_distance_m;
    require_finite(distance, "odometry distance");
    require_finite(*frame.gyro_turn_rad, "gyro turn");
    require_finite(*frame.baro_altitude_m, "baro altitude");

    // 1-2. Dead reckoning moves the grid (computed first, committed together).
    const double heading = normalize_heading(heading_rad_ + *frame.gyro_turn_rad);
    const map::Wgs84Coordinate centre = apply_en_displacement(
        centre_position_, distance * std::sin(heading), distance * std::cos(heading));
    heading_rad_ = heading;
    centre_position_ = centre;

    // 3. Position noise, applied once it reaches half a cell.
    pending_variance_m2_ += config_.position_noise_m_per_sqrt_m *
                            config_.position_noise_m_per_sqrt_m * std::abs(distance);
    const double half_cell = config_.cell_size_m / 2.0;
    if (pending_variance_m2_ >= half_cell * half_cell) {
        diffuse(pending_variance_m2_);
        pending_variance_m2_ = 0.0;
    }

    // 4. Bias random walk.
    const double dt_s = static_cast<double>(frame.at.value - frame.from.value) / 1000.0;
    const double walk_variance =
        config_.baro_bias_walk_m_per_sqrt_s * config_.baro_bias_walk_m_per_sqrt_s * dt_s;
    for (double& variance : bias_variance_m2_) {
        variance += walk_variance;
    }

    // 5. Height measurement.
    std::optional<double> terrain_height;
    if (config_.reference == TerrainHeightReference::Ground) {
        terrain_height = *frame.baro_altitude_m - config_.mount_height_m;
    } else if (frame.range->status == RangeStatus::Return) {
        terrain_height = *frame.baro_altitude_m - *frame.range->range_m;
    }
    if (terrain_height) {
        measure(*terrain_height);
    }

    // 6. Normalize, or reset when every hypothesis is gone.
    double total = 0.0;
    for (const double probability : probability_) {
        total += probability;
    }
    if (total > 0.0 && std::isfinite(total)) {
        for (double& probability : probability_) {
            probability /= total;
        }
    } else {
        reset_to_prior();
        ++resets_;
    }

    // 7-8.
    recentre_if_needed();
    latest_ = summarize(frame, terrain_height.has_value());
    return *latest_;
}

}  // namespace fleet::localization

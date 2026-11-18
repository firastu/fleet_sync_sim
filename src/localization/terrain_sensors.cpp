#include "fleet/localization/terrain_sensors.hpp"

#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

#include "fleet/localization/noise.hpp"

namespace fleet::localization {

namespace {

constexpr std::uint32_t kPartsPerMillion = 1'000'000;

void require_finite(double value, const char* field) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::format("terrain sensors: {} must be finite", field));
    }
}

void require_non_negative(double value, const char* field) {
    require_finite(value, field);
    if (value < 0.0) {
        throw std::invalid_argument(std::format("terrain sensors: {} must be >= 0", field));
    }
}

}  // namespace

void TerrainSensorConfig::validate() const {
    if (odometry) {
        require_finite(odometry->scale_error, "odometry scale_error");
        if (!(odometry->scale_error > -1.0)) {
            throw std::invalid_argument("terrain sensors: odometry scale_error must be > -1");
        }
        require_non_negative(odometry->noise_m_per_sqrt_m, "odometry noise_m_per_sqrt_m");
    }
    if (gyro) {
        require_finite(gyro->initial_bias_rad_per_s, "gyro initial bias");
        require_non_negative(gyro->bias_walk_rad_per_s_per_sqrt_s, "gyro bias walk");
        require_non_negative(gyro->angle_random_walk_rad_per_sqrt_s, "gyro angle random walk");
    }
    if (baro) {
        require_finite(baro->initial_bias_m, "baro initial_bias_m");
        require_non_negative(baro->bias_walk_m_per_sqrt_s, "baro bias walk");
        require_non_negative(baro->noise_sigma_m, "baro noise_sigma_m");
    }
    if (rangefinder) {
        require_finite(rangefinder->max_range_m, "rangefinder max_range_m");
        if (!(rangefinder->max_range_m > 0.0)) {
            throw std::invalid_argument("terrain sensors: rangefinder max_range_m must be > 0");
        }
        require_non_negative(rangefinder->noise_sigma_m, "rangefinder noise_sigma_m");
        if (rangefinder->dropout_ppm > kPartsPerMillion) {
            throw std::invalid_argument(
                "terrain sensors: rangefinder dropout_ppm must be within [0, 1000000]");
        }
    }
}

TerrainSensors::TerrainSensors(TerrainSensorConfig config, TerrainSensorSeeds seeds)
    : config_{std::move(config)},
      odometry_rng_{seeds.odometry},
      gyro_rng_{seeds.gyro},
      baro_rng_{seeds.baro},
      rangefinder_rng_{seeds.rangefinder} {
    config_.validate();
    if (config_.gyro) {
        gyro_bias_rad_per_s_ = config_.gyro->initial_bias_rad_per_s;
    }
    if (config_.baro) {
        baro_bias_m_ = config_.baro->initial_bias_m;
    }
}

TerrainSensorFrame TerrainSensors::measure(const TerrainMotionTruth& truth) {
    // Validate everything before any draw or state change.
    if (!(truth.at > truth.from)) {
        throw std::invalid_argument("terrain sensors: the interval must end after it starts");
    }
    require_finite(truth.distance_m, "truth distance_m");
    require_finite(truth.turn_rad, "truth turn_rad");
    require_finite(truth.altitude_m, "truth altitude_m");
    require_finite(truth.height_above_surface_m, "truth height_above_surface_m");
    if (truth.distance_m < 0.0) {
        throw std::invalid_argument("terrain sensors: truth distance_m must be >= 0");
    }
    if (config_.rangefinder && truth.height_above_surface_m < 0.0) {
        throw std::invalid_argument(
            "terrain sensors: truth height_above_surface_m must be >= 0 for a rangefinder");
    }

    const double dt_s = static_cast<double>(truth.at.value - truth.from.value) / 1000.0;
    const double sqrt_dt = std::sqrt(dt_s);
    TerrainSensorFrame frame{truth.from, truth.at, std::nullopt, std::nullopt, std::nullopt,
                             std::nullopt};

    if (config_.odometry) {
        const OdometrySensorConfig& odometry = *config_.odometry;
        double measured = truth.distance_m * (1.0 + odometry.scale_error);
        if (odometry.noise_m_per_sqrt_m > 0.0) {
            measured += odometry.noise_m_per_sqrt_m * std::sqrt(truth.distance_m) *
                        irwin_hall_standard_noise(odometry_rng_);
        }
        frame.odometry_distance_m = measured;
    }

    if (config_.gyro) {
        const GyroSensorConfig& gyro = *config_.gyro;
        if (gyro.bias_walk_rad_per_s_per_sqrt_s > 0.0) {
            gyro_bias_rad_per_s_ += gyro.bias_walk_rad_per_s_per_sqrt_s * sqrt_dt *
                                    irwin_hall_standard_noise(gyro_rng_);
        }
        double measured = truth.turn_rad + gyro_bias_rad_per_s_ * dt_s;
        if (gyro.angle_random_walk_rad_per_sqrt_s > 0.0) {
            measured += gyro.angle_random_walk_rad_per_sqrt_s * sqrt_dt *
                        irwin_hall_standard_noise(gyro_rng_);
        }
        frame.gyro_turn_rad = measured;
    }

    if (config_.baro) {
        const BaroSensorConfig& baro = *config_.baro;
        if (baro.bias_walk_m_per_sqrt_s > 0.0) {
            baro_bias_m_ +=
                baro.bias_walk_m_per_sqrt_s * sqrt_dt * irwin_hall_standard_noise(baro_rng_);
        }
        double measured = truth.altitude_m + baro_bias_m_;
        if (baro.noise_sigma_m > 0.0) {
            measured += baro.noise_sigma_m * irwin_hall_standard_noise(baro_rng_);
        }
        frame.baro_altitude_m = measured;
    }

    if (config_.rangefinder) {
        const RangefinderSensorConfig& rangefinder = *config_.rangefinder;
        // Draws first, unconditionally: consumption depends only on the
        // configuration (contract above).
        bool dropped = rangefinder.dropout_ppm == kPartsPerMillion;
        if (rangefinder.dropout_ppm > 0 && rangefinder.dropout_ppm < kPartsPerMillion) {
            dropped = rangefinder_rng_.uniform_below(kPartsPerMillion) < rangefinder.dropout_ppm;
        }
        double noise = 0.0;
        if (rangefinder.noise_sigma_m > 0.0) {
            noise = rangefinder.noise_sigma_m * irwin_hall_standard_noise(rangefinder_rng_);
        }
        if (truth.height_above_surface_m > rangefinder.max_range_m) {
            frame.range = RangeReading{RangeStatus::OutOfRange, std::nullopt};
        } else if (dropped) {
            frame.range = RangeReading{RangeStatus::Dropout, std::nullopt};
        } else {
            frame.range = RangeReading{RangeStatus::Return, truth.height_above_surface_m + noise};
        }
    }
    return frame;
}

}  // namespace fleet::localization

#pragma once

#include <cstdint>
#include <optional>

#include "fleet/common/time.hpp"
#include "fleet/simulation/deterministic_rng.hpp"

namespace fleet::localization {

// What the sensors of one terrain-domain body physically experienced over
// one interval (ADR-024): SIMULATION TRUTH, like GroundTruthPose. It crosses
// only into TerrainSensors; robots never see it. The orchestration fills it
// from the truth motion step.
struct TerrainMotionTruth {
    common::Tick from{};               // interval start
    common::Tick at{};                 // interval end, strictly after `from`
    double distance_m = 0.0;           // horizontal distance travelled, >= 0
    double turn_rad = 0.0;             // signed heading change, clockwise positive
    double altitude_m = 0.0;           // body altitude at `at`, terrain datum
    double height_above_surface_m = 0.0;  // altitude minus truth terrain at `at`
};

// Relative distance (wheel odometry, visual odometry or optical flow; slip
// and image texture are not modelled):
//   measured = d * (1 + scale_error) + noise_m_per_sqrt_m * sqrt(d) * N
// Noise grows with the square root of distance, so the error variance adds
// up per meter and does not depend on the step length.
struct OdometrySensorConfig {
    double scale_error = 0.0;          // finite, > -1
    double noise_m_per_sqrt_m = 0.0;   // finite, >= 0
};

// Heading-change gyro with a random-walking bias:
//   bias     += bias_walk_rad_per_s_per_sqrt_s * sqrt(dt) * N
//   measured  = turn + bias * dt + angle_random_walk_rad_per_sqrt_s * sqrt(dt) * N
struct GyroSensorConfig {
    double initial_bias_rad_per_s = 0.0;             // finite
    double bias_walk_rad_per_s_per_sqrt_s = 0.0;     // finite, >= 0
    double angle_random_walk_rad_per_sqrt_s = 0.0;   // finite, >= 0
};

// Barometric altimeter with a drifting bias (weather and temperature):
//   bias     += bias_walk_m_per_sqrt_s * sqrt(dt) * N
//   measured  = altitude + bias + noise_sigma_m * N
struct BaroSensorConfig {
    double initial_bias_m = 0.0;          // finite
    double bias_walk_m_per_sqrt_s = 0.0;  // finite, >= 0
    double noise_sigma_m = 0.0;           // finite, >= 0
};

// Downward rangefinder (radar or laser altimeter) for air platforms: the
// distance from the body to the truth terrain surface below.
struct RangefinderSensorConfig {
    double max_range_m = 1.0;     // finite, > 0
    double noise_sigma_m = 0.0;   // finite, >= 0
    std::uint32_t dropout_ppm = 0;  // probability of no return, [0, 1000000]
};

// Which sensors a body carries; an absent entry is an absent sensor.
struct TerrainSensorConfig {
    std::optional<OdometrySensorConfig> odometry;
    std::optional<GyroSensorConfig> gyro;
    std::optional<BaroSensorConfig> baro;
    std::optional<RangefinderSensorConfig> rangefinder;

    [[nodiscard]] bool any() const noexcept {
        return odometry || gyro || baro || rangefinder;
    }

    // Throws std::invalid_argument naming the offending field.
    void validate() const;
};

enum class RangeStatus : std::uint8_t {
    Return,      // a range was measured
    OutOfRange,  // the true range exceeds max_range_m
    Dropout,     // the sensor returned nothing this time
};

struct RangeReading {
    RangeStatus status = RangeStatus::Dropout;
    std::optional<double> range_m;  // engaged exactly when status == Return

    bool operator==(const RangeReading&) const = default;
};

// What a robot receives from its sensors for one interval: MEASUREMENTS
// only (ADR-024). A field is engaged exactly when that sensor is configured.
struct TerrainSensorFrame {
    common::Tick from{};
    common::Tick at{};
    std::optional<double> odometry_distance_m;
    std::optional<double> gyro_turn_rad;
    std::optional<double> baro_altitude_m;
    std::optional<RangeReading> range;

    bool operator==(const TerrainSensorFrame&) const = default;
};

// One RNG seed per sensor kind (ADR-024): each sensor draws from its own
// stream, so configuring or reconfiguring one sensor never shifts another
// sensor's noise sequence.
struct TerrainSensorSeeds {
    std::uint64_t odometry = 0;
    std::uint64_t gyro = 0;
    std::uint64_t baro = 0;
    std::uint64_t rangefinder = 0;
};

// The deterministic sensor suite of one terrain-domain body (ADR-024, M4 #21).
//
// measure() turns one TerrainMotionTruth into one TerrainSensorFrame. Noise
// uses irwin_hall_standard_noise (ADR-017) on the sensor's own stream.
//
// RNG-consumption contract (fixed; depends only on the configuration, never
// on the truth values): per frame, on each configured sensor's own stream,
//   odometry:    12 draws if noise_m_per_sqrt_m > 0;
//   gyro:        12 draws if bias_walk > 0, then 12 if angle_random_walk > 0;
//   baro:        12 draws if bias_walk > 0, then 12 if noise_sigma > 0;
//   rangefinder: 1 uniform_below(1000000) draw if 0 < dropout_ppm < 1000000,
//                then 12 draws if noise_sigma > 0 — both even when the
//                reading is out of range or dropped.
// A zero-noise sensor consumes nothing and measures exactly
// (odometry still applies its scale error, gyro and baro their initial
// bias). The rangefinder reports OutOfRange before Dropout. Measured ranges
// and distances are not clamped: noise can make them slightly negative.
//
// Thread-safety: not synchronized (ADR-002).
class TerrainSensors {
public:
    // Throws std::invalid_argument for an invalid configuration.
    TerrainSensors(TerrainSensorConfig config, TerrainSensorSeeds seeds);

    // Throws std::invalid_argument when at <= from, a truth value is not
    // finite, distance_m is negative, or (with a rangefinder)
    // height_above_surface_m is negative. Nothing is consumed or changed on
    // throw.
    [[nodiscard]] TerrainSensorFrame measure(const TerrainMotionTruth& truth);

    [[nodiscard]] const TerrainSensorConfig& config() const noexcept { return config_; }
    // Current random-walk biases (simulation-side diagnostics only).
    [[nodiscard]] double gyro_bias_rad_per_s() const noexcept { return gyro_bias_rad_per_s_; }
    [[nodiscard]] double baro_bias_m() const noexcept { return baro_bias_m_; }

private:
    TerrainSensorConfig config_;
    simulation::DeterministicRng odometry_rng_;
    simulation::DeterministicRng gyro_rng_;
    simulation::DeterministicRng baro_rng_;
    simulation::DeterministicRng rangefinder_rng_;
    double gyro_bias_rad_per_s_ = 0.0;
    double baro_bias_m_ = 0.0;
};

}  // namespace fleet::localization

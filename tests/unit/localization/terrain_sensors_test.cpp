#include "fleet/localization/terrain_sensors.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

#include "fleet/localization/noise.hpp"
#include "fleet/simulation/deterministic_rng.hpp"

namespace {

using fleet::common::Tick;
using fleet::localization::BaroSensorConfig;
using fleet::localization::GyroSensorConfig;
using fleet::localization::irwin_hall_standard_noise;
using fleet::localization::kIrwinHallDraws;
using fleet::localization::OdometrySensorConfig;
using fleet::localization::RangefinderSensorConfig;
using fleet::localization::RangeReading;
using fleet::localization::RangeStatus;
using fleet::localization::TerrainMotionTruth;
using fleet::localization::TerrainSensorConfig;
using fleet::localization::TerrainSensorFrame;
using fleet::localization::TerrainSensors;
using fleet::localization::TerrainSensorSeeds;
using fleet::simulation::DeterministicRng;

constexpr TerrainSensorSeeds kSeeds{11, 22, 33, 44};

// One 2-second interval: 8 m travelled, 0.1 rad clockwise turn, 900 m altitude,
// 250 m above the surface.
constexpr TerrainMotionTruth kTruth{Tick{1000}, Tick{3000}, 8.0, 0.1, 900.0, 250.0};
constexpr double kDt = 2.0;

[[nodiscard]] TerrainMotionTruth next_interval(const TerrainMotionTruth& truth) {
    TerrainMotionTruth next = truth;
    next.from = truth.at;
    next.at = truth.at + 2000;
    return next;
}

TEST(IrwinHallNoiseTest, ConsumesExactlyTwelveDrawsWithTheDocumentedArithmetic) {
    DeterministicRng rng{5};
    DeterministicRng control{5};
    double expected = 0.0;
    for (std::uint64_t i = 0; i < kIrwinHallDraws; ++i) {
        expected += (static_cast<double>(control.uniform_below(1ULL << 32)) + 0.5) /
                        4294967296.0 -
                    0.5;
    }
    EXPECT_EQ(irwin_hall_standard_noise(rng), expected);
    EXPECT_EQ(rng.uniform_below(1000), control.uniform_below(1000));  // streams aligned
}

TEST(TerrainSensorsTest, ZeroNoiseSensorsMeasureExactlyAndConsumeNothing) {
    TerrainSensorConfig config;
    config.odometry = OdometrySensorConfig{0.05, 0.0};
    config.gyro = GyroSensorConfig{0.01, 0.0, 0.0};
    config.baro = BaroSensorConfig{3.0, 0.0, 0.0};
    config.rangefinder = RangefinderSensorConfig{1000.0, 0.0, 0};
    TerrainSensors sensors{config, kSeeds};
    const TerrainSensorFrame frame = sensors.measure(kTruth);
    EXPECT_EQ(frame.from, kTruth.from);
    EXPECT_EQ(frame.at, kTruth.at);
    EXPECT_EQ(frame.odometry_distance_m, 8.0 * (1.0 + 0.05));  // scale error only
    EXPECT_EQ(frame.gyro_turn_rad, 0.1 + 0.01 * kDt);       // initial bias only
    EXPECT_EQ(frame.baro_altitude_m, 903.0);                // initial bias only
    EXPECT_EQ(frame.range, (RangeReading{RangeStatus::Return, 250.0}));
}

TEST(TerrainSensorsTest, UnconfiguredSensorsLeaveTheirFieldsEmpty) {
    TerrainSensorConfig config;
    config.baro = BaroSensorConfig{};
    TerrainSensors sensors{config, kSeeds};
    const TerrainSensorFrame frame = sensors.measure(kTruth);
    EXPECT_FALSE(frame.odometry_distance_m.has_value());
    EXPECT_FALSE(frame.gyro_turn_rad.has_value());
    EXPECT_EQ(frame.baro_altitude_m, 900.0);
    EXPECT_FALSE(frame.range.has_value());
}

TEST(TerrainSensorsTest, OdometryNoiseGrowsWithTheSquareRootOfDistance) {
    TerrainSensorConfig config;
    config.odometry = OdometrySensorConfig{0.02, 0.1};
    TerrainSensors sensors{config, kSeeds};
    DeterministicRng reference{kSeeds.odometry};
    const double first =
        8.0 * (1.0 + 0.02) + 0.1 * std::sqrt(8.0) * irwin_hall_standard_noise(reference);
    EXPECT_EQ(sensors.measure(kTruth).odometry_distance_m, first);

    // Parked: no distance, no error, but the draw is still consumed.
    TerrainMotionTruth parked = next_interval(kTruth);
    parked.distance_m = 0.0;
    (void)irwin_hall_standard_noise(reference);
    EXPECT_EQ(sensors.measure(parked).odometry_distance_m, 0.0);
    TerrainMotionTruth moving = next_interval(parked);
    moving.distance_m = 8.0;
    EXPECT_EQ(sensors.measure(moving).odometry_distance_m,
              8.0 * (1.0 + 0.02) + 0.1 * std::sqrt(8.0) * irwin_hall_standard_noise(reference));
}

TEST(TerrainSensorsTest, GyroBiasWalksBeforeTheTurnIsMeasured) {
    TerrainSensorConfig config;
    config.gyro = GyroSensorConfig{0.01, 0.002, 0.03};
    TerrainSensors sensors{config, kSeeds};
    DeterministicRng reference{kSeeds.gyro};
    double bias = 0.01;
    TerrainMotionTruth truth = kTruth;
    for (int frame = 0; frame < 5; ++frame) {
        bias += 0.002 * std::sqrt(kDt) * irwin_hall_standard_noise(reference);
        const double expected =
            0.1 + bias * kDt + 0.03 * std::sqrt(kDt) * irwin_hall_standard_noise(reference);
        EXPECT_EQ(sensors.measure(truth).gyro_turn_rad, expected) << "frame " << frame;
        EXPECT_EQ(sensors.gyro_bias_rad_per_s(), bias);
        truth = next_interval(truth);
    }
}

TEST(TerrainSensorsTest, BaroBiasDriftsAndNoiseIsWhite) {
    TerrainSensorConfig config;
    config.baro = BaroSensorConfig{-2.0, 0.05, 0.5};
    TerrainSensors sensors{config, kSeeds};
    DeterministicRng reference{kSeeds.baro};
    double bias = -2.0;
    TerrainMotionTruth truth = kTruth;
    for (int frame = 0; frame < 5; ++frame) {
        bias += 0.05 * std::sqrt(kDt) * irwin_hall_standard_noise(reference);
        const double expected = 900.0 + bias + 0.5 * irwin_hall_standard_noise(reference);
        EXPECT_EQ(sensors.measure(truth).baro_altitude_m, expected) << "frame " << frame;
        EXPECT_EQ(sensors.baro_bias_m(), bias);
        truth = next_interval(truth);
    }
}

TEST(TerrainSensorsTest, RangefinderReportsOutOfRangeAndDropoutsWithFixedConsumption) {
    TerrainSensorConfig config;
    config.rangefinder = RangefinderSensorConfig{300.0, 1.0, 500'000};
    TerrainSensors sensors{config, kSeeds};
    DeterministicRng reference{kSeeds.rangefinder};
    TerrainMotionTruth truth = kTruth;
    for (int frame = 0; frame < 20; ++frame) {
        truth.height_above_surface_m = frame % 4 == 0 ? 350.0 : 250.0;  // some out of range
        const bool dropped = reference.uniform_below(1'000'000) < 500'000;
        const double noise = 1.0 * irwin_hall_standard_noise(reference);
        const RangeReading expected =
            truth.height_above_surface_m > 300.0 ? RangeReading{RangeStatus::OutOfRange, {}}
            : dropped                            ? RangeReading{RangeStatus::Dropout, {}}
                                                 : RangeReading{RangeStatus::Return, 250.0 + noise};
        EXPECT_EQ(sensors.measure(truth).range, expected) << "frame " << frame;
        truth = next_interval(truth);
    }

    // The two extreme dropout settings draw nothing for the dropout decision.
    config.rangefinder = RangefinderSensorConfig{1000.0, 0.0, 1'000'000};
    EXPECT_EQ(TerrainSensors(config, kSeeds).measure(kTruth).range,
              (RangeReading{RangeStatus::Dropout, std::nullopt}));
    config.rangefinder = RangefinderSensorConfig{1000.0, 0.0, 0};
    EXPECT_EQ(TerrainSensors(config, kSeeds).measure(kTruth).range,
              (RangeReading{RangeStatus::Return, 250.0}));
}

TEST(TerrainSensorsTest, EachSensorHasItsOwnStream) {
    TerrainSensorConfig odometry_only;
    odometry_only.odometry = OdometrySensorConfig{0.0, 0.2};
    TerrainSensorConfig everything = odometry_only;
    everything.gyro = GyroSensorConfig{0.0, 0.01, 0.01};
    everything.baro = BaroSensorConfig{0.0, 0.1, 0.1};
    everything.rangefinder = RangefinderSensorConfig{1000.0, 1.0, 100'000};
    TerrainSensors alone{odometry_only, kSeeds};
    TerrainSensors together{everything, kSeeds};
    TerrainMotionTruth truth = kTruth;
    for (int frame = 0; frame < 10; ++frame) {
        EXPECT_EQ(alone.measure(truth).odometry_distance_m,
                  together.measure(truth).odometry_distance_m);
        truth = next_interval(truth);
    }
}

TEST(TerrainSensorsTest, IdenticalSeedsReplayIdentically) {
    TerrainSensorConfig config;
    config.odometry = OdometrySensorConfig{0.01, 0.1};
    config.gyro = GyroSensorConfig{0.001, 0.001, 0.01};
    config.baro = BaroSensorConfig{1.0, 0.05, 0.5};
    config.rangefinder = RangefinderSensorConfig{1000.0, 1.0, 50'000};
    TerrainSensors first{config, kSeeds};
    TerrainSensors second{config, kSeeds};
    TerrainSensors other{config, TerrainSensorSeeds{12, 23, 34, 45}};
    TerrainMotionTruth truth = kTruth;
    bool differs = false;
    for (int frame = 0; frame < 20; ++frame) {
        const TerrainSensorFrame a = first.measure(truth);
        EXPECT_EQ(a, second.measure(truth));
        differs = differs || !(a == other.measure(truth));
        truth = next_interval(truth);
    }
    EXPECT_TRUE(differs);  // a different seed changes the noise
}

TEST(TerrainSensorsTest, RejectsInvalidConfigurations) {
    const auto rejects = [](TerrainSensorConfig config) {
        EXPECT_THROW((void)TerrainSensors(config, kSeeds), std::invalid_argument);
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    TerrainSensorConfig config;
    config.odometry = OdometrySensorConfig{-1.0, 0.0};
    rejects(config);
    config.odometry = OdometrySensorConfig{0.0, -0.1};
    rejects(config);
    config = {};
    config.gyro = GyroSensorConfig{nan, 0.0, 0.0};
    rejects(config);
    config.gyro = GyroSensorConfig{0.0, -1.0, 0.0};
    rejects(config);
    config = {};
    config.baro = BaroSensorConfig{0.0, 0.0, -0.5};
    rejects(config);
    config = {};
    config.rangefinder = RangefinderSensorConfig{0.0, 0.0, 0};
    rejects(config);
    config.rangefinder = RangefinderSensorConfig{100.0, 0.0, 1'000'001};
    rejects(config);
}

TEST(TerrainSensorsTest, RejectsInvalidTruthWithoutConsumingAnything) {
    TerrainSensorConfig config;
    config.odometry = OdometrySensorConfig{0.0, 0.1};
    config.rangefinder = RangefinderSensorConfig{1000.0, 1.0, 0};
    TerrainSensors sensors{config, kSeeds};
    TerrainSensors control{config, kSeeds};

    TerrainMotionTruth bad = kTruth;
    bad.at = bad.from;
    EXPECT_THROW((void)sensors.measure(bad), std::invalid_argument);
    bad = kTruth;
    bad.distance_m = -1.0;
    EXPECT_THROW((void)sensors.measure(bad), std::invalid_argument);
    bad = kTruth;
    bad.altitude_m = std::numeric_limits<double>::infinity();
    EXPECT_THROW((void)sensors.measure(bad), std::invalid_argument);
    bad = kTruth;
    bad.height_above_surface_m = -1.0;
    EXPECT_THROW((void)sensors.measure(bad), std::invalid_argument);

    EXPECT_EQ(sensors.measure(kTruth), control.measure(kTruth));  // nothing was consumed
}

}  // namespace

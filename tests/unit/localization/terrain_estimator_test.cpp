#include "fleet/localization/terrain_estimator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "fleet/dem/synthetic_terrain.hpp"
#include "fleet/localization/pose.hpp"
#include "fleet/localization/terrain_sensors.hpp"
#include "fleet/map/elevation_grid.hpp"

namespace {

using fleet::common::Tick;
using fleet::dem::SyntheticHill;
using fleet::dem::SyntheticTerrainSpec;
using fleet::dem::synthesize_terrain;
using fleet::localization::apply_en_displacement;
using fleet::localization::RangeReading;
using fleet::localization::RangeStatus;
using fleet::localization::TerrainEstimate;
using fleet::localization::TerrainEstimatorConfig;
using fleet::localization::TerrainGridEstimator;
using fleet::localization::TerrainHeightReference;
using fleet::localization::TerrainSensorFrame;
using fleet::map::ElevationGrid;
using fleet::map::ElevationGridLayout;
using fleet::map::ElevationSurface;
using fleet::map::Wgs84Coordinate;

constexpr double kPi = std::numbers::pi;

// About 6.7 km x 6.2 km of synthetic terrain with irregular hills.
constexpr ElevationGridLayout kArea{Wgs84Coordinate{46.06, 7.00}, Wgs84Coordinate{46.00, 7.08},
                                    201, 201};

[[nodiscard]] std::shared_ptr<const ElevationGrid> hills() {
    std::vector<SyntheticHill> features;
    const double lats[] = {46.012, 46.018, 46.025, 46.031, 46.008, 46.040, 46.022, 46.035,
                           46.015, 46.028, 46.045, 46.010};
    const double lons[] = {7.015, 7.032, 7.020, 7.041, 7.050, 7.028, 7.060, 7.012,
                           7.065, 7.055, 7.045, 7.030};
    const double heights[] = {120, -60, 90, 150, 70, -40, 110, 80, 60, -70, 130, 50};
    const double radii[] = {300, 250, 400, 350, 200, 300, 450, 250, 300, 350, 400, 220};
    for (std::size_t index = 0; index < 12; ++index) {
        features.push_back(
            SyntheticHill{Wgs84Coordinate{lats[index], lons[index]}, heights[index], radii[index]});
    }
    return std::make_shared<const ElevationGrid>(synthesize_terrain(
        SyntheticTerrainSpec{kArea, 600.0, std::move(features), ElevationSurface::BareEarth}));
}

[[nodiscard]] std::shared_ptr<const ElevationGrid> flat() {
    return std::make_shared<const ElevationGrid>(synthesize_terrain(
        SyntheticTerrainSpec{kArea, 500.0, {}, ElevationSurface::BareEarth}));
}

[[nodiscard]] double distance_m(Wgs84Coordinate a, Wgs84Coordinate b) {
    const double rad = kPi / 180.0;
    const double h = std::pow(std::sin((b.latitude_deg - a.latitude_deg) * rad / 2.0), 2) +
                     std::cos(a.latitude_deg * rad) * std::cos(b.latitude_deg * rad) *
                         std::pow(std::sin((b.longitude_deg - a.longitude_deg) * rad / 2.0), 2);
    return 2.0 * 6371008.8 * std::asin(std::sqrt(h));
}

// Simulated truth plus perfect-but-configurable measurements.
struct Walker {
    Walker(std::shared_ptr<const ElevationGrid> truth_terrain, Wgs84Coordinate start,
           double heading = 0.0)
        : terrain{std::move(truth_terrain)}, position{start}, heading_rad{heading} {}

    std::shared_ptr<const ElevationGrid> terrain;
    Wgs84Coordinate position;
    double heading_rad = 0.0;
    std::uint64_t at_ms = 0;
    double baro_bias_m = 0.0;
    double mount_height_m = 0.0;
    double odometry_scale = 1.0;
    std::optional<double> altitude_m;  // air when set

    TerrainSensorFrame step(double distance, double turn = 0.0,
                            RangeStatus range_status = RangeStatus::Return) {
        heading_rad += turn;
        position = apply_en_displacement(position, distance * std::sin(heading_rad),
                                         distance * std::cos(heading_rad));
        const double ground = terrain->height_at(position).height_m.value();
        TerrainSensorFrame frame;
        frame.from = Tick{at_ms};
        at_ms += 1000;
        frame.at = Tick{at_ms};
        frame.odometry_distance_m = distance * odometry_scale;
        frame.gyro_turn_rad = turn;
        if (altitude_m) {
            frame.baro_altitude_m = *altitude_m + baro_bias_m;
            frame.range = range_status == RangeStatus::Return
                              ? RangeReading{RangeStatus::Return, *altitude_m - ground}
                              : RangeReading{range_status, std::nullopt};
        } else {
            frame.baro_altitude_m = ground + mount_height_m + baro_bias_m;
        }
        return frame;
    }
};

[[nodiscard]] TerrainEstimatorConfig config_at(Wgs84Coordinate prior, double heading) {
    TerrainEstimatorConfig config;
    config.prior_center = prior;
    config.prior_heading_rad = heading;
    config.half_width_m = 300.0;
    config.cell_size_m = 10.0;
    config.position_noise_m_per_sqrt_m = 0.3;
    config.height_sigma_m = 1.0;
    config.baro_bias_sigma_m = 10.0;
    config.baro_bias_walk_m_per_sqrt_s = 0.0;
    return config;
}

constexpr Wgs84Coordinate kStart{46.010, 7.008};

// Uniform probability over (2n+1)^2 cells of size c: per-axis variance
// c^2 n (n+1) / 3, so the radial RMS for n = 30, c = 10 is sqrt(62000).
const double kUniformSpread = std::sqrt(2.0 * 100.0 * 30.0 * 31.0 / 3.0);

TEST(TerrainEstimatorConfigTest, RejectsInvalidSettings) {
    const auto rejects = [](TerrainEstimatorConfig config) {
        EXPECT_THROW(config.validate(), std::invalid_argument);
    };
    const TerrainEstimatorConfig good = config_at(kStart, 0.0);
    EXPECT_NO_THROW(good.validate());
    TerrainEstimatorConfig config = good;
    config.half_width_m = 305.0;  // not a whole number of 10 m cells
    rejects(config);
    config = good;
    config.half_width_m = 2010.0;
    config.cell_size_m = 10.0;
    rejects(config);  // beyond 2 km
    config = good;
    config.half_width_m = 2000.0;
    config.cell_size_m = 4.0;
    rejects(config);  // 1001 cells per side
    config = good;
    config.height_sigma_m = 0.0;
    rejects(config);
    config = good;
    config.baro_bias_sigma_m = 0.0;
    rejects(config);
    config = good;
    config.position_noise_m_per_sqrt_m = -0.1;
    rejects(config);
    config = good;
    config.reference = TerrainHeightReference::Air;
    config.mount_height_m = 1.0;
    rejects(config);
    config = good;
    config.prior_center.latitude_deg = std::numeric_limits<double>::quiet_NaN();
    rejects(config);
}

TEST(TerrainEstimatorTest, StartsWithoutAnEstimateAndValidatesFrames) {
    EXPECT_THROW(TerrainGridEstimator(config_at(kStart, 0.0), nullptr), std::invalid_argument);
    TerrainGridEstimator estimator{config_at(kStart, 0.0), hills()};
    EXPECT_EQ(estimator.cells_per_side(), 61U);
    EXPECT_FALSE(estimator.latest().has_value());

    Walker walker{hills(), kStart};
    TerrainSensorFrame frame = walker.step(10.0);
    TerrainSensorFrame missing = frame;
    missing.gyro_turn_rad.reset();
    EXPECT_THROW((void)estimator.update(missing), std::invalid_argument);
    missing = frame;
    missing.odometry_distance_m.reset();
    EXPECT_THROW((void)estimator.update(missing), std::invalid_argument);
    missing = frame;
    missing.baro_altitude_m.reset();
    EXPECT_THROW((void)estimator.update(missing), std::invalid_argument);
    EXPECT_FALSE(estimator.latest().has_value());  // nothing changed

    (void)estimator.update(frame);
    ASSERT_TRUE(estimator.latest().has_value());
    TerrainSensorFrame earlier = frame;
    earlier.from = Tick{0};
    earlier.at = Tick{500};
    EXPECT_THROW((void)estimator.update(earlier), std::invalid_argument);

    TerrainEstimatorConfig air = config_at(kStart, 0.0);
    air.reference = TerrainHeightReference::Air;
    TerrainGridEstimator air_estimator{air, hills()};
    EXPECT_THROW((void)air_estimator.update(frame), std::invalid_argument);  // no range field
}

TEST(TerrainEstimatorTest, UninformativeTerrainOnlyDeadReckons) {
    TerrainEstimatorConfig config = config_at(kStart, kPi / 2.0);
    config.position_noise_m_per_sqrt_m = 0.0;
    TerrainGridEstimator estimator{config, flat()};
    Walker walker{flat(), kStart, kPi / 2.0};
    for (int step = 0; step < 20; ++step) {
        (void)estimator.update(walker.step(10.0));
    }
    const TerrainEstimate& estimate = *estimator.latest();
    EXPECT_LT(distance_m(estimate.estimate.position, walker.position), 1e-3);
    EXPECT_NEAR(estimate.spread_m, kUniformSpread, 1e-6);  // still the uniform prior
    EXPECT_NEAR(estimate.effective_cells, 61.0 * 61.0, 1e-6);
    EXPECT_DOUBLE_EQ(estimate.estimate.heading_rad, kPi / 2.0);
    EXPECT_TRUE(estimate.measured);
    EXPECT_EQ(estimate.estimate.estimated_at, Tick{20000});
}

TEST(TerrainEstimatorTest, ConvergesOnHillsAndEstimatesTheBaroBias) {
    const auto terrain = hills();
    // The prior is centred 150 m south-west of the true start.
    const Wgs84Coordinate prior = apply_en_displacement(kStart, -106.0, -106.0);
    ASSERT_NEAR(distance_m(prior, kStart), 150.0, 1.0);
    TerrainGridEstimator estimator{config_at(prior, kPi / 4.0), terrain};
    Walker walker{terrain, kStart, kPi / 4.0};
    walker.baro_bias_m = 7.0;  // unknown to the estimator
    for (int step = 0; step < 200; ++step) {
        (void)estimator.update(walker.step(10.0));
    }
    const TerrainEstimate& estimate = *estimator.latest();
    const double error = distance_m(estimate.estimate.position, walker.position);
    EXPECT_LT(error, 15.0);
    EXPECT_LT(estimate.spread_m, 20.0);
    EXPECT_LT(estimate.effective_cells, 50.0);
    EXPECT_NEAR(estimate.baro_bias_m, 7.0, 1.5);
    EXPECT_EQ(estimate.resets, 0U);
}

TEST(TerrainEstimatorTest, AirReferenceUsesTheRangeAndSkipsMissingReturns) {
    const auto terrain = hills();
    TerrainEstimatorConfig config = config_at(kStart, kPi / 4.0);
    config.reference = TerrainHeightReference::Air;
    TerrainGridEstimator estimator{config, terrain};
    Walker walker{terrain, kStart, kPi / 4.0};
    walker.altitude_m = 1200.0;
    EXPECT_TRUE(estimator.update(walker.step(15.0)).measured);
    const TerrainEstimate dropped = estimator.update(walker.step(15.0, 0.0, RangeStatus::Dropout));
    EXPECT_FALSE(dropped.measured);
    EXPECT_FALSE(estimator.update(walker.step(15.0, 0.0, RangeStatus::OutOfRange)).measured);
    for (int step = 0; step < 150; ++step) {
        (void)estimator.update(walker.step(15.0));
    }
    EXPECT_LT(distance_m(estimator.latest()->estimate.position, walker.position), 15.0);
}

TEST(TerrainEstimatorTest, CellsOffTheRobotsMapLoseTheirProbability) {
    // The prior grid straddles the map's west edge (7.00): after one height
    // measurement, only cells on the map remain.
    const auto terrain = flat();
    const Wgs84Coordinate prior{46.03, 7.0};
    TerrainEstimatorConfig config = config_at(prior, 0.0);
    config.position_noise_m_per_sqrt_m = 0.0;
    TerrainGridEstimator estimator{config, terrain};
    Walker walker{terrain, prior};
    const TerrainEstimate& estimate = estimator.update(walker.step(0.0));
    EXPECT_GE(estimate.estimate.position.longitude_deg, 7.0);
    EXPECT_GE(estimate.most_likely.longitude_deg, 7.0);
    EXPECT_LT(estimate.effective_cells, 61.0 * 61.0 * 0.6);  // about half the cells
}

TEST(TerrainEstimatorTest, LosingAllProbabilityResetsToThePrior) {
    TerrainEstimatorConfig config = config_at(kStart, 0.0);
    config.height_sigma_m = 0.01;
    config.baro_bias_sigma_m = 0.01;
    config.position_noise_m_per_sqrt_m = 0.0;
    TerrainGridEstimator estimator{config, hills()};
    Walker walker{hills(), kStart};
    walker.baro_bias_m = 1.0e6;  // no cell can explain this height
    const TerrainEstimate& estimate = estimator.update(walker.step(10.0));
    EXPECT_EQ(estimate.resets, 1U);
    EXPECT_NEAR(estimate.spread_m, kUniformSpread, 1e-6);
}

TEST(TerrainEstimatorTest, RecentresToFollowDriftingDeadReckoning) {
    // Odometry reports 10% too much: after 3 km dead reckoning alone is 300 m
    // off, beyond the grid's 200 m half-width. Terrain matching keeps the
    // belief near the truth because the grid re-centres on the estimate. The
    // systematic scale error is not in the estimator's random-walk model, so
    // the belief lags on flatter stretches (ADR-025 limitation) but never
    // follows the drift.
    const auto terrain = hills();
    TerrainEstimatorConfig config = config_at(kStart, kPi / 4.0);
    config.half_width_m = 200.0;
    config.position_noise_m_per_sqrt_m = 0.5;
    TerrainGridEstimator estimator{config, terrain};
    Walker walker{terrain, kStart, kPi / 4.0};
    walker.odometry_scale = 1.1;
    double worst = 0.0;
    for (int step = 0; step < 300; ++step) {
        const TerrainEstimate& estimate = estimator.update(walker.step(10.0));
        worst = std::max(worst, distance_m(estimate.estimate.position, walker.position));
    }
    EXPECT_LT(worst, 100.0);  // dead reckoning alone ends 300 m off
    EXPECT_LT(distance_m(estimator.latest()->estimate.position, walker.position), 50.0);
    EXPECT_EQ(estimator.latest()->resets, 0U);
}

TEST(TerrainEstimatorTest, IdenticalInputsGiveIdenticalBelief) {
    const auto terrain = hills();
    TerrainGridEstimator first{config_at(kStart, 0.5), terrain};
    TerrainGridEstimator second{config_at(kStart, 0.5), terrain};
    Walker walker{terrain, kStart, 0.5};
    walker.baro_bias_m = 3.0;
    for (int step = 0; step < 100; ++step) {
        const TerrainSensorFrame frame = walker.step(10.0, step % 10 == 0 ? 0.05 : 0.0);
        const TerrainEstimate a = first.update(frame);
        const TerrainEstimate b = second.update(frame);
        ASSERT_EQ(a.estimate, b.estimate);
        ASSERT_EQ(a.most_likely, b.most_likely);
        ASSERT_EQ(a.spread_m, b.spread_m);
        ASSERT_EQ(a.baro_bias_m, b.baro_bias_m);
    }
}

}  // namespace

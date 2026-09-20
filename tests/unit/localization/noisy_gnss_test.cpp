#include "fleet/localization/gnss_model.hpp"
#include "fleet/localization/pose.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include "fleet/simulation/deterministic_rng.hpp"

namespace {

using fleet::localization::apply_en_displacement;
using fleet::localization::GnssNoiseConfig;
using fleet::localization::GroundTruthPose;
using fleet::localization::LocalizationEstimate;
using fleet::localization::NoisyGnss;
using fleet::localization::PerfectGnss;
using fleet::map::Wgs84Coordinate;
using fleet::simulation::DeterministicRng;

constexpr std::uint64_t kDrawsPerNoiseComponent = 12;

// Twin-RNG helper: returns true iff `used` (after the model call) and a
// never-touched control with the same seed agree on the next draw.
class NoisyGnssTest : public ::testing::Test {
protected:
    DeterministicRng rng_probe_{7};
    DeterministicRng rng_control_{7};
};

TEST_F(NoisyGnssTest, ZeroNoiseEqualsPerfectGnssExactlyAndConsumesNothing) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 1.25,
                                fleet::common::Tick{4200}};
    const NoisyGnss quiet{GnssNoiseConfig{0.0, 0.0}};
    const PerfectGnss perfect;

    const std::optional<LocalizationEstimate> quiet_fix = quiet.measure(truth, rng_probe_);
    const std::optional<LocalizationEstimate> perfect_fix = perfect.measure(truth, rng_control_);
    ASSERT_TRUE(quiet_fix.has_value());
    ASSERT_TRUE(perfect_fix.has_value());
    EXPECT_EQ(*quiet_fix, *perfect_fix);  // bit-identical to the baseline
    EXPECT_EQ(quiet_fix->estimated_at, truth.at);
    // ...and consumed zero randomness.
    EXPECT_EQ(rng_probe_.uniform_below(1'000'000), rng_control_.uniform_below(1'000'000));
}

TEST_F(NoisyGnssTest, SameSeedSameTruthGivesIdenticalFixSequences) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.5,
                                fleet::common::Tick{100}};
    const NoisyGnss model{GnssNoiseConfig{5.0, 0.02}};

    std::vector<LocalizationEstimate> first;
    std::vector<LocalizationEstimate> second;
    DeterministicRng rng_a{1234};
    DeterministicRng rng_b{1234};
    for (std::uint64_t i = 0; i < 25; ++i) {
        first.push_back(*model.measure(truth, rng_a));
        second.push_back(*model.measure(truth, rng_b));
    }
    EXPECT_EQ(first, second);
}

TEST_F(NoisyGnssTest, DifferentSeedGivesDifferentFixSequences) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.5,
                                fleet::common::Tick{100}};
    const NoisyGnss model{GnssNoiseConfig{5.0, 0.02}};

    DeterministicRng rng_a{1};
    DeterministicRng rng_b{2};
    bool any_difference = false;
    for (std::uint64_t i = 0; i < 25; ++i) {
        const LocalizationEstimate a = *model.measure(truth, rng_a);
        const LocalizationEstimate b = *model.measure(truth, rng_b);
        any_difference = any_difference || !(a == b);
    }
    EXPECT_TRUE(any_difference);
}

TEST_F(NoisyGnssTest, ConsumptionIsExactly36DrawsForBothKnobs) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.5,
                                fleet::common::Tick{1}};
    const NoisyGnss model{GnssNoiseConfig{5.0, 0.02}};
    (void)model.measure(truth, rng_probe_);

    // Advance the control by the documented consumption: 12 east + 12
    // north + 12 heading draws; then both streams agree again.
    for (std::uint64_t i = 0;
         i < 3 * kDrawsPerNoiseComponent; ++i) {
        (void)rng_control_.uniform_below(1ULL << 32);
    }
    EXPECT_EQ(rng_probe_.uniform_below(1'000'000), rng_control_.uniform_below(1'000'000));
}

TEST_F(NoisyGnssTest, ConsumptionIsExactly24DrawsForPositionOnly) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.5,
                                fleet::common::Tick{1}};
    const NoisyGnss model{GnssNoiseConfig{5.0, 0.0}};
    (void)model.measure(truth, rng_probe_);
    for (std::uint64_t i = 0; i < 2 * kDrawsPerNoiseComponent; ++i) {
        (void)rng_control_.uniform_below(1ULL << 32);
    }
    EXPECT_EQ(rng_probe_.uniform_below(1'000'000), rng_control_.uniform_below(1'000'000));
}

TEST_F(NoisyGnssTest, ConsumptionIsExactly12DrawsForHeadingOnly) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.5,
                                fleet::common::Tick{1}};
    const NoisyGnss model{GnssNoiseConfig{0.0, 0.02}};
    (void)model.measure(truth, rng_probe_);
    for (std::uint64_t i = 0; i < kDrawsPerNoiseComponent; ++i) {
        (void)rng_control_.uniform_below(1ULL << 32);
    }
    EXPECT_EQ(rng_probe_.uniform_below(1'000'000), rng_control_.uniform_below(1'000'000));
}

TEST_F(NoisyGnssTest, HeadingAndPositionNoiseAreSeparateKnobs) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.75,
                                fleet::common::Tick{50}};

    // Position noise only: HEADING is untouched (GNSS position sigma
    // must not silently perturb heading).
    const NoisyGnss position_only{GnssNoiseConfig{5.0, 0.0}};
    const LocalizationEstimate position_fix = *position_only.measure(truth, rng_probe_);
    EXPECT_DOUBLE_EQ(position_fix.heading_rad, truth.heading_rad);
    EXPECT_NE(position_fix.position, truth.position);  // position IS perturbed

    // Heading noise only: POSITION is untouched.
    const NoisyGnss heading_only{GnssNoiseConfig{0.0, 0.05}};
    const LocalizationEstimate heading_fix = *heading_only.measure(truth, rng_probe_);
    EXPECT_EQ(heading_fix.position, truth.position);
    EXPECT_NE(heading_fix.heading_rad, truth.heading_rad);  // heading IS perturbed
}

TEST_F(NoisyGnssTest, EstimatedAtTracksTruthNotInvocation) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.5,
                                fleet::common::Tick{7200}};
    const NoisyGnss model{GnssNoiseConfig{5.0, 0.02}};
    const LocalizationEstimate fix = *model.measure(truth, rng_probe_);
    EXPECT_EQ(fix.estimated_at, truth.at);
}

TEST_F(NoisyGnssTest, RejectsInvalidNoiseConfiguration) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(static_cast<void>(NoisyGnss{GnssNoiseConfig{-1.0, 0.0}}),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(NoisyGnss{GnssNoiseConfig{0.0, -0.1}}),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(NoisyGnss{GnssNoiseConfig{nan, 0.0}}),
                 std::invalid_argument);
    EXPECT_THROW(
        static_cast<void>(NoisyGnss{
            GnssNoiseConfig{0.0, std::numeric_limits<double>::infinity()}}),
        std::invalid_argument);
}

TEST(EnDisplacementTest, SameEastMetersDifferentLongitudeDegreesByLatitude) {
    // THE "never add noise to degrees" contract, made visible: 1000 m
    // east is a much larger longitude change at high latitude.
    const Wgs84Coordinate equator{0.0, 0.0};
    const Wgs84Coordinate high_latitude{80.0, 0.0};

    const Wgs84Coordinate equator_moved = apply_en_displacement(equator, 1000.0, 0.0);
    const Wgs84Coordinate north_moved = apply_en_displacement(high_latitude, 1000.0, 0.0);

    // North displacement of zero changes nothing.
    EXPECT_DOUBLE_EQ(equator_moved.latitude_deg, 0.0);
    // Both moved ~1000 m east, but the degree deltas differ hugely.
    EXPECT_GT(north_moved.longitude_deg, equator_moved.longitude_deg * 4.0);

    // Exact formula check (tangent-plane approximation, R=6371008.8):
    const double meters_per_deg_lat = 6371008.8 * std::numbers::pi / 180.0;
    EXPECT_NEAR(equator_moved.longitude_deg, 1000.0 / meters_per_deg_lat, 1e-12);
    EXPECT_NEAR(equator_moved.latitude_deg, 0.0, 1e-12);

    // Pure north displacement at the equator.
    const Wgs84Coordinate north_only = apply_en_displacement(equator, 0.0, 2000.0);
    EXPECT_NEAR(north_only.latitude_deg, 2000.0 / meters_per_deg_lat, 1e-12);
    EXPECT_DOUBLE_EQ(north_only.longitude_deg, 0.0);
}

TEST(EnDisplacementTest, DatelineCrossingWrapsLongitude) {
    // +179.9999 with +30 m east must emerge just east of -180 — the
    // requested displacement is PRESERVED by wrapping, never destroyed
    // by clamping to +180. The overshoot past +180 is
    // (30 m in degrees) - (180 - 179.9999).
    const double meters_per_deg_lat = 6371008.8 * std::numbers::pi / 180.0;
    const double delta_deg = 30.0 / meters_per_deg_lat;

    const Wgs84Coordinate near_dateline{0.0, 179.9999};
    const Wgs84Coordinate moved = apply_en_displacement(near_dateline, 30.0, 0.0);
    EXPECT_NEAR(moved.longitude_deg, -180.0 + (delta_deg - 0.0001), 1e-9);
    EXPECT_DOUBLE_EQ(moved.latitude_deg, 0.0);

    // Mirrored: westing across -180 emerges just west of +180.
    const Wgs84Coordinate west_side{0.0, -179.9999};
    const Wgs84Coordinate moved_west = apply_en_displacement(west_side, -30.0, 0.0);
    EXPECT_NEAR(moved_west.longitude_deg, 180.0 - (delta_deg - 0.0001), 1e-9);
}

TEST(EnDisplacementTest, NearPoleEastDisplacementFailsExplicitly) {
    // |latitude| > 89 deg with a non-zero east component is outside the
    // local-tangent approximation's supported domain: explicit failure,
    // never a silently clamped or manufactured position.
    const Wgs84Coordinate near_north_pole{89.5, 0.0};
    EXPECT_THROW(static_cast<void>(apply_en_displacement(near_north_pole, 10.0, 0.0)),
                 std::invalid_argument);
    const Wgs84Coordinate near_south_pole{-89.9, 45.0};
    EXPECT_THROW(static_cast<void>(apply_en_displacement(near_south_pole, -5.0, 0.0)),
                 std::invalid_argument);
    // Pure NORTH displacement within the polar region is still valid
    // (no east conversion involved) as long as it does not cross the pole.
    const Wgs84Coordinate poleward{89.5, 0.0};
    const Wgs84Coordinate moved_north = apply_en_displacement(poleward, 0.0, 1000.0);
    EXPECT_GT(moved_north.latitude_deg, 89.5);
    EXPECT_LE(moved_north.latitude_deg, 90.0);
}

TEST(EnDisplacementTest, PoleCrossingNorthDisplacementFailsExplicitly) {
    const Wgs84Coordinate almost_pole{89.999, 0.0};
    EXPECT_THROW(static_cast<void>(apply_en_displacement(almost_pole, 0.0, 500'000.0)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(apply_en_displacement(Wgs84Coordinate{-89.999, 0.0},
                                                         0.0, -500'000.0)),
                 std::invalid_argument);
}

TEST(EnDisplacementTest, NonFiniteDisplacementFailsExplicitly) {
    const Wgs84Coordinate origin{53.5511, 9.9937};
    EXPECT_THROW(static_cast<void>(apply_en_displacement(
                      origin, std::numeric_limits<double>::quiet_NaN(), 0.0)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(apply_en_displacement(
                      origin, 0.0, std::numeric_limits<double>::infinity())),
                 std::invalid_argument);
}

TEST(NoisyGnssSamplerTest, MidpointCenteredUniformIsExactlySymmetric) {
    // The sampler arithmetic, replicated here: complement integer draws
    // must map to exactly opposite sample values (the property that
    // k/2^32 - 0.5 lacks — its complement pairs sum to -2^-32).
    const auto sample = [](std::uint64_t k) {
        return (static_cast<double>(k) + 0.5) / 4294967296.0 - 0.5;
    };
    const std::uint64_t n = 1ULL << 32;
    const std::vector<std::uint64_t> probes{
        0ULL, 1ULL, 42ULL, 0x80000000ULL, 0xDEADBEEFULL, n - 2, n - 1};
    for (const std::uint64_t k : probes) {
        EXPECT_DOUBLE_EQ(sample(k) + sample(n - 1 - k), 0.0) << "k = " << k;
        EXPECT_GT(sample(k), -0.5);
        EXPECT_LT(sample(k), 0.5);
    }
}

TEST_F(NoisyGnssTest, EmpiricalNoiseScaleMatchesSigma) {
    // Statistical sanity (deterministic: fixed seed): the empirical
    // horizontal error magnitude over many fixes is in the right
    // ballpark of sigma. Irwin-Hall(12) has variance exactly sigma^2
    // per axis, so the horizontal RMS ~ sigma * sqrt(2).
    const double sigma = 8.0;
    const NoisyGnss model{GnssNoiseConfig{sigma, 0.0}};
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.0,
                                fleet::common::Tick{0}};

    const double meters_per_deg_lat = 6371008.8 * std::numbers::pi / 180.0;
    const double meters_per_deg_lon =
        meters_per_deg_lat * std::cos(truth.position.latitude_deg * std::numbers::pi / 180.0);

    double sum_squared_distance = 0.0;
    constexpr std::uint64_t kFixes = 400;
    DeterministicRng rng{99};
    for (std::uint64_t i = 0; i < kFixes; ++i) {
        const LocalizationEstimate fix = *model.measure(truth, rng);
        const double north_m =
            (fix.position.latitude_deg - truth.position.latitude_deg) * meters_per_deg_lat;
        const double east_m = (fix.position.longitude_deg - truth.position.longitude_deg) *
                              meters_per_deg_lon;
        sum_squared_distance += east_m * east_m + north_m * north_m;
    }
    const double rms = std::sqrt(sum_squared_distance / kFixes);
    const double expected_rms = sigma * std::sqrt(2.0);
    // Generous bounds: this is a sanity guard, not a statistics test.
    EXPECT_GT(rms, expected_rms * 0.7);
    EXPECT_LT(rms, expected_rms * 1.3);
}

}  // namespace

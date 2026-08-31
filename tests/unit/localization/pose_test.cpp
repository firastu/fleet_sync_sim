#include "fleet/localization/gnss_model.hpp"
#include "fleet/localization/pose.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <numbers>
#include <optional>

#include "fleet/simulation/deterministic_rng.hpp"

namespace {

using fleet::localization::GroundTruthPose;
using fleet::localization::LocalizationEstimate;
using fleet::localization::normalize_heading;
using fleet::localization::PerfectGnss;
using fleet::localization::UnavailableGnss;
using fleet::simulation::DeterministicRng;
using fleet::map::Wgs84Coordinate;

// Twin-RNG probe: rng_probe_ is passed to models, rng_control_ never is.
// Identical next draws => the model consumed no randomness.
class GnssModelTest : public ::testing::Test {
protected:
    DeterministicRng rng_probe_{7};
    DeterministicRng rng_control_{7};
};

TEST(PoseTest, NormalizesHeadingIntoHalfOpenInterval) {
    const double two_pi = 2.0 * std::numbers::pi;
    EXPECT_DOUBLE_EQ(normalize_heading(0.0), 0.0);
    EXPECT_DOUBLE_EQ(normalize_heading(std::numbers::pi / 2.0),
                     std::numbers::pi / 2.0);
    EXPECT_NEAR(normalize_heading(two_pi), 0.0, 1e-15);
    EXPECT_NEAR(normalize_heading(-std::numbers::pi / 2.0),
                3.0 * std::numbers::pi / 2.0, 1e-15);
    EXPECT_NEAR(normalize_heading(5.0 * std::numbers::pi / 2.0),
                std::numbers::pi / 2.0, 1e-15);
    EXPECT_NEAR(normalize_heading(7.5 * std::numbers::pi), 1.5 * std::numbers::pi, 1e-12);
}

TEST(PoseTest, RejectsNonFiniteHeadings) {
    // NaN/inf must never quietly become localization state.
    EXPECT_THROW(static_cast<void>(normalize_heading(
                     std::numeric_limits<double>::quiet_NaN())),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(normalize_heading(
                     std::numeric_limits<double>::infinity())),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(normalize_heading(
                     -std::numeric_limits<double>::infinity())),
                 std::invalid_argument);
}

TEST_F(GnssModelTest, PerfectGnssCopiesTruthExactlyAndConsumesNoRandomness) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937},
                                 5.0 * std::numbers::pi / 8.0, fleet::common::Tick{4200}};
    const PerfectGnss model;
    const std::optional<LocalizationEstimate> estimate = model.measure(truth, rng_probe_);
    ASSERT_TRUE(estimate.has_value());

    // The four locked contracts:
    // 1. position copied exactly (values, no rounding);
    EXPECT_EQ(estimate->position, truth.position);
    // 2. heading copied exactly;
    EXPECT_DOUBLE_EQ(estimate->heading_rad, truth.heading_rad);
    // 3. estimated_at == the truth tick the estimate refers to (NOT the
    //    request time — staleness reasoning depends on this).
    EXPECT_EQ(estimate->estimated_at, truth.at);
    // 4. RNG state unchanged: an identically-seeded control that never
    //    saw the model produces the same next draw.
    EXPECT_EQ(rng_probe_.uniform_below(1'000'000), rng_control_.uniform_below(1'000'000));
    // And the full-struct equality still holds for good measure.
    EXPECT_EQ(*estimate,
              (LocalizationEstimate{truth.position, truth.heading_rad, truth.at}));
}

TEST_F(GnssModelTest, UnavailableGnssNeverFixesAndConsumesNoRandomness) {
    const GroundTruthPose truth{Wgs84Coordinate{53.5511, 9.9937}, 0.0,
                                 fleet::common::Tick{1}};
    const UnavailableGnss model;
    EXPECT_EQ(model.measure(truth, rng_probe_), std::nullopt);
    // Zero-consumption contract: switching to/from an outage model must
    // never shift future noise sequences of a shared RNG.
    EXPECT_EQ(rng_probe_.uniform_below(1'000'000), rng_control_.uniform_below(1'000'000));
}

TEST_F(GnssModelTest, ModelsAreInterchangeableThroughTheInterface) {
    // Outage is model selection (scenario policy), not special-casing:
    // the same call site works for any GnssModel.
    const GroundTruthPose truth{Wgs84Coordinate{52.52, 13.405}, 1.0,
                                 fleet::common::Tick{100}};
    const PerfectGnss perfect;
    const UnavailableGnss out;
    EXPECT_NE(perfect.measure(truth, rng_probe_), out.measure(truth, rng_probe_));
}

}  // namespace

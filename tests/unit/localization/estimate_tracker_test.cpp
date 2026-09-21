#include "fleet/localization/estimate_tracker.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "fleet/common/time.hpp"
#include "fleet/localization/pose.hpp"
#include "fleet/map/geometry.hpp"

namespace {

using fleet::common::Tick;
using fleet::localization::LocalizationEstimate;
using fleet::localization::LocalizationTracker;
using fleet::map::Wgs84Coordinate;

[[nodiscard]] LocalizationEstimate fix_at(std::uint64_t tick, double lat = 52.370,
                                          double lon = 9.730, double heading = 0.5) {
    return LocalizationEstimate{Wgs84Coordinate{lat, lon}, heading, Tick{tick}};
}

TEST(LocalizationTrackerTest, NoEstimateBeforeFirstFix) {
    LocalizationTracker tracker;
    EXPECT_FALSE(tracker.estimate().has_value());
    // Age exists only when an estimate exists — never a manufactured 0.
    EXPECT_FALSE(tracker.age_at(Tick{5000}).has_value());
}

TEST(LocalizationTrackerTest, SuccessfulFixBecomesRetainedEstimate) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(1000, 52.5, 9.5, 1.25));
    ASSERT_TRUE(tracker.estimate().has_value());
    EXPECT_EQ(tracker.estimate()->estimated_at, Tick{1000});
    EXPECT_EQ(tracker.estimate()->position, (Wgs84Coordinate{52.5, 9.5}));
    EXPECT_EQ(tracker.age_at(Tick{1000}), std::uint64_t{0});
}

TEST(LocalizationTrackerTest, NoFixDoesNotEraseRetainedEstimate) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(1000));
    tracker.apply_sample(std::nullopt);  // outage sample
    tracker.apply_sample(std::nullopt);
    ASSERT_TRUE(tracker.estimate().has_value());
    EXPECT_EQ(tracker.estimate()->estimated_at, Tick{1000});  // unchanged
    EXPECT_EQ(tracker.estimate()->position, (Wgs84Coordinate{52.370, 9.730}));
}

TEST(LocalizationTrackerTest, AgeGrowsWithLogicalTimeWhileEstimateIsUnchanged) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(1000));
    EXPECT_EQ(tracker.age_at(Tick{1000}), std::uint64_t{0});
    EXPECT_EQ(tracker.age_at(Tick{2000}), std::uint64_t{1000});
    EXPECT_EQ(tracker.age_at(Tick{4500}), std::uint64_t{3500});
    // Querying did not mutate anything: same answer again, still no
    // "last inspected" timestamp.
    EXPECT_EQ(tracker.estimate()->estimated_at, Tick{1000});
    EXPECT_EQ(tracker.age_at(Tick{4500}), std::uint64_t{3500});
}

TEST(LocalizationTrackerTest, NewFixResetsAgeToItsOwnTimestamp) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(1000));
    tracker.apply_sample(std::nullopt);       // age grows...
    EXPECT_EQ(tracker.age_at(Tick{3000}), std::uint64_t{2000});
    tracker.apply_sample(fix_at(3000, 52.6, 9.6, 2.0));  // ...until the next fix
    EXPECT_EQ(tracker.age_at(Tick{3000}), std::uint64_t{0});
    EXPECT_EQ(tracker.estimate()->position, (Wgs84Coordinate{52.6, 9.6}));
    EXPECT_EQ(tracker.age_at(Tick{3100}), std::uint64_t{100});
}

TEST(LocalizationTrackerTest, QueryBeforeEstimatedAtIsRejected) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(1000));
    // now < estimated_at is impossible in logical time; a negative age
    // must not silently exist (project error contract: explicit throw).
    EXPECT_THROW((void)tracker.age_at(Tick{999}), std::invalid_argument);
    EXPECT_THROW((void)tracker.age_at(Tick{0}), std::invalid_argument);
    // The rejected query changed nothing.
    EXPECT_EQ(tracker.age_at(Tick{1000}), std::uint64_t{0});
}

TEST(LocalizationTrackerTest, NonFiniteFixIsRejected) {
    LocalizationTracker tracker;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        tracker.apply_sample(LocalizationEstimate{Wgs84Coordinate{nan, 9.7}, 0.0, Tick{5}}),
        std::invalid_argument);
    EXPECT_THROW(tracker.apply_sample(LocalizationEstimate{
                                       Wgs84Coordinate{52.0, std::numeric_limits<double>::infinity()},
                                       0.0, Tick{5}}),
                 std::invalid_argument);
    EXPECT_THROW(tracker.apply_sample(
                     LocalizationEstimate{Wgs84Coordinate{52.0, 9.7}, nan, Tick{5}}),
                 std::invalid_argument);
    // Nothing became state.
    EXPECT_FALSE(tracker.estimate().has_value());
}

TEST(LocalizationTrackerTest, DeadReckoningAdvancesWhileNoFixRemainsFrozen) {
    LocalizationTracker frozen;
    LocalizationTracker moving;
    frozen.apply_sample(fix_at(0, 0.0, 0.0, 0.0));
    moving.apply_sample(frozen.estimate());
    const fleet::localization::DeadReckoningConfig config{0.1, 0.0};
    for (std::uint64_t tick = 1; tick <= 10; ++tick) {
        frozen.apply_sample(std::nullopt);
        moving.propagate({10.0, 0.0, Tick{tick - 1}, Tick{tick}}, config);
        const auto expected = fleet::localization::apply_en_displacement(
            Wgs84Coordinate{0.0, 0.0}, 0.0, 11.0 * static_cast<double>(tick));
        EXPECT_NEAR(moving.estimate()->position.latitude_deg, expected.latitude_deg, 1e-12);
        EXPECT_EQ(moving.age_at(Tick{tick}), 0U);
        EXPECT_EQ(frozen.age_at(Tick{tick}), tick);
    }
}

TEST(LocalizationTrackerTest, DeadReckoningPreservesHeadingErrorAndAccumulatesDrift) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(0, 0.0, 0.0, 0.2));
    tracker.propagate({10.0, std::numbers::pi / 2.0, Tick{0}, Tick{1}}, {0.0, 0.01});
    EXPECT_NEAR(tracker.estimate()->heading_rad, 0.3 + std::numbers::pi / 2.0, 1e-12);
    EXPECT_GT(tracker.estimate()->position.longitude_deg, 0.0);
    EXPECT_LT(tracker.estimate()->position.latitude_deg, 0.0);
}

TEST(LocalizationTrackerTest, DeadReckoningDoesNotManufactureAnInitialFix) {
    LocalizationTracker tracker;
    tracker.propagate({10.0, 0.1, Tick{0}, Tick{1}}, {});
    EXPECT_FALSE(tracker.estimate());
}

TEST(LocalizationTrackerTest, DeadReckoningRejectsInvalidMotionWithoutMutation) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(10));
    const auto original = tracker.estimate();
    EXPECT_THROW(tracker.propagate({1.0, 0.0, Tick{9}, Tick{11}}, {}), std::invalid_argument);
    EXPECT_THROW(tracker.propagate({-1.0, 0.0, Tick{10}, Tick{11}}, {}), std::invalid_argument);
    EXPECT_THROW(tracker.propagate({1.0, 0.0, Tick{10}, Tick{11}}, {-1.0, 0.0}),
                 std::invalid_argument);
    EXPECT_THROW(tracker.propagate({1.0, 0.0, Tick{10}, Tick{11}},
                                  {0.0, std::numeric_limits<double>::infinity()}),
                 std::invalid_argument);
    EXPECT_EQ(tracker.estimate(), original);
}

TEST(LocalizationTrackerTest, StationaryPropagationRefreshesTimeNotAbsoluteFixOrPosition) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(10));
    const auto position = tracker.estimate()->position;
    tracker.propagate({0.0, 0.0, Tick{10}, Tick{20}}, {0.1, 0.5});
    EXPECT_EQ(tracker.estimate()->position, position);
    EXPECT_DOUBLE_EQ(tracker.estimate()->heading_rad, 0.5);
    EXPECT_EQ(tracker.estimate()->estimated_at, Tick{20});
    EXPECT_EQ(tracker.last_fix_at(), Tick{10});
    EXPECT_TRUE(tracker.dead_reckoned());
    const auto propagated = tracker.estimate();
    tracker.apply_sample(std::nullopt);
    EXPECT_EQ(tracker.estimate(), propagated);
    tracker.apply_sample(fix_at(30));
    EXPECT_EQ(tracker.last_fix_at(), Tick{30});
    EXPECT_FALSE(tracker.dead_reckoned());
}

TEST(LocalizationTrackerTest, DeadReckoningWrapsAntimeridianAndRejectsPoleCrossingAtomically) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(0, 0.0, 179.99999, std::numbers::pi / 2.0));
    tracker.propagate({10.0, 0.0, Tick{0}, Tick{1}}, {});
    EXPECT_LT(tracker.estimate()->position.longitude_deg, -179.99);
    tracker.apply_sample(fix_at(10, 89.9999, 0.0, 0.0));
    const auto original = tracker.estimate();
    EXPECT_THROW(tracker.propagate({100.0, 0.0, Tick{10}, Tick{11}}, {}), std::invalid_argument);
    EXPECT_EQ(tracker.estimate(), original);
    EXPECT_FALSE(tracker.dead_reckoned());
    EXPECT_EQ(tracker.last_fix_at(), Tick{10});
}

TEST(LocalizationTrackerTest, DeadReckoningRejectsNonFiniteAndOversizedInputs) {
    LocalizationTracker tracker;
    tracker.apply_sample(fix_at(10));
    const double infinity = std::numeric_limits<double>::infinity();
    const auto original = tracker.estimate();
    EXPECT_THROW(tracker.propagate({infinity, 0.0, Tick{10}, Tick{11}}, {}), std::invalid_argument);
    EXPECT_THROW(tracker.propagate({1.0, infinity, Tick{10}, Tick{11}}, {}), std::invalid_argument);
    EXPECT_THROW(tracker.propagate({1.0, 0.0, Tick{10}, Tick{9}}, {}), std::invalid_argument);
    EXPECT_THROW(tracker.propagate({1000001.0, 0.0, Tick{10}, Tick{11}}, {}), std::invalid_argument);
    EXPECT_THROW(tracker.propagate({1.0, 0.0, Tick{10}, Tick{11}}, {infinity, 0.0}),
                 std::invalid_argument);
    EXPECT_EQ(tracker.estimate(), original);
}

}  // namespace

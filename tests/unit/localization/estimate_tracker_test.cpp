#include "fleet/localization/estimate_tracker.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
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

}  // namespace

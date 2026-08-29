#include "fleet/map/dynamic_overlay.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::OverlayVersion;
using fleet::common::RobotId;
using fleet::common::Tick;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::testsupport::make_grid_map;

EdgeDynamicState blocked_observation(Tick at, std::uint64_t sequence, double confidence) {
    return EdgeDynamicState{
        .status = EdgeStatus::Blocked,
        .observed_at = at,
        .source = RobotId{1},
        .source_sequence = sequence,
        .confidence = confidence,
    };
}

class DynamicOverlayTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    const EdgeId fg_{*grid_.base.graph().edge_between(grid_.node("F"), grid_.node("G"))};
    const EdgeId ab_{*grid_.base.graph().edge_between(grid_.node("A"), grid_.node("B"))};
    DynamicMapOverlay overlay_{grid_.base.graph().edge_count()};
};

TEST_F(DynamicOverlayTest, StartsEmpty) {
    EXPECT_EQ(overlay_.version(), OverlayVersion{0});
    EXPECT_EQ(overlay_.tracked_count(), 0U);
    EXPECT_TRUE(overlay_.tracked_edges().empty());
    EXPECT_EQ(overlay_.find(fg_), nullptr);
}

TEST_F(DynamicOverlayTest, ApplyStoresStateAndBumpsVersion) {
    const EdgeDynamicState observation = blocked_observation(Tick{5000}, 7, 0.9);
    ASSERT_TRUE(overlay_.apply(fg_, observation));

    ASSERT_NE(overlay_.find(fg_), nullptr);
    const EdgeDynamicState& stored = *overlay_.find(fg_);
    EXPECT_EQ(stored.status, EdgeStatus::Blocked);
    EXPECT_EQ(stored.observed_at, Tick{5000});
    EXPECT_EQ(stored.source, RobotId{1});
    EXPECT_EQ(stored.source_sequence, 7U);
    EXPECT_DOUBLE_EQ(stored.confidence, 0.9);

    EXPECT_EQ(overlay_.tracked_count(), 1U);
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
}

TEST_F(DynamicOverlayTest, IdenticalReapplyIsIdempotent) {
    const EdgeDynamicState observation = blocked_observation(Tick{5000}, 7, 0.9);
    ASSERT_TRUE(overlay_.apply(fg_, observation));

    EXPECT_FALSE(overlay_.apply(fg_, observation));
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
    EXPECT_EQ(overlay_.tracked_count(), 1U);
}

TEST_F(DynamicOverlayTest, ChangedStateBumpsVersionAgain) {
    ASSERT_TRUE(overlay_.apply(fg_, blocked_observation(Tick{5000}, 7, 0.9)));
    EXPECT_TRUE(overlay_.apply(fg_, blocked_observation(Tick{6000}, 8, 0.95)));

    EXPECT_EQ(overlay_.version(), OverlayVersion{2});
}

TEST_F(DynamicOverlayTest, OpenObservationsAreTrackedToo) {
    // An explicit "this edge is open" observation must be stored as well:
    // reconciliation (later commit) needs it to clear stale BLOCKED states.
    const EdgeDynamicState open{
        .status = EdgeStatus::Open,
        .observed_at = Tick{200},
        .source = RobotId{2},
        .source_sequence = 1,
        .confidence = 0.4,
    };
    ASSERT_TRUE(overlay_.apply(ab_, open));

    ASSERT_NE(overlay_.find(ab_), nullptr);
    EXPECT_EQ(overlay_.find(ab_)->status, EdgeStatus::Open);
}

TEST_F(DynamicOverlayTest, TrackedEdgesAreSortedByEdgeId) {
    const EdgeId gh = *grid_.base.graph().edge_between(grid_.node("G"), grid_.node("H"));
    ASSERT_TRUE(overlay_.apply(fg_, blocked_observation(Tick{1}, 1, 1.0)));
    ASSERT_TRUE(overlay_.apply(ab_, blocked_observation(Tick{2}, 2, 1.0)));
    ASSERT_TRUE(overlay_.apply(gh, blocked_observation(Tick{3}, 3, 1.0)));

    EXPECT_EQ(overlay_.tracked_edges(), (std::vector<EdgeId>{ab_, fg_, gh}));
}

}  // namespace

#include "fleet/map/map_view.hpp"

#include <gtest/gtest.h>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::RobotId;
using fleet::common::SequenceNumber;
using fleet::common::Tick;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::map::MapView;
using fleet::testsupport::make_grid_map;

class MapViewTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    const EdgeId fg_{*grid_.base.graph().edge_between(grid_.node("F"), grid_.node("G"))};
    const EdgeId ef_{*grid_.base.graph().edge_between(grid_.node("E"), grid_.node("F"))};
    DynamicMapOverlay overlay_{grid_.base.graph().edge_count()};
};

TEST_F(MapViewTest, UntrackedEdgeExposesBaseCost) {
    const MapView view{grid_.base, overlay_};

    EXPECT_EQ(&view.base(), &grid_.base);
    EXPECT_EQ(&view.overlay(), &overlay_);
    EXPECT_FALSE(view.is_blocked(fg_));
    ASSERT_TRUE(view.traversal_cost(fg_).has_value());
    EXPECT_DOUBLE_EQ(*view.traversal_cost(fg_), 1.0);
    EXPECT_EQ(view.dynamic_state(fg_), nullptr);
}

TEST_F(MapViewTest, BlockedEdgeIsNotTraversable) {
    ASSERT_TRUE(overlay_.apply(fg_, EdgeDynamicState{
                                        .status = EdgeStatus::Blocked,
                                        .observed_at = Tick{5000},
                                        .source = RobotId{1},
                                        .source_sequence = SequenceNumber{1},
                                        .confidence = 0.9,
                                    }));
    const MapView view{grid_.base, overlay_};

    EXPECT_TRUE(view.is_blocked(fg_));
    EXPECT_FALSE(view.traversal_cost(fg_).has_value());

    // Isolation: only the blocked edge is affected.
    ASSERT_TRUE(view.traversal_cost(ef_).has_value());
    EXPECT_DOUBLE_EQ(*view.traversal_cost(ef_), 1.0);
}

TEST_F(MapViewTest, OpenObservationKeepsBaseCost) {
    ASSERT_TRUE(overlay_.apply(fg_, EdgeDynamicState{
                                        .status = EdgeStatus::Open,
                                        .observed_at = Tick{100},
                                        .source = RobotId{2},
                                        .source_sequence = SequenceNumber{3},
                                        .confidence = 0.5,
                                    }));
    const MapView view{grid_.base, overlay_};

    // v0 semantics: confidence does not scale cost (see map_view.hpp).
    EXPECT_FALSE(view.is_blocked(fg_));
    ASSERT_TRUE(view.traversal_cost(fg_).has_value());
    EXPECT_DOUBLE_EQ(*view.traversal_cost(fg_), 1.0);

    ASSERT_NE(view.dynamic_state(fg_), nullptr);
    EXPECT_DOUBLE_EQ(view.dynamic_state(fg_)->confidence, 0.5);
}

TEST_F(MapViewTest, AdjacencyDelegatesToBaseGraph) {
    const MapView view{grid_.base, overlay_};

    EXPECT_EQ(view.adjacency(grid_.node("A")).size(), 2U);
    EXPECT_EQ(view.adjacency(grid_.node("G")).size(), 4U);
}

}  // namespace

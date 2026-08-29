#include "fleet/map/base_map.hpp"

#include <gtest/gtest.h>

#include "fleet/common/ids.hpp"
#include "fleet/map/graph.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::MapVersion;
using fleet::testsupport::make_grid_map;

TEST(BaseMapTest, ExposesGraphAndVersion) {
    const auto grid = make_grid_map(MapVersion{7});

    EXPECT_EQ(grid.base.version(), MapVersion{7});
    EXPECT_EQ(grid.base.graph().node_count(), 12U);
    EXPECT_EQ(grid.base.graph().edge_count(), 17U);
}

TEST(BaseMapTest, NewRevisionMeansNewObject) {
    // Map revisions are new BaseMap objects, never in-place mutation
    // (ADR-001); two revisions coexist independently.
    const auto v1 = make_grid_map(MapVersion{1});
    const auto v2 = make_grid_map(MapVersion{2});

    EXPECT_NE(v1.base.version(), v2.base.version());
    EXPECT_EQ(v1.base.graph().node_count(), v2.base.graph().node_count());
}

}  // namespace

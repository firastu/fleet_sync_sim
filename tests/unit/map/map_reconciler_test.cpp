#include "fleet/map/map_reconciler.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"
#include "test_maps.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::OverlayVersion;
using fleet::common::RobotId;
using fleet::common::SequenceNumber;
using fleet::common::Tick;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::map::MapDelta;
using fleet::map::MapReconciler;
using fleet::map::MapView;
using fleet::map::ReconcileDecision;
using fleet::planning::AStarPlanner;
using fleet::planning::Route;
using fleet::testsupport::make_grid_map;

class MapReconcilerTest : public ::testing::Test {
protected:
    const fleet::testsupport::GridMap grid_{make_grid_map()};
    DynamicMapOverlay overlay_{grid_.base.graph().edge_count()};
    MapReconciler reconciler_{grid_.base.graph().edge_count()};
    const MapView view_{grid_.base, overlay_};

    [[nodiscard]] EdgeId edge_id(const char* a, const char* b) const {
        const std::optional<EdgeId> edge =
            grid_.base.graph().edge_between(grid_.node(a), grid_.node(b));
        EXPECT_TRUE(edge.has_value());
        return edge.value_or(EdgeId{});
    }

    [[nodiscard]] static MapDelta make_delta(EdgeId edge, RobotId source, SequenceNumber sequence,
                                             Tick observed_at, EdgeStatus status) {
        return MapDelta{
            edge,
            EdgeDynamicState{
                .status = status,
                .observed_at = observed_at,
                .source = source,
                .source_sequence = sequence,
                .confidence = 0.9,
            },
        };
    }
};

TEST_F(MapReconcilerTest, MapDeltaWrapsObservationWithoutDuplicatingIdentity) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta original =
        make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked);

    MapDelta copy = original;
    EXPECT_EQ(copy, original);  // value semantics

    copy.state.source_sequence = SequenceNumber{8};
    EXPECT_NE(copy, original);  // identity lives in the wrapped observation
}

TEST_F(MapReconcilerTest, AppliesFirstDelta) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta delta =
        make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked);

    EXPECT_EQ(reconciler_.reconcile(delta, overlay_), ReconcileDecision::Applied);

    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->status, EdgeStatus::Blocked);
    EXPECT_EQ(overlay_.find(edge)->source, RobotId{1});
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
    EXPECT_EQ(reconciler_.progression(RobotId{1}, edge), SequenceNumber{7});
}

TEST_F(MapReconcilerTest, DuplicateDeliveryIsIdempotent) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta delta =
        make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked);

    ASSERT_EQ(reconciler_.reconcile(delta, overlay_), ReconcileDecision::Applied);
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});

    EXPECT_EQ(reconciler_.reconcile(delta, overlay_), ReconcileDecision::Duplicate);
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
    EXPECT_EQ(overlay_.tracked_count(), 1U);
}

TEST_F(MapReconcilerTest, OlderSameSourceSameEdgeIsStaleWithoutRollback) {
    const EdgeId edge = edge_id("F", "G");
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{8}, Tick{5000}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);

    EXPECT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{4000}, EdgeStatus::Open),
                  overlay_),
              ReconcileDecision::Stale);

    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->status, EdgeStatus::Blocked);  // no rollback
    EXPECT_EQ(overlay_.find(edge)->source_sequence, SequenceNumber{8});
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
}

TEST_F(MapReconcilerTest, NewerSameSourceSameEdgeWins) {
    const EdgeId edge = edge_id("F", "G");
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);

    EXPECT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{8}, Tick{6000}, EdgeStatus::Open),
                  overlay_),
              ReconcileDecision::Applied);

    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->status, EdgeStatus::Open);
    EXPECT_EQ(overlay_.find(edge)->source_sequence, SequenceNumber{8});
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});
}

TEST_F(MapReconcilerTest, ReorderedDifferentEdgesAreNotLost) {
    // The ordering model is per (source, edge): a late, lower-sequence
    // delta for a DIFFERENT edge is valid information and must not be
    // discarded by any global per-source high-water mark (ADR-004).
    const EdgeId x = edge_id("F", "G");
    const EdgeId y = edge_id("K", "L");

    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(x, RobotId{1}, SequenceNumber{20}, Tick{5000}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_.reconcile(
                  make_delta(y, RobotId{1}, SequenceNumber{19}, Tick{4990}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);

    ASSERT_NE(overlay_.find(x), nullptr);
    ASSERT_NE(overlay_.find(y), nullptr);
    EXPECT_EQ(overlay_.find(y)->source_sequence, SequenceNumber{19});
    EXPECT_EQ(overlay_.tracked_count(), 2U);
    EXPECT_EQ(reconciler_.progression(RobotId{1}, x), SequenceNumber{20});
    EXPECT_EQ(reconciler_.progression(RobotId{1}, y), SequenceNumber{19});
    EXPECT_EQ(reconciler_.progression(RobotId{2}, x), SequenceNumber{0});
}

TEST_F(MapReconcilerTest, SameIdentityDifferentPayloadIsRejectedConflict) {
    const EdgeId edge = edge_id("F", "G");
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);

    const MapDelta conflicting =
        make_delta(edge, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Open);

    EXPECT_EQ(reconciler_.reconcile(conflicting, overlay_), ReconcileDecision::RejectedConflict);
    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->status, EdgeStatus::Blocked);  // unchanged
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
}

TEST_F(MapReconcilerTest, NewerObservationFromOtherSourceWins) {
    const EdgeId edge = edge_id("F", "G");
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{5}, Tick{5000}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);

    EXPECT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{2}, SequenceNumber{3}, Tick{6000}, EdgeStatus::Open),
                  overlay_),
              ReconcileDecision::Applied);
    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->status, EdgeStatus::Open);
    EXPECT_EQ(overlay_.find(edge)->source, RobotId{2});

    // A newer observation from a third source wins; an older one is a new
    // valid event that is dominated by the better knowledge.
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{3}, SequenceNumber{1}, Tick{7000}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{4}, SequenceNumber{1}, Tick{6500}, EdgeStatus::Open),
                  overlay_),
              ReconcileDecision::Dominated);
    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->source, RobotId{3});
}

TEST_F(MapReconcilerTest, SameTickCrossSourceConflictIsDeterministicAndConvergent) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta a =
        make_delta(edge, RobotId{1}, SequenceNumber{4}, Tick{5000}, EdgeStatus::Blocked);
    const MapDelta b =
        make_delta(edge, RobotId{2}, SequenceNumber{9}, Tick{5000}, EdgeStatus::Open);

    // Arrival order 1: A then B. Observation-time tie, BLOCKED dominates:
    // B's OPEN is a new valid event that is dominated by A's knowledge.
    DynamicMapOverlay overlay_first{grid_.base.graph().edge_count()};
    MapReconciler reconciler_first{grid_.base.graph().edge_count()};
    ASSERT_EQ(reconciler_first.reconcile(a, overlay_first), ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_first.reconcile(b, overlay_first), ReconcileDecision::Dominated);

    // Arrival order 2: B then A. Same tie, same winner: A re-blocks.
    DynamicMapOverlay overlay_second{grid_.base.graph().edge_count()};
    MapReconciler reconciler_second{grid_.base.graph().edge_count()};
    ASSERT_EQ(reconciler_second.reconcile(b, overlay_second), ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_second.reconcile(a, overlay_second), ReconcileDecision::Applied);

    // Convergence: both arrival orders end with identical knowledge.
    ASSERT_NE(overlay_first.find(edge), nullptr);
    ASSERT_NE(overlay_second.find(edge), nullptr);
    EXPECT_EQ(*overlay_first.find(edge), *overlay_second.find(edge));
    EXPECT_EQ(overlay_first.find(edge)->status, EdgeStatus::Blocked);
}

TEST_F(MapReconcilerTest, SameTickSameStatusTieResolvesToHigherSourceProvenance) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta a =
        make_delta(edge, RobotId{1}, SequenceNumber{4}, Tick{5000}, EdgeStatus::Blocked);
    const MapDelta b =
        make_delta(edge, RobotId{2}, SequenceNumber{9}, Tick{5000}, EdgeStatus::Blocked);

    DynamicMapOverlay overlay_first{grid_.base.graph().edge_count()};
    MapReconciler reconciler_first{grid_.base.graph().edge_count()};
    ASSERT_EQ(reconciler_first.reconcile(a, overlay_first), ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_first.reconcile(b, overlay_first), ReconcileDecision::Applied);

    DynamicMapOverlay overlay_second{grid_.base.graph().edge_count()};
    MapReconciler reconciler_second{grid_.base.graph().edge_count()};
    ASSERT_EQ(reconciler_second.reconcile(b, overlay_second), ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_second.reconcile(a, overlay_second), ReconcileDecision::Dominated);

    // Both orders converge on B's provenance (higher source id).
    ASSERT_NE(overlay_first.find(edge), nullptr);
    ASSERT_NE(overlay_second.find(edge), nullptr);
    EXPECT_EQ(*overlay_first.find(edge), *overlay_second.find(edge));
    EXPECT_EQ(overlay_first.find(edge)->source, RobotId{2});
}

TEST_F(MapReconcilerTest, OverlayVersionTracksKnowledgeNotAttempts) {
    const EdgeId edge = edge_id("F", "G");
    EXPECT_EQ(overlay_.version(), OverlayVersion{0});

    ASSERT_EQ(reconciler_.reconcile(make_delta(edge, RobotId{1}, SequenceNumber{10}, Tick{5000},
                                               EdgeStatus::Blocked),
                                    overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});

    EXPECT_EQ(reconciler_.reconcile(make_delta(edge, RobotId{1}, SequenceNumber{10}, Tick{5000},
                                               EdgeStatus::Blocked),
                                    overlay_),
              ReconcileDecision::Duplicate);
    EXPECT_EQ(reconciler_.reconcile(make_delta(edge, RobotId{1}, SequenceNumber{9}, Tick{4990},
                                               EdgeStatus::Open),
                                    overlay_),
              ReconcileDecision::Stale);
    EXPECT_EQ(reconciler_.reconcile(make_delta(edge, RobotId{1}, SequenceNumber{10}, Tick{5000},
                                               EdgeStatus::Open),
                                    overlay_),
              ReconcileDecision::RejectedConflict);
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});

    // Fresher knowledge with identical traversability still counts as a
    // meaningful overlay change: provenance and observation time are part
    // of the stored state (ADR-004).
    EXPECT_EQ(reconciler_.reconcile(make_delta(edge, RobotId{1}, SequenceNumber{11}, Tick{5100},
                                               EdgeStatus::Blocked),
                                    overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});
}

TEST_F(MapReconcilerTest, DuplicateAfterSupersessionByOtherSourceIsANoOp) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta first =
        make_delta(edge, RobotId{1}, SequenceNumber{5}, Tick{5000}, EdgeStatus::Blocked);
    ASSERT_EQ(reconciler_.reconcile(first, overlay_), ReconcileDecision::Applied);
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{2}, SequenceNumber{2}, Tick{6000}, EdgeStatus::Open),
                  overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});

    // Re-delivery of A:5: the latest processed event for (A, edge) is
    // retained with its exact payload even though B owns the overlay
    // slot, so identity is fully verifiable.
    EXPECT_EQ(reconciler_.reconcile(first, overlay_), ReconcileDecision::Duplicate);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});
    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->source, RobotId{2});
}

TEST_F(MapReconcilerTest, PlannerSeesReconciledState) {
    const AStarPlanner planner;
    const auto start = grid_.node("A");
    const auto goal = grid_.node("D");

    const Route initial = planner.plan(view_, start, goal);
    ASSERT_TRUE(initial.found);
    ASSERT_TRUE(initial.uses_edge(edge_id("B", "C")));

    // A received delta blocks an edge on the initial route; reconciliation
    // makes it part of this participant's knowledge before replanning.
    const MapDelta received = make_delta(edge_id("B", "C"), RobotId{1}, SequenceNumber{7},
                                         Tick{5000}, EdgeStatus::Blocked);
    EXPECT_EQ(reconciler_.reconcile(received, overlay_), ReconcileDecision::Applied);

    const Route rerouted = planner.plan(view_, start, goal);
    ASSERT_TRUE(rerouted.found);
    EXPECT_FALSE(rerouted.uses_edge(edge_id("B", "C")));
    EXPECT_DOUBLE_EQ(rerouted.cost, 5.0);  // canonical detour (ADR-003)
}

TEST_F(MapReconcilerTest, DominatedEventStillAdvancesSourceProgression) {
    const EdgeId edge = edge_id("F", "G");
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{10}, Tick{10}, EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);
    ASSERT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{2}, SequenceNumber{7}, Tick{20}, EdgeStatus::Open),
                  overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});

    // New valid A event with an older observation than the winner's:
    // processed (progression advances) but B holds better knowledge.
    const MapDelta dominated =
        make_delta(edge, RobotId{1}, SequenceNumber{11}, Tick{15}, EdgeStatus::Blocked);
    EXPECT_EQ(reconciler_.reconcile(dominated, overlay_), ReconcileDecision::Dominated);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});  // winner unchanged
    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->source, RobotId{2});
    EXPECT_EQ(overlay_.find(edge)->status, EdgeStatus::Open);
    EXPECT_EQ(reconciler_.progression(RobotId{1}, edge), SequenceNumber{11});

    // Exact re-delivery of the dominated event: verifiable against the
    // retained progression payload, independent of the overlay winner.
    EXPECT_EQ(reconciler_.reconcile(dominated, overlay_), ReconcileDecision::Duplicate);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});

    // Same identity, different payload: integrity violation.
    const MapDelta conflicting =
        make_delta(edge, RobotId{1}, SequenceNumber{11}, Tick{15}, EdgeStatus::Open);
    EXPECT_EQ(reconciler_.reconcile(conflicting, overlay_), ReconcileDecision::RejectedConflict);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});
    ASSERT_NE(overlay_.find(edge), nullptr);
    EXPECT_EQ(overlay_.find(edge)->source, RobotId{2});
}

TEST_F(MapReconcilerTest, CrossSourceConvergenceAcrossPermutations) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta a =
        make_delta(edge, RobotId{1}, SequenceNumber{1}, Tick{10}, EdgeStatus::Blocked);
    const MapDelta b =
        make_delta(edge, RobotId{2}, SequenceNumber{1}, Tick{20}, EdgeStatus::Open);
    const MapDelta c =
        make_delta(edge, RobotId{3}, SequenceNumber{1}, Tick{20}, EdgeStatus::Blocked);

    const std::array<const MapDelta*, 3> permutations[6] = {
        {&a, &b, &c}, {&a, &c, &b}, {&b, &a, &c},
        {&b, &c, &a}, {&c, &a, &b}, {&c, &b, &a},
    };

    for (const auto& order : permutations) {
        DynamicMapOverlay overlay{grid_.base.graph().edge_count()};
        MapReconciler reconciler{grid_.base.graph().edge_count()};
        for (const MapDelta* delta : order) {
            reconciler.reconcile(*delta, overlay);
        }
        const EdgeDynamicState* state = overlay.find(edge);
        ASSERT_NE(state, nullptr);
        // All six arrival orders converge on C's exact knowledge:
        // tick 20 beats A's tick 10; the tick tie against B resolves to
        // BLOCKED; provenance identical. (Overlay *versions* may differ
        // legitimately: they count winner changes, which depend on order.)
        EXPECT_EQ(*state, c.state);
    }
}

TEST_F(MapReconcilerTest, SequenceZeroIsNeverAValidEvent) {
    const EdgeId edge = edge_id("F", "G");
    const MapDelta zero =
        make_delta(edge, RobotId{1}, SequenceNumber{0}, Tick{5000}, EdgeStatus::Blocked);

    EXPECT_EQ(reconciler_.reconcile(zero, overlay_), ReconcileDecision::Stale);
    EXPECT_EQ(overlay_.version(), OverlayVersion{0});
    EXPECT_EQ(overlay_.tracked_count(), 0U);
    EXPECT_EQ(reconciler_.progression(RobotId{1}, edge), SequenceNumber{0});

    // The reserved value does not poison the stream: a valid event still
    // applies normally.
    EXPECT_EQ(reconciler_.reconcile(
                  make_delta(edge, RobotId{1}, SequenceNumber{1}, Tick{5000},
                             EdgeStatus::Blocked),
                  overlay_),
              ReconcileDecision::Applied);
    EXPECT_EQ(overlay_.version(), OverlayVersion{1});
}

TEST_F(MapReconcilerTest, SameSequenceOnDifferentEdgesIsIndependent) {
    // Event identity is (source, edge, sequence): equal sequence numbers
    // on different edges are independent events and must not be treated
    // as one identity merely because their numeric sequences match.
    const EdgeId x = edge_id("F", "G");
    const EdgeId y = edge_id("K", "L");
    const MapDelta on_x =
        make_delta(x, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked);
    const MapDelta on_y =
        make_delta(y, RobotId{1}, SequenceNumber{7}, Tick{5000}, EdgeStatus::Blocked);

    EXPECT_EQ(reconciler_.reconcile(on_x, overlay_), ReconcileDecision::Applied);
    EXPECT_EQ(reconciler_.reconcile(on_y, overlay_), ReconcileDecision::Applied);

    // Both streams progressed independently; replaying either exact delta
    // is a Duplicate in its own stream.
    EXPECT_EQ(reconciler_.reconcile(on_x, overlay_), ReconcileDecision::Duplicate);
    EXPECT_EQ(reconciler_.reconcile(on_y, overlay_), ReconcileDecision::Duplicate);
    EXPECT_EQ(overlay_.tracked_count(), 2U);
    EXPECT_EQ(overlay_.version(), OverlayVersion{2});
    EXPECT_EQ(reconciler_.progression(RobotId{1}, x), SequenceNumber{7});
    EXPECT_EQ(reconciler_.progression(RobotId{1}, y), SequenceNumber{7});
}

}  // namespace

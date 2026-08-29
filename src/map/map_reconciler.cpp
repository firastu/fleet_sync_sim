#include "fleet/map/map_reconciler.hpp"

#include <cassert>

namespace fleet::map {

namespace {

// Cross-source total order (ADR-004): the candidate replaces the stored
// state when it is a newer observation; on an observation-time tie, BLOCKED
// dominates OPEN (safety bias); on a status tie, the higher source id wins
// so that independent arrival orders converge to identical state and
// provenance.
[[nodiscard]] bool supersedes(const EdgeDynamicState& candidate, const EdgeDynamicState& stored) {
    if (candidate.observed_at != stored.observed_at) {
        return candidate.observed_at > stored.observed_at;
    }
    if (candidate.status != stored.status) {
        return candidate.status == EdgeStatus::Blocked;
    }
    return candidate.source > stored.source;
}

}  // namespace

MapReconciler::MapReconciler(std::size_t edge_count) : edge_count_{edge_count} {}

ReconcileDecision MapReconciler::reconcile(const MapDelta& delta, DynamicMapOverlay& overlay) {
    assert(delta.edge.value() < edge_count_);

    // Sequences start at 1; zero is reserved ("nothing processed") and can
    // never denote a valid event. Defined release behavior: Stale, no
    // state touched.
    if (delta.state.source_sequence < common::SequenceNumber{1}) {
        return ReconcileDecision::Stale;
    }

    const auto progress = progression_.find(SourceEdgeKey{delta.state.source, delta.edge});

    if (progress != progression_.end() &&
        delta.state.source_sequence < progress->second.sequence) {
        // Older than the latest processed event of this source's stream;
        // the stream never rolls backward.
        return ReconcileDecision::Stale;
    }
    if (progress != progression_.end() &&
        delta.state.source_sequence == progress->second.sequence) {
        // Re-delivery of the latest processed event for this (source,
        // edge). The payload is always verifiable against the retained
        // progression record, regardless of who owns the overlay slot.
        return progress->second.state == delta.state ? ReconcileDecision::Duplicate
                                                     : ReconcileDecision::RejectedConflict;
    }

    // A new event in this source's stream for this edge. Source
    // progression is recorded whether or not the event becomes the
    // overlay winner.
    const EdgeDynamicState* stored = overlay.find(delta.edge);
    ReconcileDecision decision = ReconcileDecision::Applied;
    if (stored != nullptr) {
        if (stored->source == delta.state.source) {
            // The source's own strictly newer word in this (source, edge)
            // stream. Producer contract (Stage 0): within one stream, a
            // higher sequence does not move observed_at backward; release
            // builds still apply a violating event.
            assert(delta.state.observed_at >= stored->observed_at);
        } else if (!supersedes(delta.state, *stored)) {
            decision = ReconcileDecision::Dominated;
        }
    }

    if (decision == ReconcileDecision::Applied) {
        // Mechanism layer: stores the state and bumps the overlay version.
        // On the accepted path the state necessarily differs from what is
        // stored (sequence, source or observation time changed), so a no-op
        // return would violate the reconciliation invariant. The result is
        // captured outside assert() on purpose: assert() does not evaluate
        // its argument (or call apply) in NDEBUG builds.
        [[maybe_unused]] const bool changed = overlay.apply(delta.edge, delta.state);
        assert(changed && "MapReconciler: accepted delta must change the stored state");
    }
    progression_[SourceEdgeKey{delta.state.source, delta.edge}] =
        SourceProgress{delta.state.source_sequence, delta.state};
    return decision;
}

common::SequenceNumber MapReconciler::progression(common::RobotId source,
                                                  common::EdgeId edge) const noexcept {
    assert(edge.value() < edge_count_);
    const auto progress = progression_.find(SourceEdgeKey{source, edge});
    return progress != progression_.end() ? progress->second.sequence
                                          : common::SequenceNumber{0};
}

}  // namespace fleet::map

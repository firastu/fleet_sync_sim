#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_delta.hpp"

namespace fleet::map {

// Outcome of offering one MapDelta to a participant's local knowledge.
//
//   Applied          — new valid event; it became the overlay winner;
//   Duplicate        — an identical latest event for this (source, edge)
//                      was already processed;
//   Stale            — the incoming sequence is older than the latest
//                      processed event for this (source, edge), or is the
//                      reserved zero sequence (never a valid event);
//   Dominated        — new valid event; its source progression is
//                      recorded, but another source currently holds
//                      better knowledge, so the overlay is unchanged;
//   RejectedConflict — same event identity as the latest processed event
//                      for this (source, edge), but a different payload
//                      (protocol/data-integrity violation).
enum class ReconcileDecision : std::uint8_t {
    Applied,
    Duplicate,
    Stale,
    Dominated,
    RejectedConflict,
};

// Deterministic reconciliation policy for one participant's overlay
// (ADR-004). Decides whether a MapDelta — locally produced or received —
// updates that overlay. It never touches any BaseMap (ADR-001) and has no
// transport, retry or timing concerns; those belong to later layers.
//
// Two deliberately separate states:
//
//   SOURCE PROGRESSION — what has been processed from each source for
//   each edge: the latest sequence *and its exact payload*, one sparse
//   record per observed (source, edge);
//
//   OVERLAY WINNER — the single observation that currently represents
//   best knowledge of the edge.
//
// A new valid event can be processed successfully (progression advances)
// yet not become the winner: that outcome is Dominated, not Stale —
// "processed this distributed event" and "this event defines my map
// truth" are different facts.
//
// Ordering model — per (source, edge), deliberately NOT a global
// per-source high-water mark: a global mark would discard a valid late
// delta for a *different* edge when messages are reordered (the future
// network model explicitly reorders).
//
//   same source, same edge:
//     sequence <  latest processed  -> Stale (never roll the source's
//                                       stream backward);
//     sequence == latest processed  -> Duplicate, or RejectedConflict if
//                                       the payload differs (always
//                                       verifiable against the retained
//                                       progression payload, regardless
//                                       of who owns the overlay slot);
//     sequence >  latest processed  -> the source's own latest word: it
//                                       replaces an own-source winner,
//                                       or competes cross-source.
//
//   cross-source competition, resolved by a total order:
//     1. newer observed_at wins;
//     2. tie: BLOCKED dominates OPEN (safety bias — an exact-tie OPEN
//        cannot clear a block; a *newer* OPEN still can);
//     3. tie: higher source id wins — provenance converges to the same
//        state independent of arrival order.
//   A new event that loses this comparison is Dominated: progression is
//   recorded, the overlay is unchanged.
//   Confidence is stored but deliberately unused by Stage-0 policy.
//
// Sequence zero is reserved ("nothing processed"): an incoming
// SequenceNumber{0} delta is classified Stale in release builds — defined
// behavior, never a valid event, no state touched.
//
// Same-source chronology (Stage-0 producer contract): within one
// (source, edge) progression stream, a higher sequence should not carry
// an older observed_at; debug-asserted only. Release builds still apply
// a violating event. No temporal relationship is required across a
// source's different edge streams.
//
// Locally produced observations are expected to go through the same
// reconcile() path so local and remote knowledge share one set of
// ordering semantics; DynamicMapOverlay::apply() remains the low-level
// storage primitive beneath this layer.
//
// Event identity: (source, edge, source_sequence) — one independently
// ordered event stream per (source, edge); equal sequence numbers on
// different edges are independent events. A producer MAY use one
// source-global sequence counter (a stronger policy); MapReconciler must
// not depend on it, for the same reason it rejects a global per-source
// high-water mark. Stage 0 assumes participant identities and their
// streams are stable for one execution; restart/session epochs are
// deferred (ADR-004, Limitations).
//
// Complexity: O(1) expected per delta (exact-key lookups; no overlay
// scans or copies). Storage is sparse: one progression record per
// (source, edge) actually observed. The hash map is used ONLY for exact
// lookup/insert/update and is never iterated, so hash iteration order
// cannot affect any decision (determinism per ADR-002). A dense
// participants x edges table is rejected: 256 RobotId values x 200,000
// edges x ~40 bytes would be gigabytes per reconciler, while realistic
// fleets observe a small subset of edges.
//
// Thread-safety: not synchronized (ADR-002).
class MapReconciler {
public:
    explicit MapReconciler(std::size_t edge_count);

    // Offers `delta` to `overlay` (the receiving participant's local
    // dynamic knowledge) and applies it when policy accepts it as the new
    // winner. Idempotent: re-deliveries and dominated events never change
    // the overlay or bump its version (source progression always
    // advances for new valid events).
    //
    // Precondition: delta.edge.value() < edge_count. Sequence validity is
    // handled at runtime: SequenceNumber{0} is classified Stale.
    ReconcileDecision reconcile(const MapDelta& delta, DynamicMapOverlay& overlay);

    // Latest sequence from `source` processed for `edge`;
    // SequenceNumber{0} means nothing processed yet. Test/observability
    // accessor; reconcile() is the only mutating entry point.
    [[nodiscard]] common::SequenceNumber progression(common::RobotId source,
                                                     common::EdgeId edge) const noexcept;

private:
    // Exact-lookup key for sparse progression storage.
    struct SourceEdgeKey {
        common::RobotId source{};
        common::EdgeId edge{};
        bool operator==(const SourceEdgeKey&) const = default;
    };

    struct SourceEdgeKeyHash {
        std::size_t operator()(const SourceEdgeKey& key) const noexcept {
            return (static_cast<std::size_t>(key.source.value()) << 32) |
                   static_cast<std::size_t>(key.edge.value());
        }
    };

    // The latest event processed for one (source, edge) — sequence plus
    // exact payload — kept separate from the overlay winner.
    struct SourceProgress {
        common::SequenceNumber sequence{};
        EdgeDynamicState state{};
    };

    // progression_[{source, edge}] — sparse, exact lookup/insert/update
    // only, never iterated (see class comment).
    std::unordered_map<SourceEdgeKey, SourceProgress, SourceEdgeKeyHash> progression_;
    std::size_t edge_count_;
};

}  // namespace fleet::map

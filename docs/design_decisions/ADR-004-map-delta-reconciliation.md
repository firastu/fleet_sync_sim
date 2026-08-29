# ADR-004: Sequenced MapDelta reconciliation

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::map` (MapDelta, MapReconciler); deliberately transport-independent

## Context

Participants exchange dynamic map knowledge over links that will delay,
duplicate and reorder messages, and two participants may observe the same
edge differently at the same logical time. Before any network exists, the
project needs one deterministic answer to: *given local knowledge and a
MapDelta, should it be applied, ignored as duplicate/stale, or rejected as
an integrity violation?* — with idempotence (re-delivery must never change
state or bump the overlay version) and no reliance on wall-clock time
(ADR-002).

## Decision

1. **Event representation.** `MapDelta = { EdgeId, EdgeDynamicState }`.
   Identity and ordering fields (source, source sequence, observation
   tick, confidence) live *inside* the wrapped observation, never
   duplicated beside it — a delta and its payload cannot disagree.
   Transport metadata (delivery time, hop count, message id) belongs to a
   future message envelope, not to MapDelta.
2. **Event identity.** `(source, edge, source_sequence)` — one
   independently ordered event stream per (source, edge); equal sequence
   numbers on different edges are independent events. Sequences start at
   1; the value 0 is reserved ("nothing processed"). A producer MAY
   allocate sequences from one source-global counter, but reconciliation
   must not depend on it — for the same reason a global per-source
   high-water mark was rejected.
3. **Same-source ordering — per-(source, edge) progression records.**
   Deliberately *not* a global per-source high-water mark: under message
   reordering a global mark would discard a valid late delta for a
   *different* edge (seq=20/edge X, then late seq=19/edge Y must not lose
   Y). The reconciler retains, per (source, edge), the *latest processed
   event* — sequence **and exact payload** — as state that is separate
   from the overlay winner. Rules: sequence < latest → Stale;
   == latest → Duplicate, or RejectedConflict when the payload differs
   (always verifiable, independent of who owns the overlay slot);
   > latest → the source's own latest word: it replaces an own-source
   winner, or competes cross-source — and if it loses there, the outcome
   is **Dominated**: the progression advances, the overlay is unchanged.
   "Processed this distributed event" and "this event defines my map
   truth" are different facts and get different decisions.
4. **Cross-source conflict — total order.** Newer `observed_at` wins; on a
   tie, BLOCKED dominates OPEN (safety bias); on a status tie, the higher
   source id wins. The total order guarantees both participants converge
   to identical state — including provenance — independent of arrival
   order.
5. **Idempotence and overlay version.** Only accepted knowledge changes
   state; Duplicate/Stale/Dominated/RejectedConflict never touch the
   overlay or its version. An accepted delta that refreshes provenance
   while traversability is unchanged still bumps the version: provenance
   and observation time are part of the stored knowledge, and the bump is
   what lets planners detect staleness of their routes.
6. **Single update path.** Locally produced observations use the same
   `reconcile()` path as received deltas; `DynamicMapOverlay::apply()`
   remains the low-level storage primitive.
7. **Progression storage.** Sparse exact-lookup map keyed by
   (source, edge): O(observed pairs), not O(participants x edges). Used
   only for exact lookup/insert/update and never iterated, so hash
   iteration order cannot affect any decision (determinism per ADR-002).
   A dense 256-RobotId x 200,000-edge matrix of ~40-byte records would
   be multiple gigabytes per reconciler; realistic fleets observe only a
   small subset of edges.
8. **Sequence zero.** Reserved ("nothing processed"). An incoming
   SequenceNumber{0} delta is classified Stale in release builds —
   defined behavior, never a valid event, no state touched.

## Alternatives considered

- **Global per-source high-water mark.** Rejected: loses valid late
  deltas for other edges under reordering (the future network model
  explicitly reorders).
- **Dense participants x edges progression matrix.** Rejected: memory
  scales with map size x fleet size even though almost no robot observes
  almost any given edge (gigabytes at road-network scale, per
  reconciler).
- **Last-arrival-wins.** Rejected: nondeterministic knowledge, no
  convergence guarantee across arrival orders.
- **Latest wall-clock timestamp wins.** Rejected: wall clock is banned
  from behavior (ADR-002), and arrival time is not observation time.
- **Lamport clocks now.** Postponed: they order causally related events,
  but our ambiguity is *conflicting world observations*, which is resolved
  by the recency+safety total order above. No causal-chain consumer exists
  yet; revisit when multi-hop forwarding/relaying (roadmap stages 4–5)
  introduces causal-ordering questions.
- **Vector clocks / CRDT.** Premature: they encode a merge semantics we
  have not yet needed to distribute.

## Consequences and limitations

- Only the *latest* processed payload per (source, edge) is retained.
  Re-deliveries of older-than-latest sequences are classified Stale
  without payload verification; retaining full history would be a
  distributed event database, which is explicitly out of scope.
- Restart/session identity is deferred: participant ids and sequence
  counters are assumed stable for one deterministic execution. If robot
  restarts are simulated later, source epochs must be added to event
  identity.
- Chronology is a Stage-0 producer contract scoped to one (source, edge)
  stream: a higher sequence should not carry an older observed_at
  (debug-asserted only; release builds still apply a violating event).
  No temporal relationship is required across a source's different edge
  streams.
- Confidence is recorded but unused by Stage-0 policy. Clearing-age
  thresholds and confidence weighting are future policy knobs, not
  current behavior. The safety bias is shallow by design: BLOCKED only
  dominates an exact observation-time tie; a *newer* OPEN clears it.
- O(1) expected per delta; no overlay copies, no scans. The progression
  map is a hash container used strictly for exact-key operations and
  never iterated, so no iteration order can leak into decisions.

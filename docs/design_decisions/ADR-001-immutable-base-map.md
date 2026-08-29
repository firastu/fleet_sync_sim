# ADR-001: Immutable base map with per-participant dynamic overlays

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::map`

## Context

Every robot — and the control station — must reason about the same road
network while simultaneously reacting to dynamic changes (blocked edges,
degraded traversal). Candidate designs:

1. one shared mutable map that every component locks and updates;
2. full map copies per participant, merged when they meet;
3. one immutable base map plus a small dynamic overlay per participant.

The concurrency stages (1–3) will introduce threads. A single shared mutable
map would make every reader/writer interaction a synchronization point, and
would make deterministic replay nearly impossible because final state would
depend on interleaving.

## Decision

The base topology (`BaseMap` wrapping a `Graph`) is immutable after
construction and shared by const reference. All dynamic knowledge lives in a
per-participant `DynamicMapOverlay`: a dense table of `EdgeDynamicState`
records indexed by `EdgeId`. Consumers plan against a `MapView`, which
composes exactly one base map with one overlay at read time.

Conflicts between overlays are a reconciliation problem, not a map-ownership
problem; a reconciliation record will be written when that decision actually
happens.

## Alternatives considered

- **Single shared mutable map.** Rejected: couples all participants to one
  lock domain, erodes robot autonomy (the station becomes a de facto hard
  dependency), and rules out deterministic scenario replay.
- **Copy-on-write full-map snapshots.** Premature at Stage 0: the overlay is
  orders of magnitude smaller than a full map copy on realistic road
  networks. Revisit at Stage 3 *if measurements* show overlay lookup or view
  composition on hot paths (a `shared_ptr<const Overlay>` publication model
  is the likely successor).
- **CRDT-backed map.** Premature: a CRDT bakes in merge semantics before we
  have operational experience with reconciliation policy on a simple store.

## Consequences

- Base-map handoff between components is trivially safe: no locks, ever.
- Overlay memory is O(E) per participant even when nearly empty; a
  hash map over tracked edges only would be O(tracked). At Milestone 1 scale
  (hundreds to a few thousand edges) the dense vector wins on simplicity and
  cache behavior. Revisit if fleet size × map size makes this material.
- "The map changed" is explicit and cheap to observe: the overlay version
  bumps on every applied change, which later gives planners a trivial
  snapshot-invalidation check.
- A new road-network revision is a *new* `BaseMap` object with a new
  `MapVersion`, never in-place mutation.

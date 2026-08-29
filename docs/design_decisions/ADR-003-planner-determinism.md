# ADR-003: Deterministic planner tie-breaking contract

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::planning` (and any future planner implementation)

## Context

Equal-cost shortest paths are the normal case on grid-like road networks,
not an edge case. If planner output depends on heap internals, container
iteration order or push sequence, simulation traces are not reproducible,
and later stages — which must be validated against the Stage 0 oracle
(ADR-002) — cannot be compared with it.

The planner therefore needs a *canonical route*: a documented rule that
selects exactly one route among all optimal routes, independent of
platform, standard library or incidental code paths.

Related postponement: there is no `IPlanner` abstraction yet — one
implementation, no polymorphic consumer. The real boundary is the
`MapView`; revisit when a second algorithm lands.

## Decision

`AStarPlanner` produces the canonical route through three rules:

1. **Open-list pop order**: lowest `f`, then lowest `g`, then lowest
   `NodeId` — a strict total order on queue entries. Identical entries
   cannot occur because entries are pushed only on strict improvement.
2. **Predecessor updates only on strict cost improvement**
   (`candidate_g < g[v]`): among equal-cost predecessors, the first one
   found wins.
3. **Neighbor relaxation in the graph's CSR (edge-insertion) order**,
   which `Graph` already guarantees.

With the default zero heuristic this is exactly Dijkstra in `(g, NodeId)`
pop order. Floating-point costs are compared exactly — no epsilon: exact
comparison is what makes ties well-defined, while an epsilon would make
"improvement" order-dependent and break both reproducibility and the
`cost == exact sum of edge costs` invariant. Costs accumulate along the
reconstructed path in a fixed order, so route cost is bit-stable.

Any future planner (D* Lite, ARA*, ...) answering the same query must
either reproduce this canonical route or document its own contract and
the divergence — logged traces are only comparable if routes are.

## Alternatives considered

- **Rely on `std::priority_queue` tie behavior / unordered containers.**
  Rejected: unspecified or implementation-defined; traces would differ
  across standard libraries and runs.
- **Lexicographically smallest optimal route.** A cleaner mathematical
  contract, but it requires path comparison during relaxation (extra work
  per edge) and no consumer needs it today. Revisit if route *identity*
  becomes part of cross-robot agreement semantics.
- **Epsilon cost comparisons.** Rejected: see Decision.

## Consequences

- Specific canonical routes are locked by unit tests; changing the rule is
  a visible, reviewable breaking change.
- The canonical route is "the first optimal route under the pop order",
  which is not necessarily the lexicographically smallest one.
- Heuristic choice can change which optimal route is canonical (different
  expansion order) but never its cost; the zero default defines the
  baseline used across the simulator.

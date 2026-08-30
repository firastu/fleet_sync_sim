# ADR-013: One-way edges in topology

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::map` (Edge, Graph::Builder), `fleet::map::MapView`
  (traversable_from), `fleet::planning` (A*), `fleet::robot` (defensive
  departure check); prerequisite for the OSM importer (#12A.2)

## Context

Every edge so far has been bidirectional — correct for the synthetic
grid world. The OSM importer must map real one-way streets
(`oneway=yes` / `oneway=-1`) to directed traversability; silently
importing them as bidirectional would produce routes that drive the
wrong way down streets. Directionality is therefore a TOPOLOGY
property, decided before any importer exists.

## Decision

- `EdgeDirection { Bidirectional, Forward, Reverse }` on `Edge`.
  `Bidirectional` is the DEFAULT and preserves every existing map,
  fixture and trace byte-for-byte. `Forward` = traversable only from
  `a` to `b`; `Reverse` = only from `b` to `a`. Endpoint order becomes
  meaningful for one-way edges (and is meaningless otherwise).
- Direction affects TRAVERSABILITY ONLY: cost, knowledge, deltas,
  reconciliation, sensing and blocking semantics are direction-blind.
  A blocked one-way edge is blocked; an open one is open in its
  direction(s).
- `MapView::traversable_from(edge, from)` is the single decision point:
  false when blocked, when `from` is not an endpoint, or when the
  one-way direction opposes the move. The planner checks it while
  relaxing adjacency (the CSR adjacency lists both endpoints of every
  edge; direction is a per-move question, not a graph-structure
  question). `Robot::begin_transit` re-checks defensively (routes are
  planned on the same knowledge, so the check cannot fire in practice).
- Route invariant extended: no route enters an edge against its
  direction.
- `Graph::Builder::connect` gains a trailing `direction` parameter
  (defaulted); duplicate-edge detection still allows at most ONE edge
  per node pair — a pair must not be represented as two anti-parallel
  one-way edges (callers model that as one Bidirectional edge, or it is
  genuinely two different roads, which is a data question the importer
  will surface).

## Alternatives considered

- **Importer-level direction flags outside topology.** Rejected: every
  consumer (planner, movement, future map matching) would need to know
  the importer's side table; direction is a property of the road, not
  of the import format.
- **Directed dual-graph (two directed half-edges per road).** Rejected:
  doubles the edge universe for the 99% bidirectional case, complicates
  reconciliation identity (one road = one EdgeId is load-bearing in
  ADR-004), and offers no capability this design lacks.
- **Keep topology undirected; let the importer drop oneway ways.**
  Rejected: real maps are full of one-ways; dropping them silently
  distorts topology worse than modeling them.
- **Direction in MapGeometry.** Rejected: geometry is the geographic
  side (ADR-012); direction is routing topology.

## Consequences and limitations

- `traversal_cost(edge)` remains direction-blind (used for
  knowledge-change comparisons); traversability is a separate, explicit
  question — consumers must ask both when routing.
- Anti-parallel one-way pairs between the same two nodes cannot be
  represented (one edge per pair); the OSM importer must FAIL
  DETERMINISTICALLY with a clear error when a real extract contains two
  distinct ways connecting the same retained nodes in opposite
  directions. It must NOT "solve" this by merging the pair into one
  bidirectional edge: the two ways may differ in geometry, OSM way
  identity, road class, distance and future metadata — a silent merge
  would be topology loss. A per-pair multi-edge representation is a
  future topology change with its own ADR, if a demonstrated need
  survives review.
- Sensor/world truth remains per-edge: a one-way edge being "open"
  means open in its direction(s); there is no per-direction dynamic
  state.

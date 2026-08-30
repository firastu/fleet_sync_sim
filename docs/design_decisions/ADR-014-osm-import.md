# ADR-014: Deterministic OSM PBF import

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::osm` (fleet_osm_import target, OsmPbfImporter),
  `apps/fleet_map_import`, test fixtures; builds on ADR-012 (topology /
  geometry separation) and ADR-013 (one-way edges)

## Context

Real geographic data must enter FleetSyncSim as a sovereign internal
map — without contaminating the planning core with GIS or OSM concepts.
The importer is the single place where OSM semantics are translated;
everything downstream sees only `BaseMap = Graph + MapGeometry`.

## Decision

### Dependency and target boundary

- New target `fleet_osm_import` (namespace `fleet::osm`), the ONLY place
  that includes `<osmium/...>` / `<protozero/...>`: pinned FetchContent
  (libosmium v2.20.0, protozero v1.7.1), populated WITHOUT executing
  their CMake (header-only use; include paths + system zlib suffice),
  linked PRIVATE with SYSTEM includes. `BaseMap` does not know OSM
  exists; core targets link nothing new.
- `apps/fleet_map_import` is a thin CLI that prints
  `OsmImportStats`; it contains no importer logic.

### Decoding vs policy vs construction

Three separate layers inside the importer: PBF decoding (libosmium
readers, policy-free), way eligibility (one private predicate over the
documented highway whitelist: motorway, trunk, primary, secondary,
tertiary, unclassified, residential, service, living_street, road,
track — plus >= 2 node refs), and topology construction. The PBF reader
never becomes synonymous with routing semantics; the policy is one
replaceable function.

### Topology construction (all deterministic, encounter-order-free)

- **Retained nodes**: way endpoints + nodes referenced >= 2 times across
  eligible way refs (self-intersections count). Intermediate way nodes
  are NOT graph nodes — their coordinates survive in edge polylines.
- **NodeId assignment**: retained OSM node ids sorted ascending -> dense
  ids. Node names are the decimal OSM node ids.
- **Segment splitting**: each way is cut at consecutive retained nodes.
- **EdgeId assignment**: canonical key (osm_way_id, segment_index,
  direction) sorted before creation; direction disambiguates future
  anti-parallel support. Unordered containers are used only for exact
  lookups, never iterated to assign anything.
- **oneway**: absent/no/false/0 -> Bidirectional; yes/true/1 -> Forward;
  -1 -> Reverse. ANY other value fails the import loudly (way id and
  value named). **junction=roundabout with an ABSENT oneway tag also
  fails loudly** ("implicit direction unsupported"): OSM gives such ways
  an implicit one-way direction, and importing them as bidirectional
  would confidently create the wrong road topology — a deterministic
  importer that fails on unsupported semantics is useful, one that
  confidently mis-imports is dangerous. Explicit oneway values on
  roundabouts (including a deliberate `oneway=no`) go through the normal
  parser.
- **Costs**: haversine polyline length (mean earth radius 6371008.8 m)
  — the SUM over consecutive polyline points, never the
  endpoint-to-endpoint geodesic (which would underestimate winding
  roads; locked by a bent-fixture regression test) — computed in the
  importer and handed to `Graph::Builder::connect` as a plain positive
  cost. The graph never knows what a meter is. A non-positive/non-finite
  length fails the import (way + segment named); no manufactured costs.
- **Geometry**: node coordinates and polyline endpoints REUSE the exact
  canonical coordinate values (MapGeometry's exact-equality identity
  contract, ADR-012). A `oneway=-1` edge keeps the way-order polyline
  with stored endpoints (a, b) — the orientation contract validates it
  structurally.
- **One physical edge per unordered retained-node pair** (ADR-013): ANY
  second imported segment mapping to an already-used pair — opposite
  directions (anti-parallel one-ways), the SAME direction (parallel
  roads), or a revisit within a single way (looping geometry) — fails
  the import with way AND segment provenance for both offenders:
  never merged, never dropped (the segments may differ in geometry, way
  identity, road class, distance and future metadata).
- Unsplit closed ways (a segment from a node back to itself) fail the
  import — they cannot be represented without a self-loop.

### Fixtures and determinism proof

- Committed tiny fixtures (source: `generate_fixture.cpp`, human-readable
  mirror `tiny_network.osm`, README documents regeneration): a
  deliberately bent bidirectional way (summed-polyline cost must exceed
  the endpoint geodesic), `oneway=yes`, `oneway=-1`, an excluded footway;
  plus five error fixtures (`oneway=alternating`, anti-parallel ways,
  same-direction parallel ways, implicit roundabout, a single way
  revisiting the same node pair).
- Tests assert the EXACT imported representation (NodeId mapping, edge
  table, directions, haversine costs, coordinates, polylines), planner
  behavior over the imported one-ways, loud error messages with way ids,
  and **import-twice deep equality** (ids, adjacency, costs, geometry,
  stats) — the reviewer-critical guard against silent
  unordered-iteration nondeterminism.
- `.gitignore` blocks `*.osm.pbf` except `tests/fixtures/osm/*`;
  regional extracts never enter the repository.

## Alternatives considered

- **Hand-rolled PBF/protobuf parser.** Rejected: spends correctness
  effort on wire formats instead of the sovereign-map problem.
- **Executing libosmium's CMake (target `osmium`).** Rejected: foreign
  build logic at configure time, transitive requirements (expat, tests);
  header-only population is smaller and reproducible.
- **Silent fallbacks** (unsupported oneway -> bidirectional,
  non-positive length -> epsilon cost, anti-parallel -> merge).
  Rejected: every silent fallback is a wrong map that looks like a right
  one. Failures name the offending OSM object.
- **Keeping intermediate way nodes as graph nodes.** Rejected: decision
  points only — the ADR-012 split exists exactly so shape can be richer
  than topology.

## Consequences and limitations

- Real regional extracts will contain unsupported constructs (dual
  carriageways producing parallel pairs, untagged roundabouts, exotic
  oneway); today the import of such an extract fails loudly at the
  first offender — never silently mis-imported. Relaxations (per-pair
  multi-edge, implicit roundabout direction, profile-based speed costs)
  are future ADR-gated changes with explicit policies.
- OSM relations, turn restrictions, access/conditional tags, lanes,
  elevation: out of scope by design.
- Costs are lengths in meters; a travel-time profile is a future policy
  layered in front of `connect()`, not a graph change.

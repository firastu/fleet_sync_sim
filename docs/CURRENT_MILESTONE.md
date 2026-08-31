# Current Milestone

## Milestone M1 — Deterministic Distributed Reference Simulator

Status: COMPLETE (closed by commit #9: scenario runner + CLI +
deterministic structured trace; ADR-009)

The goal of this milestone was to establish a deterministic reference model
for distributed map knowledge, unreliable communication, and autonomous
rerouting.

Completion criteria — all satisfied:

1. scenarios deterministically exercise multiple robots
   (`scenarios/station_partition.json` via `ScenarioRunner`);
2. robots maintain genuinely separate local dynamic knowledge;
3. messages can be delayed, lost, duplicated, and reordered;
4. reconciliation produces deterministic results;
5. route-affecting observations trigger deterministic replanning;
6. temporary communication loss does not prevent local autonomy
   (partition → observation → reroute → reconnect → resynchronize →
   convergence, asserted through the runner);
7. tests reproduce behavior for a known seed (same scenario + seed =>
   byte-identical JSONL trace; different seed => different fault
   sequence);
8. the full test suite passes in debug, ASan+UBSan and TSan.

The next milestone is planned before new work starts (movement,
RobotState, replan-from-position; then world ground truth and
position-based sensing — see the roadmap discussion in the project
docs).

---

## Milestone M2 — Movement, sensing and geospatial grounding

Status: COMPLETE (closed by commit #13: interactive console; all items
#10–#13 delivered)

M1 proved the distributed map-knowledge core on a static fleet. M2 made
the fleet move, grounded truth and perception in a world model, and
connected FleetSyncSim to real geographic data in both directions:
agreed after M1:

1. **#10 — movement, `RobotState`, replan-from-position** (ADR-010) —
   robots walk their routes, reroute from where they are, park and retry
   when the goal is unreachable, and complete missions; movement is
   scenario opt-in with a mandatory duration horizon.
2. **#11 — world ground truth + position-based sensing** (ADR-011) —
   `set_world_edge_state` describes what happened, independent of any
   sensing configuration; a `PerfectLocalEdgeSensor` (pure perception:
   visible truth incident to the occupied node, no belief input) is the
   only path from truth to robot knowledge, with unchanged-fact
   suppression in the runner; `observe_edge` remains a low-level testing
   primitive.
3. #12A — geographic map foundation + OSM import (ADR-012: topology
   and geometry are separate map concerns; the importer produces
   BaseMap = Graph + MapGeometry, WGS84 side data used by outputs,
   never by planning);
4. #12B — geospatial output adapters: BaseMap debug GeoJSON and
   trace → GeoJSON export (docs/geospatial.md G1/G2), built as pure
   functions of (map, trace), never coupled into the simulator;
5. #13 — interactive console over the runner.

Scope guard for M2: same determinism discipline as M1 — every behavior
is a pure function of (scenario, resolved seed); no threads, no wall
clock, no sensor noise models before #11 defines them.

---

## Milestone M3 — Positioning, navigation and timing (PNT)

Status: IN PROGRESS (opened with ADR-016; commit 1 of the ladder)

The M2 foundations (movement over real geometry, truth vs belief,
operator inspection) make localization a natural extension rather than
a separate project. M3 establishes the boundary first and makes every
failure mode observable before choosing any estimator:

```text
GROUND TRUTH POSE -> SENSOR MODEL -> LOCALIZATION ESTIMATE
```

1. **#14 — the boundary, deliberately boring** (ADR-016):
   `GroundTruthPose` / `LocalizationEstimate` / `GnssModel` with
   `PerfectGnss` (zero-noise baseline, consumes no randomness) and
   `UnavailableGnss` (outages are model selection, not special
   cases). No uncertainty representation yet — staleness
   (`estimated_at`) is a first-class state.
2. Noisy GNSS on the deterministic RNG; observable degradation.
3. Outage / stale estimate scenarios; dead-reckoning drift;
   reacquisition.
4. THEN decide: complementary filter / EKF / particle filter / map
   matching on MapGeometry / cooperative localization — with an ADR
   recording why.

Scope guard unchanged: every behavior is a pure function of (scenario,
resolved seed); no threads, no wall clock.

---

## Implemented

- immutable `BaseMap`;
- CSR `Graph`;
- per-participant `DynamicMapOverlay`;
- composed `MapView`;
- deterministic `AStarPlanner`;
- sequenced `MapDelta` reconciliation;
- logical simulation clock;
- deterministic event queue;
- seeded network behavior;
- latency;
- packet loss;
- duplication;
- emergent reordering;
- autonomous robot-local map knowledge;
- route invalidation;
- autonomous rerouting;
- directed link state, partitions;
- `ControlStation` aggregation and reconnect synchronization;
- declarative JSON scenarios + `ScenarioRunner`;
- `--scenario/--seed/--trace` CLI;
- deterministic structured trace (console + JSONL);
- robot movement: `RobotState`, committed edge traversals, replan from
  position/in-transit destination, park-and-retry, mission completion
  (ADR-010);
- world ground truth + position-based sensing with a strict
  truth→sensor→belief boundary; `set_world_edge_state` scenarios
  (ADR-011);
- `MapGeometry`: optional WGS84 geometry side of `BaseMap`, topology
  unchanged (ADR-012, #12A.1);
- one-way edges in topology: `EdgeDirection`, planner-respected, default
  bidirectional preserves all existing behavior (ADR-013, #12A.2 part 1);
- deterministic OSM PBF import (`fleet::osm`, pinned libosmium/protozero
  PRIVATE to the importer target): whitelist policy, retained-node
  topology with preserved polylines, canonical id assignment,
  oneway subset, haversine costs, loud failures, import-twice
  determinism, tiny committed fixtures and a stats CLI (ADR-014,
  #12A.2 part 2);
- GeoJSON output adapters (`fleet::geojson`): BaseMap debug export and
  trace trajectory export — pure outward-facing functions, deterministic
  ordering, one tested `[lon, lat]` conversion point, `fleet_map_import
  --map-geojson` (ADR-015, #12B; geospatial doc G2 active);
- localization boundary (`fleet::localization`): `GroundTruthPose`,
  `LocalizationEstimate`, `GnssModel` with perfect/unavailable models —
  truth vs estimate, no estimator chosen yet (ADR-016, M3 commit 1).

---

## Current objective

M3, per the ladder above. Next after the boundary commit: noisy GNSS on
the deterministic RNG, then outage/stale-estimate scenarios — each
observable before any estimator is chosen.

---

## Work allowed in this milestone

- correctness fixes;
- deterministic scenario execution;
- map/reconciliation behavior;
- robot-local state;
- network fault semantics;
- event scheduling;
- route invalidation and replanning;
- integration tests;
- scenario definition;
- observability / deterministic trace output;
- documentation of current contracts.

---

## Explicitly out of scope

Do NOT introduce yet:

- threads or asynchronous runtime behavior except when explicitly working on
  the concurrency roadmap;
- GNSS simulation;
- IMU simulation;
- dead reckoning;
- Kalman filters;
- SLAM;
- visual odometry;
- LiDAR odometry;
- cooperative localization;
- distributed geometric map merging;
- detailed RF propagation;
- multi-hop communication routing;
- ROS 2;
- Gazebo.

Those capabilities belong to later milestones.

---

## Completion criteria

M1 is complete when:

1. scenarios can deterministically exercise multiple robots;
2. robots maintain genuinely separate local dynamic knowledge;
3. messages can be delayed, lost, duplicated, and reordered;
4. reconciliation produces deterministic results;
5. route-affecting observations trigger deterministic replanning;
6. temporary communication loss does not prevent local autonomy;
7. tests reproduce behavior for a known seed;
8. the full debug test suite passes.
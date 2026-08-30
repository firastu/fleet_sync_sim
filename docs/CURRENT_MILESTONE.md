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

Status: IN PROGRESS (opened with commit #10)

M1 proved the distributed map-knowledge core on a static fleet. M2 makes
the fleet move and grounds it in real-world data, per the gap analysis
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
  --map-geojson` (ADR-015, #12B; geospatial doc G2 active).

---

## Current objective

M2, item by item (see above). #10, #11, #12A and #12B are complete.
Next: #13 interactive console over the runner.

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
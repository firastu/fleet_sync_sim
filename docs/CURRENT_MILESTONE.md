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
- deterministic structured trace (console + JSONL).

---

## Current objective

Milestone closed. Next: define M2 scope (expected to start with robot
movement and `RobotState`, then world ground truth and position-based
sensing) before any new implementation work.

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
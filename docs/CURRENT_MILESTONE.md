# Current Milestone

## Milestone M1 — Deterministic Distributed Reference Simulator

Status: IN PROGRESS

The goal of this milestone is to establish a deterministic reference model
for distributed map knowledge, unreliable communication, and autonomous
rerouting.

This milestone does NOT attempt to model full robotics.

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
- autonomous rerouting.

---

## Current objective

Complete and harden the deterministic single-threaded reference model.

The reference implementation must provide behavior against which future
concurrent implementations can be compared.

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
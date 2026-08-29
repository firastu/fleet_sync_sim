# FleetSyncSim

Deterministic simulation of a distributed autonomous-robot fleet in modern
C++ (C++20). Robots maintain local maps, exchange map deltas over unreliable
links, reconcile conflicting observations, and reroute autonomously while the
control station is unreachable.

**Status: early Milestone 1** — the deterministic, single-threaded reference
(Stage 0). Currently implemented: the map core (immutable `BaseMap`, CSR
`Graph`, per-participant `DynamicMapOverlay`, composed `MapView`),
sequenced `MapDelta` reconciliation, and a deterministic `AStarPlanner`
over `MapView`. See the
[roadmap](#roadmap) and the [design records](docs/design_decisions/).

## The engineering problem

1. **Local autonomy.** Navigation, local mapping, P2P exchange and rerouting
   must not have a hard dependency on the control station.
2. **Eventual consistency.** Deltas arrive delayed, duplicated, reordered or
   not at all. Participants must converge on shared map truth after the
   network heals, without distributed consensus.
3. **Determinism.** A scenario file plus a seed must reproduce the exact
   event trace, so that concurrency versions can be validated against the
   sequential reference.

## Building

Linux, CMake >= 3.21, GCC >= 11 or Clang >= 14. The first configure fetches
GoogleTest (requires network); later configures are offline.

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Presets: `debug`, `release`, `asan` (Address+UB Sanitizers), `tsan` (Thread
Sanitizer — used from the concurrency stages onward).

Run the demo executable (currently: plan a mission, block an edge on the
route, replan around it):

```sh
./build/debug/apps/fleet_sim/fleet_sim
```

## Repository layout

```
include/fleet/   Public headers (common, map, planning, robot, ...)
src/             Library implementation
tests/           Unit and integration tests (GoogleTest)
docs/            Architecture notes and ADRs
scenarios/       (upcoming) declarative simulation scenarios
apps/            fleet_sim executable (map-core demo; scenario CLI comes later)
labs/            (upcoming) intentionally broken concurrency examples
```

## Roadmap

| Stage | Content | Status |
|-------|---------|--------|
| 0 | Deterministic single-threaded reference | in progress |
| 1 | Naive concurrency (intentional races, labs) | planned |
| 2 | Coarse-grained correct synchronization | planned |
| 3 | Snapshots, message passing, less shared state | planned |
| 4 | Bounded thread pool, parallel planning | planned |
| 5 | Systematic fault injection | planned |
| 6 | Performance engineering and benchmarks | planned |
| 7 | ROS 2 / Gazebo adapters (core stays transport-agnostic) | planned |

## Design records

- [ADR-001: Immutable base map with per-participant dynamic overlays](docs/design_decisions/ADR-001-immutable-base-map.md)
- [ADR-002: Deterministic single-threaded reference first](docs/design_decisions/ADR-002-single-threaded-reference.md)
- [ADR-003: Deterministic planner tie-breaking contract](docs/design_decisions/ADR-003-planner-determinism.md)
- [ADR-004: Sequenced MapDelta reconciliation](docs/design_decisions/ADR-004-map-delta-reconciliation.md)

## Research

- [Communication-aware autonomy roadmap](docs/communication-aware-autonomy-roadmap.md) — long-term direction for communication-constrained autonomy, derived from Suojanen, *Military Communications in the Future Battlefield* (2018), with explicit source / interpretation / extrapolation separation and section/page traceability.

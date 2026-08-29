# FleetSyncSim

FleetSyncSim is a deterministic C++20 simulation platform for exploring
**sovereign cooperative autonomy for robots**.

The long-term goal is to study robots that can navigate using locally
controlled maps, maintain independent world models, exchange information
peer-to-peer, and continue operating when centralized infrastructure,
communications, or positioning sources become unavailable.

The current implementation focuses on one foundational problem:

**distributed map knowledge under unreliable communications**.

Robots maintain local maps, exchange sequenced map deltas over unreliable
links, reconcile observations, and reroute autonomously while the control
station may be unreachable.

> **Status: Milestone M1 complete — deterministic single-threaded reference
> simulator.**

Currently implemented:

- immutable `BaseMap`;
- CSR `Graph`;
- per-participant `DynamicMapOverlay`;
- composed `MapView`;
- deterministic `AStarPlanner`;
- sequenced `MapDelta` reconciliation;
- logical simulation time;
- deterministic event scheduling;
- seeded network faults:
  - latency;
  - loss;
  - duplication;
  - emergent reordering;
- directed link state and partitions;
- `ControlStation` fleet knowledge aggregation;
- reconnect synchronization by idempotent re-announcement;
- autonomous robot-local knowledge and rerouting;
- declarative JSON scenarios, `--scenario/--seed/--trace` CLI;
- deterministic structured trace (console + JSONL).

See:

- [Current milestone](docs/CURRENT_MILESTONE.md)
- [Project vision](docs/PROJECT_VISION.md)
- [Documentation index](docs/README.md)
- [Design records](docs/design_decisions/)
- [Research](docs/research/)

---

## The engineering problem

1. **Local autonomy.**

   Navigation, local mapping, peer-to-peer exchange, and replanning must not
   have a hard dependency on the control station.

2. **Local knowledge under imperfect communication.**

   Every robot maintains its own view of dynamic state. Messages may arrive
   delayed, duplicated, reordered, or not at all.

3. **Graceful convergence.**

   Robots must remain useful while disconnected and reconcile distributed
   knowledge when communication becomes available again, without requiring
   distributed consensus for every observation.

4. **Deterministic reference behavior.**

   A scenario plus seed must reproduce the same observable simulation trace.
   Future concurrent implementations are validated against this reference.

---

## Building

Requirements:

- Linux
- CMake >= 3.21
- GCC >= 11 or Clang >= 14

The first configure fetches GoogleTest and nlohmann/json and therefore
requires network access. Later configures can operate offline.

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Sanitized builds: `cmake --preset asan|tsan` (TSan on GCC 13 needs
`setarch $(uname -m) -R` for builds and test runs; ASLR entropy issue,
per-process workaround only).

### Running scenarios

```sh
# Built-in founding scenario (station partition, observation, reroute,
# reconnect, resynchronize):
./build/debug/apps/fleet_sim/fleet_sim

# Declarative scenario file + explicit seed + machine-readable trace:
./build/debug/apps/fleet_sim/fleet_sim \
    --scenario scenarios/station_partition.json --seed 1234 --trace run.jsonl
```

Seed precedence: `--seed` overrides the scenario file's `seed`, which
overrides the documented default `0`; the resolved seed is logged in the
first trace events. Same scenario + same resolved seed produce a
byte-identical trace. Scenario format and trace contract: ADR-009.
# ADR-009: Scenario semantics and the deterministic trace contract

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::scenario` (Scenario, ScenarioLoader, ScenarioRunner,
  TraceEvent, sinks), `apps/fleet_sim` (CLI)

## Context

Milestone M1's behavior (ADR-001..ADR-008) was demonstrated through
hand-wired demos and unit tests. Reproducibility required the *code* of
the demo, not a declarative input. This commit closes that gap: a
simulation run must be fully described by a scenario file plus a seed,
and everything an outside observer sees must flow through one
machine-readable artifact, because analysis tooling and GeoJSON export
(docs/geospatial.md stage G2) will consume it.

## Decision

### Scenario = declarative input data; runner = wiring only

- **Scenario** is pure typed data (`Scenario`, `ScenarioRobot`,
  `ScenarioEvent`, closed action set). JSON is an *input encoding*
  only, parsed by `ScenarioLoader` (nlohmann/json v3.11.3, PRIVATE to
  the loader; parser types never appear past that boundary). The loader
  resolves author vocabulary — node names (`"A"`), edge names
  (`"F-G"`), participant names (`"robot_a"`, `"station"`) — into
  strong ids, validates the whole description up front, and throws
  `std::invalid_argument` with deterministic messages; no partial
  scenario state ever reaches the simulation.
- **ScenarioRunner** orchestrates existing public APIs only: it owns
  the per-run `EventQueue`, one `NetworkSimulator(config, seed)`, one
  `Robot` per declaration, and one `ControlStation` when declared. It
  must never absorb robot, network, reconciliation, planning or station
  logic. Every scheduled effect calls a public API (`set_link_state`,
  `observe`, `resynchronize`) and then observes the result.
- **Action set is closed and minimal**: `set_link_state`,
  `observe_edge`, `resynchronize`. New actions arrive with the commits
  that add the underlying semantics — never before.
- **Event ordering**: events are stably sorted by tick; equal-tick
  events keep file order, which becomes execution order through
  ADR-005's enqueue counter. Because an observation's tick IS its
  observation time, sorting also satisfies the Robot producer contract
  (non-decreasing per-edge observation ticks) by construction.
- **Fan-out order** is deterministic: a robot's sink sends to every
  other participant in robot declaration order, station last.

### Seed precedence

`CLI --seed` > scenario-file `seed` > documented default `0`
(`kDefaultSeed`), resolved by `resolve_seed()` before the runner is
constructed. The resolved seed is emitted as a tick-0 trace event, so
the trace is self-describing: (scenario, resolved seed) is a complete
reproducibility key.

### TraceEvent is the single observability artifact

- Console output and JSONL files are *formatters* over the same
  `TraceEvent` stream (`ConsoleTraceSink`, `JsonlTraceSink`); they are
  never two unrelated systems. Sinks are observation-only: they cannot
  influence behavior.
- **Stable event type names**: `scenario`, `seed`, `route`,
  `link_state`, `observation`, `send`, `delivery`, `reconcile`,
  `resynchronize`. Renaming a type is a breaking contract change.
- **Determinism rules**: fields are emitted in insertion order; values
  are deterministic functions of (scenario, resolved seed). No
  wall-clock timestamps, no pointer addresses, no unordered-container
  iteration may reach the output. Same scenario + same seed =>
  byte-identical trace (enforced by test).
- **Causality note**: an observation's `send` events appear *before*
  the `observation` event itself, because the robot's sink runs
  synchronously inside `observe()`. This ordering is stable and
  documented, not a bug.
- GeoJSON/analysis exports will be built as adapters over traces —
  another sink — never by coupling into the simulator.

## Alternatives considered

- **Homemade YAML/JSON parser.** Rejected: one pinned, well-contained
  dependency beats a maintenance liability.
- **Parser types in Scenario.** Rejected: leaks the encoding into
  simulation code and makes every consumer transitive on the parser.
- **Separate pretty log and machine log.** Rejected: two systems
  drift; one event stream with two formatters cannot.
- **DSL / scripting in scenarios.** Rejected for this stage: a
  declarative event list covers M1; scripting returns only if a
  demonstrated need survives review.

## Consequences and limitations

- Scenario files depend on node/edge names of the map they target; a
  renamed node invalidates the scenario at load time (deterministic
  error, not silent misbehavior).
- The runner supports exactly one station and no robot-to-robot
  relaying yet — matches ADR-007/008 Stage-0 semantics.
- Adding an action requires touching the closed variant, the loader,
  the runner and this ADR's action list — deliberate friction.

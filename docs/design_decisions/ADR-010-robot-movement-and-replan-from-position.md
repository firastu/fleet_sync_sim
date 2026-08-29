# ADR-010: Robot movement, RobotState and replan-from-position

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::robot` (RobotState, transit API, effective-start
  replanning), `fleet::scenario` (movement wiring, MovementSettings,
  duration horizon), trace taxonomy extension

## Context

Until now robots were static: routes were planned from the mission start
no matter what, and nothing moved along them. Reconciliation and
rerouting semantics (ADR-004/007) were proven on a static fleet. The
roadmap's next step is movement — which immediately raises the questions
this ADR answers: where is a robot (position model), what may a replan
change (a robot mid-edge cannot teleport), and who drives time forward
(the robot must not become an EventQueue client).

## Decision

### Position model: node-resident with committed traversals

- `RobotState` = `{position: NodeId, in_transit: optional<RobotTransit>,
  mission_complete: bool}`. A robot occupies a node, or is committed to
  one edge traversal (`edge, from, to, arrival`). There is deliberately
  NO sub-edge/continuous position: planning needs node-granular starts,
  and finer motion models belong to the geospatial stages.
- `begin_transit(at, ms_per_cost_unit)` starts the next route edge;
  traversal time is `ceil(effective_cost * ms_per_cost_unit)` ticks
  (effective cost = this robot's own knowledge, same view the route was
  planned on). `complete_transit()` commits the arrival.
- **A traversal is physically committed.** If the robot learns its
  current edge is blocked while on it, the traversal still completes —
  a robot cannot un-drive. Knowledge affects departures, never arrivals.

### Replan from the effective start

- `plan_current_route()` plans from the **effective start**: the
  destination of the in-progress traversal while in transit, otherwise
  the current position. A robot never re-plans from a node it has left;
  while in transit, the committed edge is treated as already decided and
  the plan starts at its destination.
- Effective-change replanning rules (ADR-007) are unchanged — only the
  start moved. Note the emergent property: while in transit, the
  in-progress edge can never appear in the robot's own route (the route
  starts at its destination), so "route uses the edge" checks apply
  cleanly to edges ahead.
- `complete_transit()` re-derives the route from the new position. This
  is a deterministic suffix-equivalent re-derivation, not a
  knowledge-driven change: it is NOT traced as a `route` event, and the
  ADR-007 replan triggers do not apply to it.

### Driving model: runner-owned scheduling, robot-owned semantics

- `Robot` remains queue-free (ADR-007 boundary): it exposes pure
  begin/complete transit calls. `ScenarioRunner` owns one
  self-rescheduling advance chain per robot (arrival events, retry
  events) — consistent with ADR-009's "runner = wiring + scheduling".
- A robot with no usable route (goal unreachable under its knowledge)
  parks and retries every `retry_ms` ticks. Movement advances ONLY at
  chain events: a knowledge improvement does not wake a parked robot
  early — it departs at the next retry tick. Deterministic, and free of
  reverse coupling from knowledge into the movement chain.
- Movement is scenario opt-in: a `"movement"` object enables it
  (`ms_per_cost_unit` default 1000, `retry_ms` default 1000); absent =
  static fleet, so pre-movement scenarios (including the founding
  scenario) produce byte-identical traces.

### Termination: movement requires a duration horizon

- A scenario with movement must declare `duration_ms`; the loader
  rejects it otherwise. The runner then executes via
  `run_until(duration)` (ADR-005 horizon semantics): events beyond the
  horizon never run and the clock lands exactly on the horizon. Without
  this, a permanently unreachable goal would park-and-retry forever and
  `run_to_completion` would never return. `duration_ms` may also be
  used without movement as a plain horizon.
- **No zero-time self-scheduling loops**, guaranteed at three layers:
  (1) graph construction rejects non-positive traversal costs, so
  `ceil(cost x ms_per_cost_unit) >= 1` for any legal speed and every
  traversal advances the clock by at least one tick; (2) zero timing
  values (`ms_per_cost_unit`, `retry_ms`) are rejected by the loader
  AND by the ScenarioRunner constructor — the choke point covering
  programmatically constructed scenarios; (3) `begin_transit`
  defensively clamps traversal ticks to >= 1 regardless.

### Trace taxonomy extension (amends ADR-009's list)

- `departure` {edge, from, to, arrival} — committed traversal starts;
  `arrival` {node}; `mission_complete` {goal}. Sources are robot names;
  node/edge fields use map names (the vocabulary scenarios use), giving
  future GeoJSON export (geospatial G2) a complete trajectory: depart →
  arrive → complete chains fully determine each robot's path over time.

## Alternatives considered

- **Continuous sub-edge position.** Rejected for now: no consumer
  (planner needs node starts; the trace needs hops), real cost in model
  complexity. Revisited with geospatial grounding.
- **Robot-owned movement scheduling (Robot holds EventQueue&).**
  Rejected: couples the autonomy component to the simulation scheduler;
  the queue-free Robot keeps ADR-007's boundary crisp.
- **Reactive wake-up (knowledge change triggers immediate movement
  decision).** Rejected: requires the reconciliation path to know about
  movement chains (reverse coupling). Polling at retry cadence is
  simpler and fully deterministic; the latency cost is one retry period.
- **Canceling in-transit traversals on block events.** Rejected as
  unphysical; also breaks the single-directional flow
  knowledge → replan → departures.
- **Movement on by default.** Rejected: changes every existing
  scenario's trace; opt-in keeps #9 outputs reproducible.

## Consequences and limitations

- A parked robot reacts to reopened routes with up to `retry_ms` delay.
- Events scheduled beyond `duration_ms` never fire (documented loader
  behavior; scenarios should place their events inside the horizon).
- `RobotTransit.arrival` is computed once at departure; later knowledge
  does not change an in-flight traversal's duration (there is no speed
  model yet).
- Traversal duration uses knowledge at departure time; cost changes
  while in transit affect only future departures.

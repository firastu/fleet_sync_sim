# ADR-005: Deterministic event scheduling and logical time

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::simulation`

## Context

Stage 0 models all asynchrony — observations, message delivery, robot
processing, link changes — as scheduled effects on a single event queue
instead of threads (ADR-002). The mechanism that orders those events is
the determinism backbone of the simulator: if event order were ever
unspecified, traces could not be reproduced and later stages could not
be validated against the Stage 0 oracle. The communication-aware
autonomy roadmap (§23.3) requires an explicit equal-timestamp tie-break
rule.

## Decision

- Logical time is `common::Tick`; **1 tick = 1 millisecond**
  (`SimulationClock::kTicksPerSecond == 1000`). Tick↔second mapping is a
  simulation-layer concern; domain types remain unitless.
- `EventQueue` is the only driver of time. Simulated time advances
  exclusively through deterministic queue operations: to an event's
  scheduled tick when the event executes, or to the horizon requested by
  `run_until(T)` — postcondition `now() == T`, even across idle time, so
  subsequent relative scheduling can never land before the requested
  horizon. A backward horizon throws `std::invalid_argument`. Nothing
  reads wall-clock time for behavior.
- **Total execution order on events: `(tick, enqueue_order)`** — earlier
  tick first; equal ticks run in the order they were scheduled, using a
  monotonically increasing enqueue counter assigned at `schedule()` time.
  No heap, container or pointer tie-breaks are observable.
- Effects may schedule further events, including at the current tick. A
  reentrant same-tick follow-up runs after the currently executing
  effect *and* after every event already queued at that tick: enqueue
  order is assigned when `schedule()` is called, so a follow-up can
  never overtake an already-queued event.
- **Basic exception safety for effects:** EventQueue never catches effect
  exceptions. If an effect throws, the exception propagates to the
  caller; the throwing event is already consumed; the clock remains at
  that event's tick; all other events — including any the effect
  scheduled before throwing — remain queued. No transactional rollback.
- Causality is enforced: scheduling at a tick earlier than `now()`
  throws `std::invalid_argument` — a past event is a scenario bug that
  would otherwise silently break reproducibility.

## Alternatives considered

- **Per-actor queues / multiple clocks.** Premature: one total order is
  simpler and sufficient until a measured need appears.
- **Real-time loop with sleeps.** Rejected (ADR-002).
- **Priority dimension in the event key `(tick, priority, id)`.** Not
  needed yet; enqueue order is a sufficient and simpler tie-break, and a
  priority dimension can be added later without changing existing
  semantics.

## Consequences

- Traces are bit-reproducible for a fixed schedule; locked by a
  repeated-run test with many exact tick collisions.
- Scenarios own termination: `run_to_completion()` on a scenario that
  always schedules new events runs forever (documented contract).
- Effect exceptions are outside the queue's responsibility: they
  propagate with the queue left consistent (basic exception safety —
  documented and tested).
- The queue is non-copyable/non-movable: scheduled effects typically
  capture references to simulation state.

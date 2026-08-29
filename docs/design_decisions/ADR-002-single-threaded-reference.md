# ADR-002: Deterministic single-threaded reference implementation first

- Status: Accepted
- Date: 2026-08-29
- Scope: entire simulator core

## Context

The end state of this project is a concurrent fleet simulator. It is tempting
to start with threads, queues and atomics immediately. Experience shows the
opposite order is cheaper: concurrency is much easier to build on top of a
correct, deterministic sequential core than to debug directly, because race
reports are non-reproducible while sequential traces are.

## Decision

Milestone 1 is single-threaded and deterministic end to end:

- one logical clock advances the world in discrete ticks;
- all "asynchrony" (network latency, partitions, robot processing) is modeled
  as scheduled events on a single event queue, not as threads;
- all randomness flows through seeded engines owned by the simulator:
  scenario + seed fully determine the event trace;
- no component reads wall-clock time; wall time may only appear in
  telemetry, never in behavior.

This implementation becomes the behavioral oracle for later stages: the
concurrent versions must reproduce its traces on fixed seeds.

## Alternatives considered

- **Start concurrent.** Rejected: without an oracle there is no way to
  distinguish a scheduling-dependent bug from correct behavior, and every
  test failure becomes a debugging session.
- **Real-time loop with sleeps.** Rejected: non-deterministic, untestable in
  CI, and conflates simulated time with wall time.

## Consequences

- Every Stage 0 class is documented "not thread-safe, by design". When
  concurrency lands, the seams (event queue, messages as values, immutable
  base map) are already shaped for it.
- The single-threaded event-driven path must remain available in later
  stages for tests and regression comparison; we resist removing it.
- Determinism constrains implementation choices: no unordered-container
  iteration order may leak into observable behavior (CSR adjacency is
  ordered), and tie-breaking in planners must be deterministic.

# ADR-006: Deterministic network fault model

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::network` (+ `fleet::simulation::DeterministicRng`)

## Context

Distributed behavior must be studied under reproducible communication
faults: latency, latency variation, packet loss, duplication and message
reordering — before any real transport exists. Determinism is a core
project claim (ADR-002), so every random decision must replay bit-identically.

## Decision

- **Seeded engine, custom sampling.** One `std::mt19937_64` stream per
  `NetworkSimulator`, seeded explicitly (no `std::random_device`, no wall
  time, no global RNG). Standard distributions
  (`uniform_int_distribution`, `bernoulli_distribution`, ...) are NOT
  used: their algorithms are unspecified across standard-library
  implementations, breaking cross-platform replay. Instead
  `simulation::DeterministicRng::uniform_below(bound)` implements
  uniform sampling in [0, bound) via classic rejection sampling
  (`threshold = 2^64 mod bound`; reject draws below the threshold;
  return `draw % bound`) — fully specified arithmetic, no modulo bias.
- **Exact probability.** `network::Probability` is parts-per-million
  integer (0 = never, 1'000'000 = always); no floating-point thresholds.
- **All randomness sampled synchronously in `send()`** — never at
  delivery-event execution. Order: loss trial; if not dropped, sample
  latency; duplication trial (only for delivered transmissions); if
  duplicated, sample an independent second latency. Consumption
  contract: trials at 0%/100% and latency ranges with min == max consume
  no engine output, so pure configurations draw zero numbers.
- **Delivery via EventQueue.** Delivery is scheduled at
  `clock().now() + latency` (overflow-checked, throws). The simulator
  never advances the clock directly. Equal delivery ticks resolve by
  ADR-005 enqueue order.
- **Loss semantics:** a dropped logical transmission schedules zero
  deliveries; duplication is only considered for non-dropped
  transmissions; duplicates never duplicate recursively; at most two
  deliveries per send, both carrying the identical MapDelta value —
  network copies never mutate delta identity.
- **Reordering is emergent.** Two sends sample independent latencies,
  so a later send may be delivered first. There is deliberately no
  `reorder_probability` and no packet swapping: reordering is a
  consequence of the latency model, not an injected fault.
- **Transport/domain separation.** Endpoints are `network::EndpointId`
  (transport addresses); `MapDelta` carries no transport fields. The
  transport sender (EndpointId) and the observation source (RobotId)
  are distinct concepts, which is exactly what relaying will need.
- **Endpoints are permanent in Stage 0** (register once); the
  destination handler is resolved and copied at **send time** —
  delivery-time lookup would add no semantics while making scheduled
  deliveries depend on NetworkSimulator lifetime. Unknown destinations
  throw at send time. Scheduled deliveries capture only the transport
  sender, a payload copy and a handler copy, so the simulator need not
  outlive deliveries it has already scheduled. Handler exceptions
  propagate with ADR-005 semantics — no retry, no conversion to loss.

## Alternatives considered

- **`std::*_distribution`.** Rejected: unspecified algorithms break the
  cross-platform replay claim.
- **Explicit reorder probability.** Rejected: latency variation already
  produces reordering; a swap-style injection would be fault injection,
  not a network model.
- **One RNG stream per link.** Postponed: one stream means outcomes
  depend on the ordered sequence of `send()` calls, which is
  deterministic; per-link streams solve no current problem.
- **Real sockets / DDS / ROS now.** Out of scope for Stage 0; the API
  (send + registered receive handlers) is shaped so a real transport can
  replace the simulator behind the same boundary later.

## Consequences and limitations

- Inserting an earlier `send()` shifts subsequent outcomes on the
  single stream — acceptable and documented; scenarios are replayed as
  ordered send sequences.
- The fault model is deliberately simple: no bandwidth, congestion,
  link asymmetry, partitions, offline endpoints, retransmission, ACKs
  or per-link queues yet. Partitions arrive with link-state work;
  relaying and store-carry-forward will reuse the EndpointId/source
  distinction.
- The payload type is concretely `MapDelta` (one real payload today);
  generalization is postponed until a second message type exists.
- `DeterministicRng` lives in `fleet::simulation` because it is a
  determinism primitive of the simulator core, not a networking detail.

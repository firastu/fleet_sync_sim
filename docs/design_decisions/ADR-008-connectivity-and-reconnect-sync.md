# ADR-008: Connectivity, partitions and reconnect synchronization

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::network` (link state), `fleet::robot` (resynchronize),
  `fleet::station` (ControlStation)

## Context

The project's founding scenario requires the fleet to keep operating
while the control station is unreachable, and to converge after the
station reconnects and receives delayed, duplicated or reordered
messages. Until now the simulated network always delivered what was
sent; there was no connectivity concept and no station.

## Decision

- **Directed link state, send-time evaluation.** Links are per
  (from -> to) transmission path, up by default, toggled via
  `set_link_state`. A down path makes `send()` a deterministic drop
  (`SendResult::link_down`) that consumes no randomness. Connectivity is
  checked at send time ONLY: deliveries already scheduled are never
  interrupted — there is no per-hop interruption model. A partition is
  expressed by disabling both directions explicitly: the primitive is
  directional; symmetry is scenario policy, not hidden state.
- **ControlStation is a thin participant.** Overlay plus the shared
  reconciliation policy; no mission, route, transport knowledge or
  producing identity in Stage 0. It is an aggregator that can never be
  a hard dependency for robot autonomy.
- **Reconnect synchronization by re-announcement.** After a reconnect,
  scenario policy calls `Robot::resynchronize()`: the robot re-emits its
  current knowledge winners in ascending EdgeId order with their
  ORIGINAL event identity. Receivers that already know a fact suppress
  it as Duplicate; receivers that missed it Apply it — ADR-004
  idempotence makes re-announcement safe without any reliability
  protocol, ACKs or history retention. This is explicit state sync, not
  gossip of individually received deltas.

## Alternatives considered

- **Symmetric link primitive.** Rejected as the primitive: half-open
  (asymmetric) partitions are also legitimate faults; symmetry composes
  trivially on top, the reverse does not.
- **Delivery-time connectivity checks / killing in-flight messages.**
  Rejected: couples delivery events back to mutable simulator state and
  adds a per-hop interruption model Stage 0 does not need.
- **Retransmission queues / ACK protocols on disconnect.** Rejected:
  re-announcement of current winners achieves convergence with zero
  additional protocol machinery; true reliability protocols remain
  future work.
- **Station as knowledge authority (pull model).** Rejected for Stage 0:
  the station holds no special powers; it reconciles like any
  participant.

## Consequences and limitations

- Deltas sent while a link was down are gone; convergence relies on a
  resynchronize being triggered by scenario policy after reconnect.
- resynchronize() re-sends only the *winners* — dominated observations
  are not part of state sync (the winner is the knowledge that matters).
- Link state is global simulator configuration, not a scheduled event:
  scenario code changes it at a logical time by calling it between
  run_until() horizons.
- No bandwidth/capacity limits, no partial degradation (a link is up or
  down), no endpoint lifecycle. Those arrive with later roadmap stages.

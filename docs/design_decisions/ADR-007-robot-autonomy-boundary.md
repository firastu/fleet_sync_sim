# ADR-007: Robot autonomy boundary

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::robot`

## Context

After the transport simulator existed, the remaining gap was the
participant itself: something that owns a mission, local knowledge, and
routing autonomy, and that keeps operating from its own knowledge. The
demo had been coordinating overlays, reconcilers and the planner by
hand. The design question was where the robot's boundary sits relative
to the transport.

## Decision

- `Robot` owns its `Mission`, `DynamicMapOverlay`, `MapReconciler`,
  current `Route` and a zero-heuristic planner. It borrows the shared
  immutable `BaseMap`.
- **No network dependency.** Deltas flow out through a `DeltaSink`
  callback; the application wires sinks to the transport. The robot
  never sees transport types, endpoint addresses or delivery
  diagnostics — in particular it cannot base autonomy on simulated
  future delivery times (oracle knowledge).
- **Sequence production lives here**: observe() allocates the next
  sequence in the robot's per-(source, edge) event stream (starting at
  1), matching the ADR-004 identity consumed by every reconciler.
  Because Robot is the producer, the ADR-004 stream-chronology contract
  is *enforced*, not assumed: a tick earlier than the stream's last
  observation throws std::invalid_argument before any sequence, state
  or emission is touched (same tick is legal — non-decreasing).
  Sequence exhaustion (UINT64_MAX per stream) throws
  std::overflow_error; sequences never wrap to the reserved 0.
- Local observations and received deltas share the single reconcile()
  path. An observation is emitted to the sink even when locally
  Dominated — it is valid information for other participants.
- **No rebroadcast**: received deltas are never re-emitted. Gossip and
  relaying are future roadmap stages.
- **Effective-change replanning.** current_route() is the deterministic
  best route under current knowledge, not merely "a route not yet
  broken": when an Applied delta makes effective traversal worse
  (OPEN -> BLOCKED, or costlier), the robot replans only if the current
  route uses the edge; when it makes traversal better (BLOCKED -> OPEN,
  or cheaper), the robot replans regardless of the current route so a
  re-opened edge can restore a shorter one; a provenance/time refresh
  with unchanged effective traversal does not replan. Evaluated via
  MapView effective cost before/after reconciliation, so it extends
  naturally to future dynamic cost overrides.
- **DeltaSink exception semantics**: emission happens after the local
  observation is committed; a throwing sink propagates with nothing
  rolled back (sequence stays consumed). No transactional emission.
- Replanning recomputes from the mission start; Stage 0 has no
  movement.

## Alternatives considered

- **Robot depends on network::NetworkSimulator directly.** Rejected:
  couples autonomy to a simulated transport and invites oracle
  knowledge; a real transport (ROS 2, DDS) could not be swapped in
  behind the same boundary.
- **Fleet-wide shared overlay.** Rejected (ADR-001): destroys local
  autonomy and disagreement — the phenomenon the project studies.
- **Gossip/rebroadcast now.** Rejected: needs duplicate suppression
  policies and loop handling; no current scenario requires it.

## Consequences

- The application (later: scenario runner) is responsible for wiring
  sinks and delivery handlers; robots remain inert until wired.
- A sink-less robot still operates locally (useful for isolated tests).
- No movement, mission completion or RobotState yet: those arrive with
  movement; the mission currently defines only the planning query.
- The sink is called synchronously inside observe(); it must outlive
  the robot's use, like the borrowed BaseMap.

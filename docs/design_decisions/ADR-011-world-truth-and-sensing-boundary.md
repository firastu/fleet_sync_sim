# ADR-011: World ground truth and the sensing boundary

- Status: Accepted
- Date: 2026-08-29
- Scope: `fleet::world` (World, ObservationModel, PerfectLocalEdgeSensor),
  `fleet::scenario` (set_world_edge_state, sensing wiring, trace
  extension), AGENTS.md boundary rule

## Context

Scenarios told robots what they "observed" (`observe_edge`), which
conflates what happened in the world with what a robot could perceive.
The sovereign-autonomy architecture requires the separation:

    SIMULATION TRUTH          — authoritative, simulation-side only
        X  (robots cannot access directly)
    ObservationModel          — position-based, replaceable
    ROBOT LOCAL BELIEF        — overlays, reconciliation, routes

Movement (ADR-010) supplies the missing input: where a robot physically
is, and therefore what it could sense.

## Decision

### World: authoritative truth, simulation-side only

- `fleet::world::World` holds the authoritative dynamic state of every
  edge (dense by EdgeId; initial truth = the immutable base map, i.e.
  all open — a scenario without world changes models an unchanging
  world and initial sensing reports nothing).
- NOTHING in `fleet::robot` includes, links or references `fleet::world`
  (dependency direction: world → robot/map, never the reverse). Robots
  cannot query truth; they only ever receive `Robot::observe()` calls.
  This rule is recorded in AGENTS.md.

### ObservationModel: the only truth→belief path, replaceable

- `ObservationModel::sense(world, robot_state, belief)` is a PURE
  function: no queues, no network, no mutation. The simulation wiring
  (ScenarioRunner) turns returned observations into `Robot::observe()`
  calls. Later models (noisy, range-limited, map-matching, real-sensor
  adapters) replace the implementation without moving the boundary.
- Stage 0 ships exactly one model, `PerfectLocalEdgeSensor`,
  deliberately artificial and deterministic:
  - range = every edge incident to the robot's occupied node (while in
    transit, the occupied node is the traversal's departure node; the
    transit edge is incident to it, and changes at the upcoming node
    are learned on arrival);
  - perfect accuracy, confidence 1.0;
  - belief gating: only edges whose truth status differs from the
    robot's believed status (untracked = open) are reported — so a
    sensed fact is reported once per truth change, not continuously;
  - output in ascending EdgeId order (deterministic).

### Sensing triggers (wiring, in the runner)

- at every world truth change: robots sense in declaration order at the
  change tick;
- at every movement arrival: the robot senses before its next departure
  decision (a sensed change can replan and change that departure).
- No periodic sensing and no t=0 sensing: initial truth equals the base
  map, so there is nothing to sense at start.

### Scenario surface

- `"sensing": {"mode": "perfect_local"}` — opt-in; `"none"` is the
  explicit off value; unknown modes are rejected with the supported
  list. Adding a mode = new ObservationModel implementation + loader
  entry.
- New action `set_world_edge_state {edge, state}`: what actually
  happened. **Invariant: world evolution is independent of sensing
  configuration.** The action is valid with any (or no) sensing; sensing
  only determines whether and when world truth becomes robot knowledge.
  A world change with no sensor leaves belief diverged from truth — a
  first-class sovereign-autonomy scenario, locked by test
  (`WorldCanChangeWithoutRobotLearning`).
- `observe_edge` REMAINS as a low-level testing primitive: scenarios
  and tests can still inject an observation directly to exercise
  reconciliation, confidence and replanning without a world/sensor
  setup. It is no longer the preferred way to describe physical events.

### Perception vs knowledge management

- `ObservationModel::sense(world, robot_state)` is PURE PERCEPTION: it
  measures what is currently observable and has NO belief input. A
  physical sensor does not ask what the robot believes before measuring;
  keeping belief out of the sensor contract is what lets later work
  (noise, repeated measurements, confidence accumulation, filtering,
  SLAM-style re-observation) extend the model instead of fighting it.
- Suppression of unchanged facts (a measurement equal to current belief
  is not forwarded) lives in the simulation wiring — the M2 observation
  processor. It is policy, not perception, and will be replaced by a
  real processor/filter without touching the sensor boundary.
- `PerfectLocalEdgeSensor` therefore reports the measured truth of ALL
  observable edges, including ones matching belief.

### Trace taxonomy extension (amends ADR-009/010 lists)

- `world_edge` {edge, status}, source "world": a truth change.
- Sensor-generated `observation` events carry `origin=sensor`; scripted
  `observe_edge` events keep no origin field (absence = scripted). This
  keeps pre-existing traces byte-identical while making the physical
  path visible.

## Alternatives considered

- **Robots query the world directly.** Rejected: destroys the boundary;
  every future sensor improvement would become robot logic.
- **World changes require a sensor.** Rejected: the world must evolve
  independently of observers — "truth changed, nobody noticed" is a
  first-class scenario, not an error.
- **Belief as a sensor input (belief-gated sensing).** Rejected: mixes
  perception with knowledge management; noise, repeated measurements and
  filtering would have to un-learn the belief dependency. The sensor
  measures; the wiring decides what is new.
- **Sensor emits only changed facts continuously.** Rejected: same
  belief dependency in disguise.
- **Sensing wakes robots reactively on knowledge changes.** Not needed:
  sensing runs at world changes and arrivals — the only moments truth
  or position change under this model.
- **Continuous range models (radius in meters, line of sight).**
  Rejected for Stage 0: node-incident range is the minimal model that
  establishes the boundary; richer ranges arrive with geospatial
  grounding (#12) and sensor models later.

## Consequences and limitations

- A stationary robot learns only about its incident edges; everything
  else reaches it through the fleet channel (which is the point).
- While in transit, a robot does not learn changes at its destination
  node until arrival (deterministic, physical, cheap).
- World truth is global truth; per-region truths or partial observability
  of truth itself are future work.
- `observe_edge` bypasses the world entirely by design — reviewers
  should treat its use in new scenarios as a smell unless it is a test
  primitive.

# ADR-018: GNSS outage, retained estimates and the truth-pose adapter

* Status: Accepted
* Date: 2026-09-20
* Scope: `fleet::localization` (`LocalizationTracker`), `fleet::robot`
  (`RobotTransit::departed_at`, robot-local estimate storage),
  `fleet::world` (`truth_pose`), `fleet::scenario` (localization settings,
  truth-orientation maintenance, GNSS model selection, deterministic
  sampling, trace); M3 #16

## Context

ADR-016 established the truth/estimate boundary and ADR-017 established
deterministic GNSS degradation, but no scenario sampled GNSS during a run:
estimates existed only inside tests.

M3 #16 makes localization degradation observable in a running scenario.
When GNSS stops producing fixes, the robot's position knowledge must become
stale visibly:

```text
simulation truth continues to evolve
            |
            v
GNSS produces no fix
            |
            X
last valid LocalizationEstimate remains
            |
            v
age = now - estimated_at
```

The estimate must not disappear, and it must not be refreshed through a
hidden path to simulation truth.

This ADR records the durable contracts that M3 #17 dead reckoning and
M3 #18 reacquisition may rely upon.

## Decision

### 1. The retained localization estimate is robot-local

`localization::LocalizationTracker` owns the last valid
`LocalizationEstimate`.

Its contract is:

* `apply_sample(estimate)` replaces the retained estimate;
* `apply_sample(nullopt)` retains the previous estimate exactly;
* before the first successful fix there is no estimate;
* `age_at(now)` is derived from `now - estimated_at`, never stored as
  independent mutable state;
* querying age before `estimated_at` is rejected with
  `std::invalid_argument`;
* non-finite estimates are rejected.

The tracker is owned by `Robot` behind the narrow APIs:

```text
apply_gnss_sample(...)
localization()
```

This resolves ADR-016's deferred estimate-store ownership decision.

Retention represents robot-local belief, therefore it lives with the robot
rather than as hidden `ScenarioRunner` state.

The dependency remains one-directional:

```text
fleet::robot
    |
    v
fleet::localization
```

`fleet::localization` has no dependency on robots, scenarios, world truth,
or the event queue.

---

### 2. Ground-truth pose is derived only on the simulation side

`world::truth_pose(base, robot_state, at_rest_heading_rad, now)` is the
simulation-side adapter from graph movement state and geographic map geometry
to `localization::GroundTruthPose`.

Robot autonomy never receives `GroundTruthPose`, `fleet::world`,
`ScenarioRunner`, or the GNSS model.

The information flow remains:

```text
simulation movement + map geometry
              |
              v
       GroundTruthPose
              |
              v
          GnssModel
              |
              v
optional<LocalizationEstimate>
              |
              v
   robot-local LocalizationTracker
```

#### Position during transit

For a transit over:

```text
[departed_at, arrival]
```

normalized progress is derived from elapsed logical time.

Planning cost is not interpreted as physical distance.

That normalized progress is mapped onto the edge geometry using cumulative
physical polyline length, computed with the same haversine class used by the
existing geospatial boundary.

Progress is therefore:

```text
movement timing fraction
        |
        v
polyline physical arc length
```

and never:

```text
planning cost -> meters
```

or:

```text
polyline point index
```

If an edge has no stored polyline, its two endpoint coordinates form the
straight-segment geometry fallback already established by ADR-012.

Traversal respects actual physical travel direction. A traversal from the
canonical edge's `b` endpoint toward `a` walks the same stored geometry in
reverse travel order without mutating the canonical `MapGeometry`.

At progress 0 and 1, truth positions are the bit-exact canonical endpoint
coordinates.

At an interior geometry vertex, segment ownership is deterministic:
segments use half-open arc-length intervals `[start, end)`, while the final
segment additionally owns its endpoint.

#### Heading during transit

Heading is the great-circle initial bearing of the active polyline segment
in the actual direction of travel.

#### Stationary orientation

Stationary orientation is real simulation truth and is not manufactured by
resetting heading to north.

Each localization-enabled robot begins with the explicit physical initial
condition:

```text
heading = 0 rad
        = north
```

`ScenarioRunner` maintains this physical orientation on the simulation side.

Immediately before a transit is committed at arrival, the final-segment
travel bearing of that transit becomes the robot's retained physical
orientation.

Therefore:

```text
initial state
heading = north
      |
      v
travel along edge
heading = live travel bearing
      |
      v
arrival
      |
      v
stationary heading = completed traversal's final bearing
```

A robot does not reset to north merely because it becomes stationary.

When it later departs along another graph edge, heading may change
instantaneously to that edge's travel bearing. This is an intentional
graph-kinematic abstraction: steering dynamics, turn radius, wheel angles,
and vehicle physics are not modeled in M3 #16.

This simulation-truth orientation is not stored in `RobotState` and is never
exposed as oracle information to robot autonomy.

If geometry is degenerate and no travel bearing can be derived, the existing
simulation-side physical orientation is retained rather than replaced with a
fabricated direction.

#### Required movement-state addition

Truth-pose interpolation requires the beginning of the movement interval.
Therefore `RobotTransit` gains:

```text
departed_at
```

alongside its existing arrival time.

This does not change movement behavior; it only makes the already-existing
movement interval explicit enough to derive truth pose.

Missing geographic geometry, missing required node coordinates, inconsistent
transit endpoints, or otherwise invalid truth-pose inputs fail explicitly
rather than manufacturing coordinates.

---

### 3. Localization is opt-in and uses one GNSS model seam

Scenario localization configuration is:

```json
"localization": {
  "gnss": {
    "period_ms": 1000,
    "initial_model": "perfect"
  }
}
```

`initial_model` uses one grammar:

```text
"perfect"

"unavailable"

{
  "noisy": {
    "position_axis_sigma_m": 2.5,
    "heading_sigma_rad": 0.1
  }
}
```

The same grammar is used by:

```text
set_gnss_model
```

There is one scenario parser and one runner-side model factory.

Model behavior itself remains in `fleet::localization`:

```text
PerfectGnss
UnavailableGnss
NoisyGnss
```

An outage remains:

```text
active GnssModel returns nullopt
```

There is no separate outage flag or outage special case in the tracker,
robot, or runner sampling path.

Rules:

* absence of the localization block means localization is completely off;
* localization requires a finite scenario `duration_ms`, because periodic
  sampling is a self-rescheduling chain;
* `period_ms >= 1`;
* localization requires geographic `MapGeometry`;
* the loader and the `ScenarioRunner` constructor both enforce the relevant
  configuration invariants;
* `set_gnss_model` without enabled localization is rejected;
* changing model does not itself produce a measurement.

Existing scenarios without localization remain behaviorally unchanged.

The founding `station_partition` scenario remains byte-identical against the
pre-#16 repository state.

---

### 4. GNSS sampling and same-tick model-switch semantics

The first GNSS measurement is scheduled at tick 0.

Each later sample occurs exactly:

```text
period_ms
```

after the previous sample.

`period_ms >= 1` guarantees that periodic localization cannot form a
zero-time self-scheduling loop.

Changing the active GNSS model:

* does not trigger an immediate sample;
* consumes no randomness itself;
* affects subsequent scheduled measurements.

Same-tick behavior remains based on ADR-005 `EventQueue` enqueue order.
M3 #16 deliberately locks the GNSS-specific guarantee:

> A scripted `set_gnss_model` event at tick `T` takes effect before a GNSS
> sample scheduled for the same tick `T`, including `T = 0`.

Therefore:

```text
set_gnss_model @ T
        |
        v
GNSS sample @ T uses the new model
```

This rule is test-locked both at tick 0 and at later sampling ticks.

M3 #16 does not introduce a new priority scheduler or a universal priority
ordering among all scenario event classes.

Movement continues to use the existing event-chain semantics. When a GNSS
sample coincides with an already-scheduled movement arrival, the movement
transition is committed before that GNSS sample; if movement immediately
begins the next edge, the sample observes that resulting movement state.

---

### 5. GNSS randomness uses independent deterministic per-robot streams

A single shared GNSS RNG would make one robot's measurement noise depend on
whether another robot happened to sample first.

That coupling is rejected.

Each robot owns an independent GNSS `DeterministicRng` stream.

Its seed is derived from:

```text
resolved scenario seed
        +
fixed GNSS domain
        +
stable RobotId
```

through:

```cpp
derive_stream_seed(resolved_seed, kGnssStreamDomain, robot_id)
```

The fixed GNSS domain is:

```text
"GNSS_STR"
0x474E53535F535452
```

`derive_stream_seed` uses fully specified unsigned 64-bit arithmetic and a
fixed SplitMix64-style mixing step.

It does not use:

```text
std::hash
wall-clock state
addresses
unordered-container iteration
library distributions
```

Golden-value tests pin the derivation algorithm.

The resulting contract is:

> Adding, removing, or sampling an unrelated robot does not perturb another
> robot's GNSS noise sequence, provided that robot's identity, scenario
> inputs, and resolved seed are unchanged.

Within each robot-local stream, the existing ADR-016/017 consumption rules
remain unchanged:

* `PerfectGnss` consumes zero draws;
* `UnavailableGnss` consumes zero draws;
* zero-noise `NoisyGnss` consumes zero draws;
* nonzero `NoisyGnss` consumes exactly the already-defined number of draws
  for its enabled noise components;
* changing models consumes no draws;
* no RNG warm-up is performed.

Thus noise evolution depends only on actual noisy measurements for that robot,
not on GNSS activity elsewhere in the fleet.

## Trace observability

`gnss_sample` records robot-local belief:

```text
outcome = fix | no_fix

when an estimate exists:
    estimated_at
    age
    position
    heading
```

Truth coordinates are deliberately not included in the robot-local
localization event.

`gnss_model` records explicit model changes.

Position and heading are serialized as fixed six-decimal strings to preserve
useful geographic visibility in the deterministic trace.

The console command:

```text
robot <name>
```

shows the tracker read-only:

```text
localization:
    estimate: unavailable
```

or, when available:

```text
localization:
    estimated_at: ...
    age_ms: ...
    position: ...
    heading: ...
```

No independent console-side localization state exists.

## Alternatives considered

### Runner-owned retained localization estimates

Rejected.

The retained estimate represents robot-local belief, not orchestration state.
Keeping it only inside `ScenarioRunner` would put autonomy state in the wrong
ownership domain.

### Continuous physical position stored inside `RobotState`

Rejected.

ADR-010 intentionally keeps graph movement state compact. Geographic truth
can be derived from movement interval + map geometry + logical time without
turning physical simulation truth into robot autonomy state.

### Reset stationary heading to north

Rejected.

It creates a fictitious orientation discontinuity at every arrival.

North is only the explicit initial physical orientation. After movement,
stationary orientation retains the completed traversal's final bearing.

### Store exact simulation-truth heading in Robot

Rejected.

That would create an oracle path from simulation truth into robot autonomy.

Physical orientation required solely for truth generation remains on the
simulation side.

### Introduce steering or vehicle dynamics

Rejected for M3 #16.

The current graph-motion abstraction permits instantaneous orientation changes
when a new traversal begins. Ackermann steering, wheel angles, turn radius,
and physics belong to a later physical-simulation boundary.

### Immediate GNSS sample on model switch

Rejected.

Model selection is scenario policy. A switch changes the model used by the
next periodic sample and does not create an additional measurement.

### One globally shared GNSS RNG

Rejected.

It would couple Robot A's noise history to Robot B's sampling activity.

Per-robot deterministic streams provide reproducible fleet composition
independence while preserving exact per-model draw-consumption contracts.

## Consequences and limitations

During a GNSS outage:

```text
truth position          continues evolving
LocalizationEstimate    remains unchanged
estimated_at             remains unchanged
age                      increases with logical time
```

This frozen estimate is the intentional M3 #16 baseline.

M3 #17 may introduce dead reckoning that advances the estimated state while
error accumulates. That future estimator must remain downstream of the
truth/measurement boundary established here.

M3 #18 may define reacquisition behavior when GNSS becomes available again.

Graph movement does not model continuous steering through intersections;
orientation may change instantaneously at a new traversal.

Great-circle bearings between same-latitude points differ slightly from a
constant-rhumb east/west heading. Tests use the same geographic model and
appropriate tolerances.

The demo/test grid now carries canonical WGS84 coordinates for
localization-capable scenarios. Planning remains independent of geographic
geometry.

## Validation

M3 #16 is accepted only with:

* full debug test suite passing;
* ASan + UBSan passing;
* TSan passing using the repository's established execution workaround;
* `git diff --check` clean;
* founding pre-localization scenario byte-identical against pre-#16 `HEAD`;
* `gnss_outage.json` byte-identical across repeated runs with the same seed;
* per-robot GNSS stream derivation pinned by golden tests;
* an integration regression proving unrelated robot GNSS sampling does not
  perturb another robot's noisy measurement sequence.

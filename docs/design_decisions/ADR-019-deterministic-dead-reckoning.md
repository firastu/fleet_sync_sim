# ADR-019: Deterministic robot-local dead reckoning

* Status: Proposed (implemented; awaiting #17 review)
* Date: 2026-09-21
* Scope: M3 #17; localization propagation, robot-local graph odometry,
  opt-in scenarios and outward diagnostics

## Context

ADR-018 retains a frozen estimate during GNSS loss. M3 #17 needs a contrasting
failure mode: relative motion advances belief while systematic error grows.
This must not introduce a filter or a hidden absolute-truth correction.

## Decision

### Relative motion, not absolute truth

`LocalizationTracker::propagate(OdometryIncrement, DeadReckoningConfig)` takes
distance in meters, a clockwise relative turn in radians, and interval
timestamps. It has no robot, world, event-queue or sensor-model dependency.
Propagation without a prior fix does not manufacture an estimate.

`Robot` derives increments from its own committed traversal timing and the
immutable geographic map it already holds. Planning cost determines traversal
duration, never physical meters. Directed polyline arc lengths and bearings
are cached once at departure, with endpoint geometry as the fallback. Reverse
travel reverses segment order and bearings; zero-length segments are skipped.
The graph-command heading cursor is not a measured absolute heading: only its
relative changes enter belief. An erroneous estimated heading remains erroneous.
No `GroundTruthPose` or `fleet::world` API is available to this path.

This is commanded-motion dead reckoning in the reference graph model, not a
wheel/IMU sensor simulation. Slip, actuator failure and unknown motion are not
modeled. A future explicit odometry adapter can supply the same relative
increments without replacing the propagation contract.

### Deterministic accumulating bias

Configuration:

```json
"localization": {
  "gnss": { "period_ms": 250, "initial_model": "perfect" },
  "dead_reckoning": {
    "distance_scale_error": 0.05,
    "heading_drift_rad_per_m": 0.0005
  }
}
```

Both fields default to zero. Scale error must be finite and greater than -1;
heading drift is finite and signed. Unknown fields and nonnumeric values fail.
The typed runner validates configuration as well as the JSON loader.

For an increment of commanded distance `d` and turn `turn`:

```text
heading += turn
propagated distance = d * (1 + distance_scale_error)
heading drift      = d * heading_drift_rad_per_m
```

The propagated distance is divided into `max(1, ceil(distance / 10))` equal
steps. Each uses the midpoint heading of its equal drift increment and
`apply_en_displacement` anchored at the current ESTIMATE, then advances heading.
Headings normalize to [0, 2*pi). A stationary increment advances the represented
time without adding position or heading drift. It does not snap to a map node.

Negative/non-finite distances, non-finite turns, backward intervals,
noncontiguous known-estimate intervals, overflow, and propagated distances over
1,000,000 meters per increment fail explicitly. Pole-domain failures retain
ADR-017's explicit exception behavior. A failed propagation does not commit
partial tracker state. Same-tick ordered increments are allowed for graph
turns and the segment pieces of a single update.

These are fixed systematic biases, not white noise or a random walk. They
consume ZERO RNG draws for every configuration. Therefore GNSS streams and
unrelated robots are unaffected. Same scenario and resolved seed reproduce
the same trace on the same toolchain/platform; cross-libm bit identity is not
claimed. Seed effects enter through existing noisy fixes, not new randomness.

### Update timing and retention

Dead reckoning is separately opt-in; absent means the exact ADR-018 baseline.
Configure a robot before movement or its first fix. It propagates:

* before completing a traversal, so unsampled intermediate edges are not lost;
* at departures, including same-tick relative turns;
* before each GNSS sample, using the same period as GNSS;
* up to an incoming fix's timestamp before replacing its propagation baseline.

Interior geometry corners are processed in order even if several lie between
updates. Segment ownership is half-open, with the final endpoint retained.
No separate periodic chain is introduced. Equal ticks keep ADR-005 enqueue
order and ADR-018's loaded-model-switch guarantee, not universal priorities.
Inspection and arbitrary `run_until` boundaries do not propagate state.

An update's `estimated_at` is the time represented by its propagated state.
`last_fix_at` remains the last absolute fix's timestamp, and `dead_reckoned`
identifies propagated belief. Age remains derived; a low estimate age is not
a claim of positioning accuracy. `apply_sample(nullopt)` itself still retains
state exactly. A successful fix still replaces belief per ADR-018, with no
smoothing, rejection, covariance, or new #18 reacquisition policy.

The runner rejects localization without a duration horizon. Localization does
not change planning, traversal timing, map reconciliation, or network behavior.

### Outward observability

For opted-in scenarios, `gnss_sample` adds belief-only `estimate_source`,
`last_fix_at`, and `last_fix_age` fields when an estimate exists. The existing
position, heading, represented timestamp and age remain available.

A separate `world` / `localization_error` event carries robot identity and
haversine `position_error_m` against current simulation truth. This diagnostic
is computed outside autonomy; it never corrects belief. The console shows
source, last-fix age, and a separately labeled simulation diagnostic. Read-only
inspection has no scheduling, RNG or state effect. Legacy structured traces
remain unchanged when dead reckoning is absent.

## Alternatives rejected

* Add a bias to the current truth pose: this would erase accumulated error.
* Snap belief to node coordinates at arrival: this is hidden absolute correction.
* Derive displacement between occasional GNSS samples: this loses curved paths
  and intervening traversals and cannot operate during an outage.
* Implement an EKF or fuse measurements: #18 review must precede that decision.
* Add stochastic drift immediately: fixed bias already demonstrates cumulative
  error without inventing additional random draw contracts.

## Consequences and validation

Zero bias is a local-tangent engineering approximation, not a promise of exact
identity with interpolated geographic truth. Bias errors need not grow
monotonically on arbitrary turning or returning routes. Straight-line tests
lock increasing error; bent/reverse tests lock relative integration and no
arrival correction. Direct tests also cover missing fixes, invalid inputs,
timestamp continuity, stationary behavior, stream independence, stepped versus
one-shot execution, serialization, and observer noninterference.

The scenario fixture contrasts with `gnss_outage.json`; no estimator,
cooperative localization, external simulator, new dependency or thread is added.
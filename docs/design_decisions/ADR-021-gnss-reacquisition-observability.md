# ADR-021: GNSS reacquisition observability

* Status: Proposed (implemented; awaiting #18 review; owner promoted #18 on
  2026-09-21 after accepting #17/ADR-019)
* Date: 2026-09-21
* Scope: M3 #18; the fix-returning transition of robot-local localization
  state; trace observability. No estimator, no fusion, no new randomness.

## Context

ADR-018 freezes belief during an outage and lets a returning fix REPLACE it.
ADR-019 advances belief by dead reckoning and again lets the returning fix
replace it, explicitly deferring "reacquisition policy" to #18. Two failure
modes now exist upstream of a returning fix (frozen-stale vs drifted), and
the moment of return is currently invisible as a transition: the trace shows
the new estimate but not what the robot just had to give up. Before deciding
on estimator architecture, the project must be able to SEE reacquisition
magnitude, timing and prior mode — the same observability-first discipline
used in #14-#17.

## Decision

### The replace contract IS the reacquisition policy

A successful fix unconditionally replaces robot-local belief, exactly as in
ADR-018/019: no plausibility gate, no weighting, no smoothing, no
uncertainty representation. Outlier handling and fusion are estimator
questions, gated behind the post-#18 review, and require their own ADR.

### `FixApplication` — one outcome per applied sample

`LocalizationTracker::apply_sample` (and the `Robot::apply_gnss_sample`
pass-through) now RETURNS a pure-data outcome:

```text
fix                    a fix arrived and replaced belief
reacquisition          first fix after >= 1 missed samples AND a prior
                       estimate existed (an initial acquisition is not one)
since_last_fix_ms      gap to the replaced fix's timestamp (nullopt without
                       a prior fix)
correction_distance_m  haversine from the prior belief to the fix
correction_heading_rad bearing of that correction, [0, 2*pi) (nullopt with
                       no prior or exactly zero distance)
prior_dead_reckoned    the replaced belief had been propagated (drifted),
                       not frozen
```

The correction is measured from the FULLY-PROPAGATED prior: the robot
applies `advance_localization(fix.estimated_at)` BEFORE the fix replaces
belief. The committed end state is identical to the ADR-019 ordering (fix
replaces; odometry baselines advanced); only the previously unobservable
intermediate becomes the measurement baseline. Missed samples are counted,
never acted on; with no prior estimate, misses do not manufacture an
acquisition. The outcome consumes zero randomness and cannot be reached
from autonomy or truth — it describes belief only.

### Trace and fixtures

On a reacquisition sample only, `gnss_sample` gains `reacquired`,
`prior_source` ("gnss" | "dead_reckoning"), `since_last_fix_ms`,
`correction_m` and `correction_heading_rad` (6-decimal strings like the
other localization fields). `gnss_outage.json` and `dead_reckoning.json`
never restore a fix-producing model, so their traces are byte-unchanged;
the new `gnss_reacquisition.json` fixture exercises the drifted case and an
inline no-DR scenario locks the frozen case. Console display of a stored
"last reacquisition" is deliberately deferred: the transition is an event,
and event observability lives in the trace.

## Alternatives rejected

* An innovation/outlier gate rejecting "implausible" fixes: the first step
  onto the estimator ladder; #18 exists to make such judgments OBSERVABLE
  first.
* Storing a "last reacquisition" record in tracker state for consoles:
  derived transition data does not justify persistent state; the trace is
  the event record.
* Computing the correction from simulation truth: reacquisition is a
  belief-to-belief jump; the world-side `localization_error` diagnostic
  already covers truth comparison.
* A dedicated `gnss_reacquisition` event type: it would split one sample's
  story across two events; optional fields on `gnss_sample` keep the
  one-sample-one-event shape of ADR-018.

## Consequences and validation

Reacquisition observability arrives with zero behavior change: same
scenario + seed reproduce byte-identical traces everywhere no reacquisition
occurs (all founding scenarios and both existing localization fixtures),
locked by tests. The propagated-prior baseline makes `correction_m` the
honest "belief jump" of ADR-019's replace step. Direct tests cover first
acquisition, missed-then-fix, consecutive fixes, stationary zero
correction, dead-reckoned prior flag, and no-prior misses; scenario tests
lock the trace fields, the frozen/drifted contrast and the untouched
existing fixtures. Haversine/bearing use the repository's shared mean-earth
constant and libm class (same-toolchain bit identity, like ADR-017/019).

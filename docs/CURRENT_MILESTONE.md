# Current Milestone

This document defines normal implementation scope.

For historical architecture decisions, read the relevant ADRs rather than
reconstructing previous milestones here.

---

## Closed milestones

### M1 — Deterministic distributed reference simulator

**COMPLETE**

Established deterministic scenarios, robot-local dynamic map knowledge,
unreliable communication, partitions, reconciliation, autonomous replanning,
ControlStation synchronization, and byte-reproducible structured traces.

Closed by ADR-009 / commit #9.

### M2 — Movement, sensing and geospatial grounding

**COMPLETE**

Added movement and replan-from-position, world truth versus robot belief,
position-based sensing, WGS84 map geometry, one-way topology, deterministic OSM
import, GeoJSON export, and the interactive console.

Key decisions: ADR-010 through ADR-015.

---

# Milestone M3 — Resilient positioning / localization

**Status: IN PROGRESS**

M3 studies what robot-local position knowledge means when absolute positioning
is imperfect or unavailable.

The architectural separation is:

```text
GROUND TRUTH POSE
       |
       | sensor / measurement model
       v
LOCALIZATION ESTIMATE
```

Robot autonomy must not obtain ground-truth pose by bypassing this boundary.

---

## Completed

### #14 — Localization boundary — ADR-016

Introduced:

```text
GroundTruthPose
LocalizationEstimate
GnssModel
PerfectGnss
UnavailableGnss
```

`LocalizationEstimate::estimated_at` is the time represented by the estimate,
making estimate age and staleness observable.

No estimator was selected.

### #15 — Deterministic noisy GNSS — ADR-017

Introduced `NoisyGnss`.

Position noise is generated in local East/North meters using deterministic
midpoint-centered Irwin-Hall(12) sampling.

Public configuration:

```text
position_axis_sigma_m
heading_sigma_rad
```

Random draw consumption is fixed by contract.

Zero noise reproduces the perfect model and consumes no randomness.

The metric-to-WGS84 boundary wraps the antimeridian and explicitly rejects
unsupported pole-domain behavior rather than clamping.

The model is an engineering degradation model, not a high-fidelity physical
GNSS receiver model.

### #16 — GNSS outage and stale localization — ADR-018

Made localization degradation observable inside scenarios:

```text
GNSS fix
   |
   v
retained robot-local estimate (LocalizationTracker on Robot)

GNSS unavailable (active model returns nullopt)
   |
   X  no new fix
   |
estimate retained unchanged; age = now - estimated_at grows
```

- scenario opt-in `"localization"` block (period, initial GNSS model) and
  `set_gnss_model` action — one parser, one model factory; outages stay
  "which model is active";
- simulation-side truth pose (`world::truth_pose`) derived from movement
  timing + MapGeometry arc length, direction-aware (reverse traversal),
  explicit failure when geometry is missing — never manufactured;
- sampling from tick 0 at a fixed period, strictly later rescheduling;
  equal ticks retain enqueue order, with loaded scripted model switches
  preceding the same-tick sample (including tick 0), as defined by ADR-018;
  there is no universal movement/script/sample priority;
- `gnss_sample` / `gnss_model` trace events (belief-side fields only,
  including derived age) and console `robot <name>` localization state;
- zero-consumption RNG contracts preserved; same scenario + seed is
  byte-identical; founding trace unchanged (localization is opt-in).

---

## Current objective

### #17 — Deterministic dead reckoning / drift

**Implemented; awaiting review.** Contract and limitations are recorded in
[ADR-019](design_decisions/ADR-019-deterministic-dead-reckoning.md).
The opt-in implementation uses robot-owned graph motion, fixed distance/heading
bias, last-fix age, and outward-only position-error diagnostics. #18 is not
promoted until this step has been reviewed.

Advance robot-local estimated state during a GNSS outage from the
robot's OWN motion, with accumulating error — the frozen #16 estimate
is the baseline it diverges from.

The work should cover:

* a deterministic motion-propagation model behind the localization
  boundary (no oracle truth: graph movement knowledge the robot itself
  holds, or an explicit odometry seam);
* error that accumulates deterministically (same scenario + seed =>
  same drift);
* the conceptual contrast locked by tests:

```text
#16 outage:  estimate frozen, estimated_at unchanged, age grows
#17 drift:   estimated state advances, error grows vs truth
```

* trace/console observability of the divergence.

Do not add an estimator (filter) to implement #17; dead reckoning is a
measurement/propagation model, not a fusion algorithm. An estimator
decision remains gated behind the #18 review.

---

## Planned M3 sequence

After #16, continue in review-gated steps:

```text
#17 deterministic dead reckoning / drift
        |
#18 reacquisition behavior
        |
review accumulated measurement/failure semantics
        |
decide whether estimator architecture is justified
```

Do not skip directly to an estimator.

An estimator decision requires an ADR explaining why the selected approach is
appropriate for the failure modes that now exist.

---

## Allowed in M3

Work may introduce or change:

* localization-domain value types;
* deterministic positioning sensor models;
* scenario-controlled localization availability;
* estimate retention and age;
* deterministic dead-reckoning experiments when #17 is active;
* reacquisition semantics when #18 is active;
* trace and console observability for localization state;
* tests and scenario fixtures required by those behaviors;
* correctness fixes in existing subsystems.

Cross-cutting changes must preserve existing subsystem boundaries.

---

## Not currently authorized

Do not introduce merely because they appear in the project vision:

* EKF / UKF / particle-filter implementations;
* SLAM;
* visual odometry;
* LiDAR odometry;
* cooperative localization;
* distributed geometric map merging;
* ROS 2;
* NVIDIA Isaac Sim integration;
* Gazebo integration;
* physical-robot drivers;
* threads or a new asynchronous runtime;
* detailed RF propagation;
* multi-hop communication routing.

Some of these are likely future work. They are not current implementation
scope.

---

## M3 invariants

1. simulation truth and robot-local estimate remain separate;

2. truth reaches localization only through explicit models/adapters;

3. estimate age is observable;

4. sensor degradation is deterministic under the resolved scenario seed;

5. degraded positioning does not silently obtain oracle truth;

6. randomness continues through `DeterministicRng`;

7. scenario + resolved seed preserves deterministic reference behavior;

8. existing map, network, reconciliation, and autonomy contracts remain
   independent of localization implementation details.

---

## Validation gate

Before review of an M3 feature:

* debug passes with warnings as errors;
* ASan+UBSan passes;
* TSan passes using the repository's established per-process workaround;
* new localization behavior has direct tests;
* orchestration changes have runner/scenario tests;
* `git diff --check` is clean;
* founding-trace output remains byte-identical unless the feature intentionally
  changes that scenario.

---

## Milestone transition

After #18, review the resulting localization semantics before deciding whether:

1. estimator work remains part of M3;
2. M3 closes;
3. a physical-simulation integration milestone begins.

Future work is promoted explicitly rather than inferred from the project
vision.

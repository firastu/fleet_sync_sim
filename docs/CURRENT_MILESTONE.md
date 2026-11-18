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

**COMPLETE — reviewed by the owner on 2026-09-21.** Contract and limitations
are recorded in
[ADR-019](design_decisions/ADR-019-deterministic-dead-reckoning.md).
The opt-in implementation uses robot-owned graph motion, fixed distance/heading
bias, last-fix age, and outward-only position-error diagnostics.

### #18 — GNSS reacquisition behavior

**Implemented; awaiting review.** Contract recorded in
[ADR-021](design_decisions/ADR-021-gnss-reacquisition-observability.md).

A fix returning after an outage (or after drifted dead reckoning) already
replaces belief per ADR-018. #18 makes that TRANSITION a first-class
observable without changing what it does:

* one `FixApplication` outcome per applied sample: fix/no-fix, reacquisition
  (first fix after at least one missed sample, with a prior estimate), gap
  since the replaced fix, correction distance and bearing from the
  fully-propagated prior belief, and whether that prior was dead-reckoned;
* `gnss_sample` trace fields on reacquisition samples only
  (`reacquired`, `prior_source`, `since_last_fix_ms`, `correction_m`,
  `correction_heading_rad`) — existing localization fixtures never
  reacquire, so their traces are unchanged;
* the replace contract itself is unchanged: no rejection gate, no fusion,
  no covariance — the estimator decision stays gated behind the
  post-#18 review;
* a `gnss_reacquisition.json` fixture contrasting drifted and frozen priors.

After #18: review the accumulated measurement/failure semantics, then decide
whether estimator architecture is justified (its own ADR).

Do not add an estimator, outlier rejection or uncertainty representation as
part of #18.

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

---

# Proposed next milestone — M4 Terrain-referenced navigation without GNSS

**Status: DRAFT for owner review (2026-11-18) — NOT authorized.**

Nothing in this section is implementation scope until the owner has reviewed
#18, accepted this draft, and promoted it to the active milestone. Until then,
the M3 rules above govern.

## Objective

Off-road ground robots and drones keep navigating when GNSS is jammed, spoofed
or absent, by matching sensed terrain height against a locally held elevation
map — the principle behind aircraft terrain-referenced navigation.

```text
truth pose + truth terrain (world)       robot terrain map (robot-local)
            |                                       |
      sensor models                                 |
  (baro, rangefinder, velocity, gyro)               |
            |                                       |
            +-------------> estimator <-------------+
                                |
                  LocalizationEstimate + spread
                                |
                       guidance (from #23)
```

The deterministic simulator remains the behavioral reference: same scenario +
resolved seed produces byte-identical traces and scorecards. A physical
backend is a candidate for the milestone after M4, not part of it.

## Why the estimator decision is justified here

M3 deferred an estimator until failure modes justified one. Terrain matching
supplies them: the terrain likelihood is nonlinear, and flat or repeating
terrain yields several equally good position hypotheses at once. The current
single-hypothesis, replace-on-fix tracker cannot represent that ambiguity.
#22 selects an estimator that can, in its own ADR.

## Relationship to the existing core

Unchanged: the deterministic event core, `DeterministicRng` and stream
derivation, scenario and trace contracts, networking and reconciliation, the
truth/belief separation, the localization boundary and the GNSS models.

Added beside the road-graph domain, not replacing it: a terrain motion domain.
A scenario selects one domain; mixing domains in one scenario is outside M4.
Road-graph scenarios, the founding trace and the existing localization
fixtures stay byte-identical.

## Planned M4 sequence

### #19 — Terrain elevation model — own ADR

* immutable georeferenced elevation grid in meters, bilinear interpolation;
* lookups outside coverage or on nodata fail explicitly; never clamp,
  extrapolate or zero-fill;
* heights stay in the source DEM's vertical datum; no implicit geoid
  conversion;
* surface-type metadata: surface model (treetops and roofs, what a downward
  rangefinder sees) versus bare-earth model (what a ground robot drives on);
* first file format: SRTM `.hgt` (plain big-endian 16-bit grid, no new
  dependency); GeoTIFF sources such as Copernicus GLO-30 only if justified,
  with any reader kept `PRIVATE`;
* two roles from the start: world truth terrain and each robot's own terrain
  map, separate instances even when loaded from the same file;
* the ADR defines the metric frame for kilometer-scale motion;
  `apply_en_displacement` is only specified for meter-to-tens-of-meters
  displacements (ADR-017);
* tests use small synthetic grids committed as fixtures; full DEM tiles are
  not committed.

### #20 — Free-space terrain motion (scripted truth) — own ADR

* missions are waypoint lists (latitude, longitude, plus altitude for air
  platforms) instead of graph start/goal;
* deterministic kinematics: constant speed and bounded turn rate along
  waypoint legs, integrated at a fixed integer-tick step; no physics, wind or
  wheel slip;
* platform variants: ground (height = truth terrain + mount height) and air
  (constant-altitude legs in the DEM datum);
* truth pose gains altitude; the road-graph pose path is unchanged;
* motion is scripted truth, independent of belief, as in M3, until #23;
* no terrain-aware path planning: legs are straight.

### #21 — Relative-motion and altitude sensors — own ADR

Deterministic noisy models, each on its own per-robot RNG stream derived from
the resolved seed, with fixed draw consumption; zero-noise configurations
consume no draws:

* velocity/odometry sensor with scale error and noise — a stand-in for wheel
  odometry, visual odometry or optical flow; slip and image texture are not
  modeled;
* gyro heading with bias random walk;
* barometric altitude with drifting bias and noise;
* downward rangefinder for air platforms: range from truth altitude to the
  truth terrain surface, with noise, maximum range and dropouts.

Sensors read truth pose and truth terrain; robots receive only sensor outputs.

### #22 — Terrain-referenced estimator — own ADR (the estimator decision)

* the ADR compares at least batch profile correlation (TERCOM-style), a
  grid/point-mass filter (deterministic without random draws) and a particle
  filter, and records why the selected approach fits the ambiguity above;
* measurement: terrain height under the vehicle (air: baro altitude minus
  range; ground: baro altitude minus mount height) compared with the robot's
  own map; baro bias is estimated, not assumed known;
* how GNSS fixes enter the estimator, replacing ADR-018's replace-on-fix rule
  for terrain scenarios, is part of this ADR; a GNSS-free start requires an
  explicit prior region;
* output: `LocalizationEstimate` plus the first uncertainty/spread
  representation, introduced by this ADR;
* estimator randomness, if any, uses its own derived stream with fixed
  consumption;
* traces carry summaries (estimate, spread, hypothesis health), not full
  particle or grid dumps.

### #23 — Navigate from belief

* the robot chooses commanded heading and speed from its estimate;
* `fleet::world` integrates truth motion from those commands; truth returns to
  the robot only through sensors;
* degraded behavior: hold or loiter when spread exceeds a configured bound,
  resume when it recovers.

### #24 — Scenario suite and scorecard

* fixtures: rough terrain, flat terrain (an expected failure that must be
  reported, not hidden), repeating terrain, a GNSS jamming region, spoofed
  GNSS (a new offset-fix GNSS model), robot-map error (coarser or biased
  DEM) and rangefinder dropouts;
* a seed-sweep tool producing a scorecard: horizontal error over time,
  convergence time, divergence count and mission success;
* same scenario + seed range produces a byte-identical scorecard.

## Allowed in M4 (once authorized)

* elevation-grid types, loaders and fixtures;
* the terrain motion domain and waypoint missions;
* deterministic relative-motion and altitude sensor models;
* the estimator selected by #22 and its uncertainty representation;
* belief-driven guidance and degraded-mode behavior (#23);
* scorecard tooling and outward-only visualization of terrain, truth, belief
  and spread;
* tests and fixtures for the above.

## Not authorized in M4

* Gazebo, Isaac Sim, PX4, ArduPilot or ROS 2 integration;
* physics, aerodynamics, wind or wheel-slip physics;
* image-based sensing (visual odometry or optical flow from images, SLAM);
* terrain-aware path planning;
* cooperative localization or terrain-map sharing over the network;
* threads or a new asynchronous runtime;
* GDAL or other GIS libraries in domain modules.

## M4 invariants (in addition to M3's)

1. truth terrain and each robot's terrain map are separate inputs;

2. robots read terrain only from their own map; truth terrain reaches them
   only through sensor models;

3. the estimator consumes sensor outputs and robot-local data only;
   error-versus-truth diagnostics are outward-only;

4. elevation lookups never silently extrapolate, clamp or zero-fill;

5. road-graph scenarios, the founding trace and existing localization
   fixtures stay byte-identical;

6. same scenario + resolved seed produces byte-identical traces and
   scorecards.

The M3 validation gate applies unchanged.

## After M4 (candidates, not authorized)

* physical backend, Gazebo first: world terrain generated from the same DEM,
  PX4 SITL for drones and a differential-drive model for ground robots,
  through a simulator-neutral version of the adapter design in
  [isaac/ARCHITECTURE.md](isaac/ARCHITECTURE.md);
* cooperative terrain navigation over the existing unreliable network.

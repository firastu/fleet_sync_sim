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

### M3 — Resilient positioning / localization

**CLOSED** by the owner on 2026-11-18, when M4 was promoted. The owner review
of #18 is still outstanding; findings from it are handled as correctness
fixes.

Established the localization boundary (`GroundTruthPose` versus
`LocalizationEstimate`, with an observable `estimated_at`), deterministic noisy
GNSS, scenario-controlled outages with aging estimates, opt-in dead reckoning
with deterministic drift, and observable GNSS reacquisition. No estimator was
selected; that decision moved into M4 #22.

Key decisions: ADR-016 through ADR-019 and ADR-021.

---

# Milestone M4 — Terrain-referenced navigation without GNSS

**Status: IN PROGRESS** (promoted by the owner on 2026-11-18)

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

---

## Current objective

### #19 — Terrain elevation model — ADR-022

**Implemented; awaiting review.** Contract recorded in
[ADR-022](design_decisions/ADR-022-terrain-elevation-grids.md).

* immutable `fleet::map::ElevationGrid`: a WGS84 lattice of point samples
  stored by its corners, bilinear lookups, and explicit `OutsideCoverage` /
  `NoData` statuses — never clamp, extrapolate or zero-fill;
* `ElevationSurface` (surface model versus bare earth) and `VerticalDatum`
  (`Egm96`) labels; no datum conversion;
* `fleet_dem_import` (`fleet::dem`): SRTM `.hgt` tiles identified by their
  file name, both 1- and 3-arc-second sizes, voids become NODATA, no new
  dependency;
* decided in ADR-022 for later steps: truth terrain and robot terrain maps
  are separate handles (wired in #20); positions stay WGS84, and motion
  advances them by per-step displacements through `apply_en_displacement`;
* no motion, sensing, scenario, trace or console change; every existing trace
  is unchanged.

---

## Planned M4 sequence

### #20 — Free-space terrain motion (scripted truth) — own ADR

* missions are waypoint lists (latitude, longitude, plus altitude for air
  platforms) instead of graph start/goal;
* scenario terrain block: world truth terrain and per-robot terrain maps as
  separate handles (ADR-022);
* deterministic kinematics: constant speed and bounded turn rate along
  waypoint legs, integrated at a fixed integer-tick step with bounded
  per-step displacements (ADR-022); no physics, wind or wheel slip;
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

---

## Allowed in M4

Work may introduce or change:

* elevation-grid types, loaders and fixtures;
* the terrain motion domain and waypoint missions;
* deterministic relative-motion and altitude sensor models;
* the estimator selected by #22 and its uncertainty representation;
* belief-driven guidance and degraded-mode behavior (#23);
* scorecard tooling and outward-only visualization of terrain, truth, belief
  and spread;
* tests and fixtures required by those behaviors;
* correctness fixes in existing subsystems, including findings from the
  outstanding #18 review.

Cross-cutting changes must preserve existing subsystem boundaries.

---

## Not currently authorized

* estimators other than the one selected through #22's ADR;
* SLAM, visual odometry, LiDAR odometry or any image-based sensing;
* cooperative localization, terrain-map sharing or distributed geometric map
  merging over the network;
* terrain-aware path planning;
* physics, aerodynamics, wind or wheel-slip physics;
* ROS 2, NVIDIA Isaac Sim, Gazebo, PX4 or ArduPilot integration;
* physical-robot drivers;
* GDAL or other GIS libraries in domain modules;
* threads or a new asynchronous runtime;
* detailed RF propagation;
* multi-hop communication routing.

Some of these are likely future work. They are not current implementation
scope.

---

## M4 invariants

1. simulation truth and robot-local estimate remain separate;

2. truth reaches localization only through explicit models/adapters;

3. estimate age is observable;

4. sensor degradation is deterministic under the resolved scenario seed;

5. degraded positioning does not silently obtain oracle truth;

6. randomness continues through `DeterministicRng`;

7. existing map, network, reconciliation, and autonomy contracts remain
   independent of localization implementation details;

8. truth terrain and each robot's terrain map are separate inputs;

9. robots read terrain only from their own map; truth terrain reaches them
   only through sensor models;

10. the estimator consumes sensor outputs and robot-local data only;
    error-versus-truth diagnostics are outward-only;

11. elevation lookups never silently extrapolate, clamp or zero-fill;

12. road-graph scenarios, the founding trace and existing localization
    fixtures stay byte-identical;

13. same scenario + resolved seed produces byte-identical traces and
    scorecards.

---

## Validation gate

Before review of an M4 feature:

* debug passes with warnings as errors;
* ASan+UBSan passes;
* TSan passes using the repository's established per-process workaround;
* new behavior has direct tests at the level where it lives;
* orchestration changes have runner/scenario tests;
* `git diff --check` is clean;
* founding-trace output remains byte-identical unless the feature intentionally
  changes that scenario.

---

## Milestone transition

After M4, review the terrain-navigation results before deciding what follows.
Candidates, not authorized:

* physical backend, Gazebo first: world terrain generated from the same DEM,
  PX4 SITL for drones and a differential-drive model for ground robots,
  through a simulator-neutral version of the adapter design in
  [isaac/ARCHITECTURE.md](isaac/ARCHITECTURE.md);
* cooperative terrain navigation over the existing unreliable network.

Future work is promoted explicitly rather than inferred from the project
vision.

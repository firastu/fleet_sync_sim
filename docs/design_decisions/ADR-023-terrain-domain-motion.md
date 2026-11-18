# ADR-023: Terrain-domain motion and scenarios

* Status: Proposed (implemented; awaiting #20 review)
* Date: 2026-11-18
* Scope: M4 #20; `world::TerrainMotion`, synthetic terrain (`fleet::dem`),
  `robot::TerrainRobot`, terrain-domain scenarios (`TerrainScenario`,
  `TerrainScenarioLoader`, `TerrainScenarioRunner`), scenario-domain dispatch
  and `fleet_sim`. Road-graph behavior is unchanged.

## Context

M4 robots move freely over terrain instead of along a road graph. The
existing `Robot`, `ScenarioLoader` and `ScenarioRunner` are graph-centric:
missions are node pairs, positions are nodes, movement is edge traversal. #20
must add free movement over the elevation model of ADR-022, leave every
road-graph contract and trace unchanged, and keep simulation truth separate
from what robots carry, so that sensors (#21), the estimator (#22) and
belief-driven guidance (#23) can attach later.

## Decision

### A second scenario domain, not a generalized runner

* A scenario file declares `"domain": "terrain"`. A file without `"domain"`
  (every pre-M4 scenario) is a graph scenario. `ScenarioLoader::domain_of`
  lets callers dispatch; the graph loader rejects any other domain by name.
* The terrain domain has its own data (`TerrainScenario`), loader
  (`TerrainScenarioLoader`) and runner (`TerrainScenarioRunner`). It shares
  the `EventQueue`, the trace sinks, seed resolution and ADR-005 ordering.
  One scenario uses one domain.
* `fleet_sim` runs both domains. `fleet_console` refuses terrain scenarios
  with a message until it supports them.

Generalizing `ScenarioRunner` and `Robot` would touch every graph contract and
put the founding trace at risk, while the terrain domain needs none of the
network, station or edge-sensing machinery in M4.

### Scripted truth motion: `world::TerrainMotion`

Truth motion lives in `fleet::world`. It is kinematic: constant speed, bounded
turn rate, no acceleration, physics, wind or slip. Each step of `step_ms`:

1. consecutive waypoints within `arrival_radius_m` are reached;
2. after the last waypoint the body stays put (ground: parked; air:
   hovering) and only time advances;
3. otherwise the heading turns toward the next waypoint's initial
   great-circle bearing by at most `max_turn_rate * dt`; a target exactly
   behind turns clockwise;
4. the body moves `speed * dt` along the new heading through
   `localization::apply_en_displacement`, anchored at its current position
   (ADR-022: no scenario-global projection);
5. the truth terrain below is looked up. Ground: altitude = terrain +
   `mount_height_m`. Air: a constant `altitude_m`, strictly above the
   terrain;
6. step 1 repeats at the new position.

Rules that make this well-defined:

* **Steering uses TRUE position.** This is a script for simulation truth, not
  autonomy. In #23 the robot decides commands from belief and the world
  integrates them; the integration half stays here.
* **Arrival radius >= turning diameter (2 * speed / turn rate).** A waypoint
  inside the body's turning circles counts as reached, so pursuit never
  orbits it forever.
* **Step bound: per-step displacement <= min(50 m, arrival radius).** This is
  the step bound ADR-022 left to #20. It keeps every displacement inside
  `apply_en_displacement`'s meter-to-tens-of-meters validity (ADR-017), and a
  step cannot jump over the arrival circle. The runner checks it before the
  run starts.
* **Explicit failure.** Leaving the truth terrain, reaching NODATA, or an air
  body at or below the terrain surface throws `std::runtime_error` naming the
  position, and the state is left as it was (strong guarantee). The runner
  adds the robot's name and the run fails: truth is never manufactured.
  Start conditions are validated when the runner is constructed.
* **Altitude lives in the truth state.** `TerrainBodyState` carries position,
  heading, altitude and the terrain height below. `GroundTruthPose` is
  unchanged in #20; the sensor step (#21) decides how altitude crosses the
  measurement boundary.
* **Deterministic.** Fixed operation order and libm trigonometry (the
  ADR-014/017 reproducibility class); no randomness.

### Robot side: `robot::TerrainRobot`

The robot-side participant holds its mission (the waypoint list) and its own
terrain map handle: the robot role of ADR-022. It holds no truth and has no
`fleet::world` dependency (ADR-007/011). It has no behavior in #20; sensors,
the estimator and guidance attach to it.

### Terrain-domain scenario format

* Top level: `name`, `domain`, `seed`, `duration_ms` (required),
  `motion.step_ms` (default 100), `terrain`, `robots`. Every object is strict:
  unknown fields are rejected.
* `terrain.truth` is a source; `terrain.robot_map` is `"truth"` (the default:
  robots carry the same immutable grid) or another source, which must use the
  same vertical datum. One robot map serves all robots in #20; per-robot maps
  arrive with the map-error scenarios of #24.
* Sources: `{"synthetic": {...}}` (below) or
  `{"hgt": {"file": ..., "surface": ...}}`, with the file relative to the
  scenario file.
* Robots: `name`, `id`, `platform` (`ground` or `air`), `speed_mps`,
  `max_turn_rate_deg_per_s`, `arrival_radius_m`, then `mount_height_m`
  (ground, optional) or `altitude_m` (air, required), `start`
  (`lat`, `lon`, `heading_deg`) and `waypoints`. Files use degrees; the code
  uses radians.
* No events yet: GNSS models and their switches arrive with their own steps.

### Synthetic terrain: `dem::synthesize_terrain`

Gaussian hills (or depressions, with negative height) summed over a base
height, sampled onto an `ElevationGridLayout` by haversine distance. Results
are labelled with a new `VerticalDatum::Synthetic`: heights over an arbitrary
scenario reference, comparable only with other synthetic grids. Synthetic
terrain lets fixtures and the #24 suite describe rough, flat and repeating
terrain in a few lines without committing DEM tiles.

### Trace

All terrain-domain events in #20 are simulation truth, under source
`"world"`:

* tick 0: `scenario` (`name`, `domain`), `seed`, `terrain` (`surface`,
  `datum`, and `robot_map` = `truth` or `separate`);
* `terrain_pose` (`robot`, `position`, `heading`, `altitude_m`, `terrain_m`,
  `agl_m`, `next_waypoint`) at tick 0 and after every step until the robot's
  mission completes;
* `waypoint_reached` (`robot`, `index`, `last`).

Positions and headings are 6-decimal strings, as in `gnss_sample`; heights
are the standard 2-decimal doubles.

## Alternatives rejected

* **Generalizing `Robot` and `ScenarioRunner`.** See above.
* **Analytic arcs (Dubins paths).** Exact, but far more complex. Fixed-step
  integration is simple, deterministic, and matches how sensors will sample.
* **Steering truth from belief now.** Belief-driven guidance needs sensors and
  an estimator first (#21-#23).
* **Unbounded steps or no arrival rule.** Bodies would orbit waypoints or
  leave the displacement approximation's domain.
* **Per-waypoint air altitude with instant changes.** Physically wrong;
  climb-rate modelling is deferred.
* **Committing DEM tiles as fixtures.** Size and licensing; synthetic terrain
  instead.
* **Poses only at waypoints.** Too little for replay and visualization; the
  full step cadence is the observability baseline, and the author controls
  its volume with `step_ms`.

## Consequences and limitations

* Road-graph scenarios, the founding trace and every existing trace are
  byte-identical. The graph loader's only change is a clear error for
  non-graph domains.
* `fleet_console` does not run terrain scenarios yet.
* The terrain domain has no events, GNSS or sensors yet (#21).
* Air altitude is constant per robot; there is no climb or descent.
* Steering follows truth until #23.
* Trace volume grows with robots x duration / `step_ms`.
* A body that leaves the truth terrain fails the run, by design.

Validation: `tests/unit/world/terrain_motion_test.cpp`,
`tests/unit/dem/synthetic_terrain_test.cpp` and
`tests/unit/scenario/terrain_scenario_test.cpp` pin the configuration and step
rules, constant speed, bounded and clockwise turning, waypoint order and
completion, ground and air altitude, explicit failures with unchanged state,
bit-identical replay, the synthetic surface, strict loading, hgt path
resolution and datum checks, eager runner validation, the committed
`scenarios/terrain_hills.json` run, separate robot maps, and byte-identical
traces.

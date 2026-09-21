# ADR-020: Isaac Sim stage 1 read-only replay export

* Status: Accepted (owner-authorized bounded stage, 2026-09-21)
* Date: 2026-09-21
* Scope: Isaac Sim integration stage 1 per `docs/isaac/`; outward export
  format, adapter placement, georeferencing seam; no runtime coupling to any
  simulator

## Context

`docs/isaac/` proposes a staged integration whose first stage is read-only
replay: display an already-completed FleetSyncSim run inside Isaac Sim to
validate coordinate conversion, robot identity and visualization before any
physics or sensor work. The owner explicitly promoted this bounded stage into
scope (recorded in `CURRENT_MILESTONE.md`). Stages 2 (sensor harness) and 3
(physical motion backend) remain unauthorized.

The repository needs a bridge that Isaac can consume without FleetSyncSim
acquiring any simulator dependency, and without weakening the deterministic
reference contract.

## Decision

### Observation-only export, not a second trace format

A new adapter executable, `fleet_isaac_export` (`apps/fleet_isaac_export/`),
loads a scenario against a purpose-built fixture map, executes it through the
normal `ScenarioRunner` public API (`begin()` once, then ordered `run_until()`
stepping — already contractually identical to one-shot execution), and writes
into an output directory:

```text
manifest.json    run identity: schema, scenario name, resolved seed, hashes,
                 geographic origin, frame/heading conventions, footprint,
                 tick bounds, ordered robot list
poses.jsonl      one JSON object per sampled tick: tick plus, per robot in
                 ascending numeric RobotId order, named truth fields and the
                 robot-local belief (JSON null before the first fix)
```

Export format identity is `"fleet-isaac-replay/1"`. The format is a
versioned adapter contract, deliberately separate from the founding
`TraceSink` schema (ADR-009), which is unchanged. Rules:

* every 64-bit integer (ticks, resolved seed) is a decimal string; robot ids
  are small integers;
* coordinates are named `latitude_deg` / `longitude_deg` / `heading_rad`
  (heading clockwise from north, the domain convention) with round-trip double
  precision; never an unlabeled pair and never console-rounded text;
* the snapshot domain is ticks `0..duration_ms` inclusive at a fixed period;
  the horizon must be a multiple of the period — misalignment is rejected,
  not rounded;
* missing map geometry, a missing node coordinate, or map extent beyond the
  declared footprint radius fails the export explicitly — a zero coordinate is
  never manufactured.

### Determinism and non-interference

The export observes state that already exists. Sampling:

* consumes no randomness, advances no belief, and schedules nothing —
  `run_until` boundaries were already proven behavior-neutral;
* produces a normal trace byte-identical to a plain one-shot run of the same
  (scenario, resolved seed);
* is itself reproducible: exporting the same inputs twice produces
  byte-identical `manifest.json` and `poses.jsonl` on the same toolchain.

Correlation hashes recorded in the manifest are FNV-1a 64-bit values over a
canonical hexfloat serialization of the immutable map (topology + geographic
side, id order) and over the exact scenario-file bytes. They identify inputs
across tooling; they are not checksums for integrity and not cryptographic.

### One new public seam: `ScenarioRunner::truth_pose_for(name)`

Truth pose derivation stays where it already lives: the runner holds the
simulation-side physical orientation (ADR-018) and calls
`world::truth_pose`. The exporter reads it through a single new
observation-only accessor; it does not re-derive rest headings itself, does
not mutate `Robot`, and there is no path from robot autonomy to this
information. Belief is read through the existing read-only
`Robot::localization()` view. No injection API exists (that is stage 3
territory and requires its own ADR).

### Adapter placement and build isolation

* `FLEET_BUILD_ISAAC_TOOLS` is an opt-in CMake option, default OFF. Neither
  its ON nor OFF configuration locates or links Isaac, CUDA, USD, ROS or a
  Python runtime; the exporter is pure CPU C++.
* Export logic lives in an app-local target shared with tests; no protocol or
  export type enters `include/fleet/**`. `nlohmann_json` remains PRIVATE to
  adapter targets (the existing pinned dependency, not a new one).
* The Isaac-side viewer and the pure conversion functions live in
  `tools/isaac/` and run through NVIDIA's own Python environment; they are
  importable and unit-testable without Isaac installed. Rendering requires a
  supported GPU and is a separate acceptance check, never part of CPU CTest.

### Georeferencing seam lives in the conversion layer

The native export carries only WGS84 named fields and the pinned origin.
Scene mapping happens exactly once, in `tools/isaac/conversion.py`:

```text
R = 6371008.8 m;  meters_per_degree = R * pi / 180
east  = wrapped dlon * meters_per_degree * cos(origin_lat)
north = dlat * meters_per_degree
axes: X east, Y north, Z up; 1 stage unit = 1 m
yaw (CCW from +X) = pi/2 - heading;  quaternion scalar-first (w, x, y, z)
```

The conversion layer rejects non-finite values, anchor latitudes beyond
80 degrees, and positions outside the manifest footprint radius. The fixture
is anchored at (52.370, 9.730) and fits inside 50 m. This is a bounded
engineering frame, not a general projection; wider scenes need a reviewed
projection decision (see `docs/isaac/ARCHITECTURE.md`).

## Alternatives rejected

* Parse console/trace text into poses: rounded, presentation-oriented, not a
  pose transport.
* Extend the founding trace schema with pose frames: couples the reference
  contract to an external viewer's needs.
* Reuse GeoJSON export: map geometry, not per-tick robot poses.
* Compute rest headings in the exporter by duplicating runner logic: two
  owners of one truth derivation.
* Expose truth pose on `Robot`: an autonomy-reachable oracle (ADR-007/011).
* pybind11 / DDS / sockets to Isaac for stage 1: replay needs no live
  coupling; process-protocol work belongs to stage 2 review.

## Consequences and validation

CPU-only tests cover: ascending robot ordering, tick-0 and final-horizon
snapshots, stationary/arrived heading truth, explicit failure on missing
geometry, byte-identical repeat exports, and trace non-interference versus a
plain run. Pure-Python tests cover ENU conversion against independently
derived expected values, cardinal-heading quaternions with pinned component
ordering, and export-file validation. Native presets pass with the option OFF
(baseline unchanged) and ON. GPU rendering checks are documented as unrun
until executed on a supported workstation; the documentation alone is not
evidence of a working Isaac bridge.

Stage 2 (sensor harness, process protocol, external measurement input) and
stage 3 (motion backend, odometry ownership) require separate owner promotion
and their own ADRs before implementation.

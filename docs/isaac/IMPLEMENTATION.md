# Isaac Integration Implementation Checklist

Status: stage 1 work packages 1 (native export) and the CPU-side parts of 2
(conversion functions, export loader/validator) are implemented under
[ADR-020](../design_decisions/ADR-020-isaac-stage1-readonly-replay-export.md);
the Isaac rendering path of package 2 has not run on a GPU yet. Stage 2
(sensor harness) and stage 3 (motion backend) remain proposals requiring
separate owner promotion. Read [README.md](README.md), complete
[SETUP.md](SETUP.md), and review [ARCHITECTURE.md](ARCHITECTURE.md) first
for the not-yet-implemented stages.

## Working agreement

Implement one numbered work package per review-sized change. Stop at every
stage gate. A junior developer can implement the bounded packages; an owner
must approve scope, durable contracts and the physical-motion API design.

Use these terms consistently:

* **USD stage:** Isaac's scene, containing objects called prims.
* **Articulation:** a physical robot with connected bodies and movable joints.
* **Adapter:** code translating simulator-specific data into domain inputs.
* **Lockstep:** complete one request/response exchange before advancing physics.
* **Journal:** the exact ordered input messages needed to replay a run.
* **Truth:** the simulated physical state, unavailable directly to autonomy.
* **Belief:** the estimate retained or propagated by the localization tracker.

## 0. Scope and baseline approval

1. Check Git state and read the current milestone. Preserve outstanding #17
   changes; do not treat its proposed ADR as accepted.
2. Ask the owner to authorize only stage 1 initially. Record its durable
   export/frame contracts in the next available ADR when approved. Do not
   preallocate ADR numbers for later stages.
3. Complete the native baseline in the setup guide. Keep the before trace
   outside tracked sources and record its scenario, map and resolved seed.
4. Choose a flat fixture contained within 50 m of the configured geographic
   origin. Agree on the robot asset, base frame and a supported workstation.

**Done when:** the scope and acceptance criteria are reviewed, native tests
pass, and there is a reproducible baseline. Lack of a GPU does not block
CPU-only packages, but it blocks claiming the stage's Isaac smoke test passed.

## Proposed file placement

Create files only as their package requires them. Stage 1 entries exist and
are tested; the rest are future paths, not links to implemented components:

```text
apps/fleet_isaac_export/       stage 1 native replay exporter + local CMakeLists
apps/fleet_isaac_worker/       stage 2 worker, codec and measurement adapters
tools/isaac/conversion.py      stage 1 pure WGS84<->scene conversions (implemented)
tools/isaac/replay.py          stage 1 read-only Isaac viewer (implemented, GPU-untested)
tools/isaac/sensor_harness.py  stage 2 Isaac coordinator
tools/isaac/tests/             pure-Python tests; optional GPU smoke test
tests/unit/isaac/              native adapter/protocol tests
tests/fixtures/isaac/          small intentional manifests and input journals
```

The opt-in `FLEET_BUILD_ISAAC_TOOLS` option exists (root `CMakeLists.txt`),
defaults OFF, and in neither configuration locates Isaac, CUDA, USD, Python
or ROS. The Python application runs separately through NVIDIA's launcher.

Follow [apps/CMakeLists.txt](../../apps/CMakeLists.txt) and
[tests/CMakeLists.txt](../../tests/CMakeLists.txt). Keep JSON parsing PRIVATE,
reusing the existing dependency from [src/CMakeLists.txt](../../src/CMakeLists.txt).
Share plain adapter functions between their executable and tests using a
small internal target if needed; do not put protocol types in domain headers.

## Stage 1: Read-only replay

### 1. Export a bounded reference run without a GPU

**Goal:** produce precise pose snapshots that Isaac can display without
influencing FleetSyncSim.

1. Build a tiny geometric map and bounded movement scenario using existing
   test patterns in [tests/support/test_maps.hpp](../../tests/support/test_maps.hpp)
   and [robot movement tests](../../tests/unit/robot/robot_movement_test.cpp).
   Include a straight segment, a turn and a stop inside the approved footprint.
2. In the new exporter, load the scenario with `ScenarioLoader::load` or
   `load_string`, construct `ScenarioRunner`, attach its normal trace sink,
   then call `begin()` exactly once.
3. For ordered sample ticks including 0 and the final horizon, call
   `run_until(tick)` and read the runner's const robot state. Use the existing
   [truth adapter](../../include/fleet/world/truth_pose.hpp) on the simulation
   side. Emit robots in ascending numeric RobotId order.
4. Export a versioned manifest and pose stream separate from the normal trace.
   Include geographic origin, map/scenario hashes, resolved seed, robot IDs,
   tick and named pose fields. Preserve round-trip double precision. Fail on
   missing geometry; never fill it with a zero coordinate.
5. Run the same scenario once without snapshot inspection. Compare its normal
   trace byte-for-byte with the exporter's trace. Also export twice and compare
   the pose streams. Sampling must not advance belief or consume RNG draws.

Do not parse rounded console text into poses. The existing
[TraceSink](../../include/fleet/scenario/trace.hpp) is observational, but its
formatted fields are not a precise sensor or pose transport. Do not modify
the founding trace schema just to supply this viewer.

**Tests:** stable robot ordering; tick 0/final snapshot; stationary heading;
missing geometry; two identical exports; inspection noninterference. Compare
snapshots with the existing truth adapter at the exact represented tick.

**Done when:** a CPU-only test can read the complete export, and the normal
trace matches the run without the exporter.

### 2. Display the export in Isaac

1. Add pure conversion functions for the bounded geographic frame and heading
   contract from the architecture guide. Keep them importable without Isaac.
2. Test the origin, east/north offsets, four cardinal headings and quaternion
   component ordering before loading a robot. Use independent expected values,
   not only a forward/inverse round trip that could hide matching errors.
3. Start `SimulationApp`, then import the selected runtime API. Create a floor
   and visible robot markers. Disable physical dynamics for replay objects.
4. Map each RobotId to one explicitly configured prim. Convert each snapshot
   and set its display pose. Never derive the next snapshot tick from FPS.
5. Display truth and belief with distinct labels/materials. An absent estimate
   has no belief marker, not a marker at the origin. Interpolation is optional
   presentation only and must never become native input.
6. Support a bounded headless smoke mode and guaranteed application cleanup.

**Done when:** the robot follows the recorded straight/turn/stop sequence at
the correct scale and orientation, reset restarts the playback, and rendering
enabled/disabled does not change the input file or native trace.

**Stage gate:** review stage 1. Stop here unless the owner separately promotes
the sensor harness and approves its time, protocol and external-input contract.

## Stage 2: Physical sensor harness

### 3. Build and test the native worker before connecting Isaac

**Goal:** feed a `LocalizationTracker` using measurements without changing
`Robot` or `ScenarioRunner`.

1. Freeze the `hello`, initial snapshot, frame, response, error and shutdown
   schemas. Use the architecture guide's units and string-encoded integers.
   Set initial sequence to `0`, then require sequence + 1 and tick + 10 ms.
   Choose explicit limits, initially 32 robots and 1 MiB per line.
2. In `hello`, require robot IDs, wheel dimensions/signs, frame/origin,
   physics/control/GNSS periods, ordered model-switch schedule, seed and input
   hashes. Reject unknown fields, duplicate keys, invalid ranges and an
   unsupported protocol. Echo the validated configuration in the acknowledgment.
3. For each response, echo protocol, sequence and tick. Report each robot's
   optional estimate, `estimated_at`, `last_fix_at`, estimate age and belief
   source. Keep truth and position error in a separate diagnostic object.
   Use JSON null for unavailable values; never manufacture an estimate.
4. Decode and validate complete messages before mutation. Use bounded reads,
   checked integer arithmetic and finite-number checks. For processing that
   can still throw, use candidate tracker/RNG states and publish them only
   after all robots succeed. No partial frame or success response on failure.
5. Give each robot its own `LocalizationTracker`, model and deterministic RNG.
   Reuse [GNSS models](../../include/fleet/localization/gnss_model.hpp) and the
   stream rules in [ADR-017](../design_decisions/ADR-017-deterministic-noisy-gnss.md)
   and [ADR-018](../design_decisions/ADR-018-gnss-outage-and-stale-localization.md).
6. Convert wheel increments into relative distance/turn and call `propagate`.
   At GNSS ticks only, convert truth to `GroundTruthPose`, call the active
   measurement model, then `apply_sample` with its optional result. Tick 0
   establishes baselines and takes the initial scheduled sample without motion.
7. Support file-driven requests for CPU replay and synchronous stdin/stdout
   for a future live peer. Keep stdout machine-readable and flush responses.

JSON libraries commonly accept repeated object keys by keeping the last value.
Configure duplicate-key detection explicitly on both sides. The schema must
also distinguish booleans from numbers, reject empty/sign-prefixed/overflowing
integer strings, and reject nonstandard NaN/Infinity tokens.

**Tests:** a hand-written journal with perfect fixes and forward motion; outage
before any fix; outage after a fix; zero motion; pure turn; scale bias; model
switch on a sample tick; identical replay; changing diagnostic output cannot
change belief or RNG state. Zero bias, 0.1 m wheel radii and two +1 rad wheel
increments should give 0.1 m forward motion before any GNSS replacement.

**Failure tests:** every protocol rejection from the architecture guide;
malformed JSON; oversized/truncated lines; wrong version; bad quaternion;
duplicate robot; reverse translation; unsupported curved interval; wrong
response sequence; overflow; mid-frame failure; fresh run after reset.

**Done when:** all worker tests pass on CPU, including a fake peer. No Isaac
installation is required and existing domain code remains unchanged.

### 4. Establish physical stepping and encoder readings

1. Load the pinned differential-drive asset on a flat floor. Resolve the
   articulation root and wheel DOFs by name, not assumed array indices. Verify
   meters-per-unit, wheel radii, separation, joint signs and base orientation.
2. Pin PhysX CPU dynamics for this small experiment. With the documented 6.1
   `PhysxScene`, configure `set_steps_per_second(100)` and read it back. Record
   solver and contact settings; CPU dynamics still requires a supported GPU
   for the Isaac application.
3. Initialize physics and verify tensor state validity before reading joint
   positions. Establish a settled, stationary initial state and encoder
   baseline before logical tick 0. Setup/warm-up steps are not experiment data.
4. Use `SimulationManager.step(steps=1)` for each physical interval. Check its
   step count relative to the recorded baseline; GUI updates must not add
   unaccounted steps. Derive Fleet ticks from the integer interval count.
5. Apply wheel velocity **targets**, then read measured joint positions after
   stepping. Use separate straight, stopped and in-place-turn segments. Pin
   allowed speed and turn tolerances; abort unsupported reverse/curved motion
   instead of silently coercing it into the current odometry primitive.
6. Read the base world pose only for the truth channel. Never derive encoder
   deltas from it. Record raw readings and the configuration needed to explain
   their conversion into increments.

Version-specific API references:

* [SimulationManager and PhysxScene](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html)
* [Articulation and XformPrim](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html)

The documented articulation surface includes `dof_names`, `get_dof_indices`,
`get_dof_positions`, `set_dof_velocity_targets` and `get_world_poses`. Velocity
control needs zero stiffness and suitable nonzero damping. Do not use
`set_dof_positions` or `set_world_poses` to move the physical robot during a
run: those teleport state. Experimental APIs can change without a deprecation
cycle, so isolate them in the Isaac adapter and verify the pinned build.

**Done when:** a bounded 1000-step run reports 10000 ms, repeats a stationary
baseline without carrying prior encoder state, and produces correct wheel
signs for forward and in-place-turn motion. Rendering does not affect the
logical step count. Declare numeric motion tolerances before evaluating runs.

### 5. Connect the coordinator and worker

1. Start the native worker with an absolute executable path and sanitized
   environment. Use argument arrays, not shell interpolation. Inherit stderr
   or drain it continuously so logs cannot fill a pipe and deadlock the child.
2. Complete the handshake and tick-0 exchange before stepping the scene.
   Permit exactly one outstanding request. Validate each response completely.
3. After each step, send all robots' data in a single frame and wait for its
   acknowledgment before another physics step. Journal the exact request bytes.
4. Use a wall-clock watchdog only to terminate a failed run. On error, inhibit
   further stepping, clear commands, close the child and mark the run failed.
   Do not continue simulation while waiting for a missing response.
5. On reset, start a fresh worker and reset physics, tick mapping, encoder
   baselines and control state together. On completion, acknowledge shutdown,
   check child exit status and close Isaac even if an exception occurred.

**Acceptance experiment:** run one robot forward under perfect GNSS, switch
to unavailable GNSS while motion continues, stop, then restore perfect GNSS.
Use zero bias first, then an explicit nonzero scale bias. During outage,
last-fix age grows; propagation advances estimate time; zero motion adds no
drift. The next scheduled perfect fix replaces the retained estimate using
the current tracker contract. This is not a new #18 fusion/reacquisition policy.

Replay the captured requests through the native worker with Isaac absent.
Require byte-identical canonical responses on the same native toolchain.
Then kill a fake peer between request and response and prove no further
physics step is permitted. Compare separately labeled position errors against
predeclared physical tolerances, not against an assumption of exact physics.

**Stage gate:** review stage 2 and its journal/metrics. Do not claim this proves
fleet routing or belief-driven navigation. Approve stage 3 separately.

## Stage 3: Physical movement backend

### 6. Approve and implement motion interfaces

This is a design task before it is a junior implementation task. The owner
must approve an ADR covering route intent, completion feedback, timeouts,
blocked motion, route changes in transit and exclusive odometry ownership.
Use [the architecture requirements](ARCHITECTURE.md#7-physical-movement-is-a-separate-gate).

Do not extend `ScenarioEvent` with simulator-specific objects or force physical
motion into graph arrival timers. Identify the smallest backend-independent
public Robot operations needed, keep controller/transport code adapter-side,
and preserve graph mode as the default. Then divide implementation into:

1. API and fake-backend unit tests proving exactly one motion/odometry source.
2. Adapter-side route-target controller with explicit speed/turn limits.
3. Physical feedback integration, including late arrival and blocked motion.
4. One-robot end-to-end fixture, then ordered multi-robot frames and isolation.

**Stop conditions:** any need for truth access in autonomy, reverse/arc support
beyond the approved odometry contract, map matching, collision avoidance,
ROS, IMU fusion or a new estimator. Bring these back for scope/design review.

## Review and validation

The commands below are existing native gates, run from the repository root.
Run each separately and require exit code zero before continuing:

```sh
cmake --preset debug -DFLEET_WERROR=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure
cmake --preset asan
cmake --build --preset asan
ctest --preset asan --output-on-failure
setarch "$(uname -m)" -R cmake --preset tsan
setarch "$(uname -m)" -R cmake --build --preset tsan
setarch "$(uname -m)" -R ctest --preset tsan --output-on-failure
git diff --check
```

After the proposed build option exists, test both its OFF native baseline and
its ON adapter targets with these presets. Run pure-Python protocol/conversion
tests without Isaac imports, using the standard library test runner unless a
reviewed need justifies another dependency. GPU tests are a separate required
stage acceptance check, not part of ordinary CPU-only CTest discovery.

Re-run the setup guide's reference command using a distinct `reference-after`
output filename and compare it with `reference-before` using `cmp`. Also
preserve the project's founding trace at the same seed. A new backend must not
change reference output just because an adapter target was enabled.

Each review should contain: scope/ADR status, exact commands and exit codes,
test counts, environment manifest, input journal hash, reference trace
comparison, physical metrics/tolerances, and any explicitly unrun GPU checks.
Do not commit installers, caches or bulk run outputs. Commit small intentional
fixtures only. No feature commit or push without the owner's instruction.
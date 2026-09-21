# Proposed Isaac Adapter Architecture

Status: design proposal for review. These are recommended decisions for a
future integration milestone, not accepted ADRs and not implemented APIs.
The [current milestone](../CURRENT_MILESTONE.md) and accepted ADRs still govern.
Use the next available ADR number only when a bounded stage is approved.

## 1. Responsibility split

```text
Isaac process                          Native FleetSyncSim process

USD stage + rigid-body physics         adapter executable / protocol decoder
wheel joint readings             --->  relative-motion measurement adapter
simulation truth                 --->  truth adapter -> GnssModel
fixed-step coordinator                 localization harness / later Robot API
robot actuator interface         <---  later motion commands

                   recorded, versioned process protocol

           diagnostics observe both sides; never correct belief
```

The arrows above are two different data paths even if they share a transport.
Isaac body pose is simulation truth. It must pass through a named measurement
model before becoming a localization estimate. Wheel odometry is relative
measurement data, not a truth-pose delta disguised as a sensor.

The first physical experiment is a localization harness owning a
`LocalizationTracker`, not a modified `ScenarioRunner`. It reuses the C++ GNSS
models and deterministic RNG rather than reimplementing them in Python.
This establishes an honest sensor boundary before changing autonomous motion.

## 2. Decisions to approve

| Decision | Recommendation | Reason / rejected shortcut |
| --- | --- | --- |
| Runtime coupling | Separate Isaac Python process and small native adapter executable | Avoid Python ABI coupling and simulator libraries in the core |
| Transport | One synchronous, versioned JSON Lines request/response stream over child-process pipes | No DDS, broker, socket service, threads or new transport dependency for the first experiment |
| Scheduling | Isaac-side coordinator owns physical steps; native worker consumes one ordered frame at a time | Never run two independent movement clocks |
| Ground truth | Separate labeled truth records, usable only by sensor models and diagnostics | Do not call `Robot::apply_gnss_sample` with a raw body transform |
| Odometry | Wheel-joint increments with explicit geometry/sign configuration | Do not use world-pose differences as wheel odometry |
| Motion control | Begin with scripted straight/stop/turn tests; add route following later | Graph edge cost is not a velocity controller |
| Determinism | Recorded-input native replay is exact within the existing toolchain scope; physical runs use tolerances | A seed does not make GPU physics byte-identical |
| Dependency policy | Reuse existing JSON parsing privately; no ROS, Isaac Lab, Autoproj or pybind11 initially | Each additional framework adds setup and version coupling |

The process coordinator may wait on I/O, but the FleetSyncSim domain remains
single-threaded. A wall-clock watchdog may abort a stuck experiment; it must
never invent a sample, skip a physics step or change a valid run's logical dt.

## 3. Coordinate and unit contract

Configure the initial test stage as right-handed, Z-up, **one stage unit = one
meter**. Define its world axes as X east, Y north, Z up. This geographic
alignment is our adapter configuration, not a fact Isaac infers from a USD.
Use a base frame with X forward, Y left, Z up, and explicitly transform assets
whose authored orientation differs.

See [NVIDIA's conventions](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/reference_conventions.html):
Isaac Core/USD quaternion interfaces use scalar-first `(w,x,y,z)`, while some
lower-level interfaces use `(x,y,z,w)`. Pin the API used; never guess ordering
from an unnamed array. Wire values should use named components. Normalize and
validate finite, nonzero quaternions at the adapter boundary.

For a planar base yaw `yaw` counterclockwise from world +X, convert to the
FleetSyncSim heading clockwise from north:

```text
heading_rad = normalize_heading(pi/2 - yaw)
```

| Facing | Isaac yaw | Fleet heading |
| --- | --- | --- |
| East | 0 | pi/2 |
| North | pi/2 | 0 |
| West | pi | 3*pi/2 |
| South | -pi/2 | pi |

Use a configured transform for sensor lever arms. The first fixture places
the modeled antenna at the base reference point; a real offset requires
rotation into world coordinates before the GNSS model. Current FleetSyncSim
poses are 2D. Keep altitude, roll and pitch as adapter/diagnostic values and
reject use outside the approved planar fixture instead of silently claiming
full 3D localization support.

### Small-fixture georeferencing

For the first experiment, use a purpose-built flat fixture within **50 meters
of its origin**, with anchor latitude magnitude at most 80 degrees. Fix the
origin, for example `(latitude=52.370, longitude=9.730)`, in the experiment
manifest. Never derive it from a noisy fix or reset it during an outage.

Reuse the existing spherical/local-tangent convention only in this bounded
engineering experiment:

```text
R = 6371008.8 meters
meters_per_degree = R * pi / 180
east  = wrapped_longitude_difference_deg * meters_per_degree * cos(anchor_lat_rad)
north = (latitude_deg - anchor_lat_deg) * meters_per_degree
```

The inverse uses the same fixed origin and scale; normalize longitude and
reject pole-domain/out-of-footprint inputs. Use named latitude/longitude
fields, not unlabelled coordinate pairs. The adapter conversion is a fixed
scene-frame mapping, distinct from localization's incremental
`apply_en_displacement`, which stays anchored at the current belief.

This is not a general WGS84 projection. The full built-in demo route extends
beyond this initial footprint. Do not squeeze it into the fixture by silently
changing scale. Wider real-map scenes require a reviewed projection decision,
such as WGS84 ECEF/local Cartesian conversion through a pinned adapter-private
geodesy library. Test poles, antimeridian and long baselines before advertising
those domains. The existing graph truth interpolation also needs antimeridian
edge-case review before those datasets are used.

## 4. Time and lockstep contract

The scenario vocabulary uses milliseconds. For the proposed bridge, explicitly
define **one Fleet Tick = one simulated millisecond**. `Tick` itself is a
strong integer type, not a wall-clock or a floating-point seconds value.

Start with physics dt = 0.010 s (10 ms), control period = 20 ms and GNSS period
= 1000 ms. All scheduled experiment changes must align to a 10 ms boundary.
Reject unaligned times; do not silently round them. Choose different rates
only by changing the manifest and validating their integer relationship.

Derive time from the step index: `tick = step_index * 10`, with checked integer
arithmetic. Never repeatedly add a floating-point dt and round it to ticks.
Never call `run_until(wall_clock_time)` or use rendering FPS for timekeeping.

The **new adapter loop**, not the existing reference queue, has this proposed
ordering at tick T:

1. Finish the physical interval `(T-10,T]` under the preceding command.
2. Read its encoder deltas and the truth snapshot at T.
3. In the native worker, propagate relative motion ending at T.
4. Apply scripted measurement-model changes at T in manifest order.
5. If T is a GNSS tick, measure truth through the active model and apply its
   fix/no-fix outcome. A switch does not create an extra sample.
6. Emit belief and separate truth/error diagnostics; acknowledge frame T.
7. Choose/hold the command for the next interval, then permit the next step.

At T=0 there is no preceding motion: initialize encoder baselines, apply T=0
model changes, and take the scheduled initial GNSS sample. Before the first
fix there is no estimate. Zero motion advances represented time once belief
exists; it does not create position drift or an absolute fix.

This proposed loop is not a claim of universal movement/script/sample
priorities in `EventQueue`. Existing reference behavior stays governed by
[ADR-005](../design_decisions/ADR-005-deterministic-event-scheduling.md) and
[ADR-018](../design_decisions/ADR-018-gnss-outage-and-stale-localization.md).
For the first harness, sensor capture and delivery occur in the same lockstep
frame. Delayed/out-of-order measurements require a separately reviewed policy;
the current tracker is not a delayed-measurement filter.

## 5. Measurement paths

### GNSS

Convert the Isaac truth snapshot into `GroundTruthPose` on the adapter side.
Its timestamp is the represented simulation tick. Pass it through an existing
`PerfectGnss`, `NoisyGnss` or `UnavailableGnss`, then apply the optional result.
Keep per-robot RNG streams and model draw-consumption rules from ADR-017/018.
No Python `random`/NumPy noise should replace the deterministic C++ model.

The heading returned by these models is a reference-simulation convenience,
not a claim that an ordinary GNSS receiver directly measures heading. Label it
as modeled GNSS in experiment metadata. During an outage, never substitute an
Isaac world position, joint-derived absolute position, or map-node snap.

### Wheel odometry

Read **measured joint positions**, not requested joint velocities. Configure
wheel names, sign corrections, radii and axle separation in meters. Establish
the initial joint baseline before the first interval. If joint values wrap,
unwrap with a declared per-step angular-speed bound; reject ambiguous jumps.

For a planar differential drive, with sign-corrected rotations in radians:

```text
left_distance  = left_radius  * left_rotation_delta
right_distance = right_radius * right_rotation_delta
forward_distance = (left_distance + right_distance) / 2
yaw_delta_ccw    = (right_distance - left_distance) / wheel_separation
turn_clockwise  = -yaw_delta_ccw
```

The initial test sequence separates straight forward motion and in-place
turning. These match the tracker's turn-then-translate primitive without
pretending a simultaneous arc is straight. Current `OdometryIncrement`
rejects negative distance: reject reverse translation in the first harness,
do not take an absolute value or silently flip its sign. Continuous curved
motion and reverse support need explicit integration/error contracts and tests
before the physical backend is generalized.

Apply configured scale/heading bias in exactly one layer. If the adapter has
already applied a measurement bias, propagate with zero additional bias.
Wheel slip may naturally create disagreement with physical truth; do not
correct it from body-pose differences. Raw IMU acceleration is not a displacement
increment and cannot simply be passed to `propagate`; IMU integration, gravity
compensation and fusion are later reviewed work.

## 6. Process protocol and failure behavior

Use JSON Lines as a proposed adapter protocol, separate from the existing
output-only trace format. Parse it with the standard Python JSON parser and
the repository's existing C++ JSON dependency, kept private to the executable.
The protocol example below is a design target, not an existing CLI endpoint.

One worker handles a run and all its declared robots. Start with a `hello`
request containing protocol version, resolved seed, map/config hashes, time
step, origin, sensor/robot configuration and ordered RobotIds. Its response
must confirm those fields before any physics step is accepted.

After a valid tick-0 snapshot, a single-robot frame might be:

```json
{"type":"frame","protocol":"fleet-isaac/1","sequence":"1","tick_ms":"10","robots":[{"robot_id":"1","truth":{"position_enu_m":{"east":0.01,"north":0.0,"up":0.0},"orientation_wxyz":{"w":1.0,"x":0.0,"y":0.0,"z":0.0}},"wheel_delta_rad":{"left":0.1,"right":0.1}}]}
```

For this example the handshake would declare 0.1 m wheel radii and a valid
axle separation. Truth and wheel readings remain separate channels even when
the test values happen to agree.

Validation rules to implement and test:

* Encode 64-bit ticks, seeds, sequence numbers and IDs as decimal strings;
  parse with range checks. Do not lose precision through JSON double storage.
* Require exactly one initial snapshot at tick 0; subsequent frames advance
  exactly one step and one sequence. Sort by numeric RobotId, not its string.
* Require every configured robot exactly once per frame. Unknown IDs, missing
  robots, duplicate entries, invalid quaternions and NaN/infinity are errors.
* Bound each message, initially to 1 MiB, and validate the complete frame
  before mutating any participant. Set a clear maximum robot count in config.
* Allow one outstanding request. A response echoes protocol, sequence and
  tick and includes estimates separately from diagnostics. Flush each line.
* A rejected frame, duplicate, gap, EOF, timeout or worker crash fails the run.
  Do not retry an uncertain frame, advance physics, synthesize no-fix data or
  continue with half the fleet updated. Restart from a recorded clean state.
* Put C++ logs on stderr; stdout contains protocol messages only. Isaac logs
  stay in the parent's logging channel, never in the worker's input pipe.
* A watchdog abort is an operational failure outside deterministic output.
  On normal completion send a shutdown message, wait for acknowledgment and
  worker exit, then close Isaac in a guaranteed cleanup path.

A scene reset starts a new worker/run with tick, RNGs, trackers, controller
state and encoder baselines reset together. Do not feed post-reset tick 0
into a worker that still holds estimates from the previous run.

## 7. Physical movement is a separate gate

[Robot::begin_transit](../../include/fleet/robot/robot.hpp) currently commits a
graph edge and computes its arrival from cost. `complete_transit()` uses that
committed timing. The runner schedules arrivals itself. None of these are
physical arrival acknowledgments.

Before stage 3, approve an interface separating route intent, actuator
commands and motion-completion feedback. Proposed requirements, not existing
method names:

* Exactly one backend owns movement: reference graph timing OR physical
  commands/feedback, never both. Preserve the reference implementation unchanged.
* External odometry has an explicit Robot API and an exclusive source mode;
  it cannot coexist with automatic graph-derived propagation for that robot.
* Route nodes/geometry become controller targets on the adapter side; the
  controller obeys explicit speed and turn limits rather than teleporting.
* Decide who produces completion feedback and what information it uses.
  Truth-based geometric arrival is acceptable only as a labeled simulation
  baseline behind an explicit measurement/feedback adapter. It is not proof
  of belief-driven autonomous arrival detection.
* Physical arrival time is actual feedback time, not the graph's predicted
  arrival. Define timeout, blocked-motion and route-change-during-motion policy.
* Applying localization remains independent of routing until localization-
  dependent autonomy is explicitly authorized. Do not silently turn a noisy
  estimate into a node ID or add map matching.

Do not work around the missing APIs with `const_cast`, mutable tracker getters,
fake `SetWorldEdgeStateAction` messages, or mutation of `RobotState` internals.

## 8. Reproducibility and build isolation

There are two distinct acceptance claims:

1. Native reference: same scenario and seed preserve existing byte-stable
   traces and preset gates, with no NVIDIA runtime installed.
2. Physical experiment: record simulator/driver/assets/settings and the input
   journal. Replaying that exact ordered journal through the native harness
   reproduces its deterministic results on the same toolchain. Re-running
   physics itself is compared with declared tolerances, not assumed identical.

Record external inputs and their acquisition ticks as explicit experiment
inputs. This extends the input contract only for the proposed external
backend; it does not silently weaken the reference simulator's scenario/seed
contract. Reference and physical traces must carry distinct backend metadata.

No `isaacsim`, `omni`, `pxr`, ROS or simulator-specific types enter public
FleetSyncSim domain headers. Any new adapter target is opt-in and defaults OFF.
Use headless fake-peer/recorded-journal tests on CPU CI; mark actual GPU tests
separately. A missing GPU means a clearly reported skip, never a passing
physical integration test.
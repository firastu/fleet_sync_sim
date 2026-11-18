# ADR-024: Terrain-domain relative-motion and altitude sensors

* Status: Proposed (implemented; awaiting #21 review)
* Date: 2026-11-18
* Scope: M4 #21; `localization::TerrainSensors` and its measurement types,
  the shared noise sampler, the true motion increment reported by
  `world::TerrainMotion`, `robot::TerrainRobot::receive`, the scenario
  `sensors` block and the `sensor_frame` trace event. No estimator.

## Context

Terrain-referenced navigation (M4) estimates position from what a body
senses: how far it moved, how it turned, how high it is, and (for air
platforms) how far the ground is below it. #20 moves bodies by scripted
truth; #22 will choose an estimator. #21 supplies the measurements in
between, with explicit, reproducible error, so the estimator can be judged
against known failure modes. Truth must cross into robots only as
measurements (ADR-011/016), and the measurement noise must follow the
project's determinism rules: `DeterministicRng`, a fixed draw consumption and
independent streams.

## Decision

### The measurement boundary

```text
world::TerrainMotion step  ->  localization::TerrainMotionTruth  (truth)
                                        |
                           localization::TerrainSensors (per robot)
                                        |
                           localization::TerrainSensorFrame  (measurements)
                                        |
                           robot::TerrainRobot::receive
```

* `TerrainStepOutcome` now reports the true increment of each step:
  `distance_m` (0 when parked) and the applied `turn_rad`.
* The runner builds one `TerrainMotionTruth` per step: the interval,
  distance, turn, the body's altitude, and its height above the truth
  terrain surface. `TerrainMotionTruth` is truth, like `GroundTruthPose`;
  only sensor models read it. `GroundTruthPose` stays unchanged.
* Sensors live in `fleet::localization` and need no `fleet::world`: they
  receive scalar truth, not the terrain grid.
* The robot receives the `TerrainSensorFrame` only. In #21 it keeps the
  latest frame and a frame count and rejects frames that go back in time;
  the #22 estimator consumes the frames.
* Sensors sample once per motion step, right after it. A robot whose mission
  completes stops stepping, and therefore stops producing frames, exactly as
  its `terrain_pose` events stop (ADR-023).

### Sensor models

Every model is an engineering degradation model, not a device simulation.
`N` is one standard noise sample; `dt` is the interval in seconds.

* **Odometry** (stand-in for wheel odometry, visual odometry or optical
  flow): `d * (1 + scale_error) + noise_m_per_sqrt_m * sqrt(d) * N`. The
  noise grows with the square root of the distance travelled, so its variance
  adds up per meter and does not depend on the step length. A parked body
  measures 0.
* **Gyro**: the bias walks first, `bias += bias_walk * sqrt(dt) * N`; then
  `turn + bias * dt + angle_random_walk * sqrt(dt) * N`.
* **Barometric altimeter**: the bias walks first (weather and temperature
  drift), `bias += bias_walk * sqrt(dt) * N`; then
  `altitude + bias + noise_sigma * N`.
* **Downward rangefinder** (air platforms only): the true range is the height
  above the truth terrain surface. Above `max_range_m` the reading is
  `OutOfRange`; otherwise a dropout happens with probability
  `dropout_ppm / 10^6`; otherwise `range + noise_sigma * N` is returned.
  `OutOfRange` is reported before `Dropout`.

Measured distances and ranges are not clamped: noise may make them slightly
negative. Clamping would bias the estimator silently. A sensor with zero noise
measures exactly: odometry still applies its scale error, and the gyro and
barometer their initial bias.

### Determinism

* **One shared sampler.** The Irwin-Hall sample of ADR-017 moves into
  `localization::irwin_hall_standard_noise` (12 midpoint-centered uniform
  draws, no transcendental functions). `NoisyGnss` uses it with unchanged
  arithmetic, and its output is bit-identical to before.
* **One stream per robot and sensor kind.** Seeds are
  `derive_stream_seed(resolved_seed, domain, RobotId)` with the fixed domains
  ASCII `SNS_ODOM`, `SNS_GYRO`, `SNS_BARO` and `SNS_RNGF`. Adding,
  removing or reconfiguring one sensor never shifts another sensor's noise,
  and no robot's noise depends on another robot.
* **Fixed consumption that depends only on the configuration**, never on the
  truth values. Per frame:
  * odometry: 12 draws if its noise is non-zero;
  * gyro: 12 for the bias walk, then 12 for the angle random walk, each only
    when non-zero;
  * barometer: the same pattern for bias walk and noise;
  * rangefinder: 1 `uniform_below(1000000)` draw when 0 < `dropout_ppm`
    < 10^6, then 12 for noise when non-zero. These are drawn even when the
    reading is out of range or dropped.
* Invalid truth (time going backwards, non-finite values, negative distance
  or negative height with a rangefinder) throws before any draw or state
  change.

### Scenario format and trace

* Each robot may declare a `sensors` object with any of `odometry`
  (`scale_error`, `noise_m_per_sqrt_m`), `gyro` (`initial_bias_deg_per_s`,
  `bias_walk_deg_per_s_per_sqrt_s`, `angle_random_walk_deg_per_sqrt_s`), `baro`
  (`initial_bias_m`, `bias_walk_m_per_sqrt_s`, `noise_sigma_m`) and
  `rangefinder` (`max_range_m` required, `noise_sigma_m`, `dropout_ppm`). Fields
  default to 0 and unknown fields are rejected. Files use degrees; the code
  uses radians. A rangefinder on a ground platform is rejected by the loader
  and by the runner.
* Without a `sensors` object a robot has no sensors, draws no randomness and
  emits no frames: `scenarios/terrain_hills.json` traces are byte-identical.
* `sensor_frame` (source: the robot) carries `from` and, per configured
  sensor, `odometry_m`, `gyro_turn_rad`, `baro_m`, and `range` (`return`,
  `out_of_range` or `dropout`) with `range_m` on a return. Values are
  6-decimal strings. Frames carry no truth; truth stays in the `world` events.
* `scenarios/terrain_sensors.json` is the terrain-hills run with sensors on
  both robots.

## Alternatives rejected

* **Fusing measurements in the robot now** (dead reckoning from odometry and
  gyro). That is estimation and belongs to #22's ADR.
* **IMU accelerations and angular rates as raw measurements.** Integrating
  them needs an inertial navigation model that M4 does not need yet; the
  heading-change gyro and distance odometry are the increments a
  terrain-matching estimator consumes.
* **Per-sensor sampling periods.** More realistic, but adds scheduling
  contracts before the estimator needs them. One frame per step is enough to
  start.
* **Noise per sample independent of `dt`.** Results would change whenever the
  scenario's step length changed; `sqrt(dt)` and `sqrt(distance)` scaling
  keep error statistics physical.
* **Truth-dependent draw consumption** (skipping noise when out of range or
  parked). It would couple the noise sequence to the trajectory, so a small
  truth change would reshuffle every later measurement.
* **Clamping negative measured ranges or distances.** Silent bias.

## Consequences and limitations

* Pre-existing traces (road-graph and `terrain_hills`) are byte-identical, and
  noisy GNSS output is bit-identical (checked against the previous build).
* Robots now hold measurements but do nothing with them until #22.
* Frames stop when a mission completes; a parked robot's sensors are silent.
* The rangefinder measures to whatever the truth terrain represents: a
  surface model gives canopy and roofs, a bare-earth model gives the ground.
* Odometry has no slip, the gyro no scale-factor error, and the barometer no
  pressure-altitude model. These are documented approximations.

Validation: `tests/unit/localization/terrain_sensors_test.cpp` reproduces every
sensor's output exactly from reference streams, which locks the formulas and
the draw consumption. It also covers zero-noise exactness, stream
independence, replay, dropout extremes, and validation that consumes nothing.
`tests/unit/robot/terrain_robot_test.cpp` covers frame ordering;
`tests/unit/world/terrain_motion_test.cpp` covers the reported increment; and
`tests/unit/scenario/terrain_scenario_test.cpp` covers sensor parsing and
rejection, frames reaching the robot and the trace, seed sensitivity of
measurements but not truth, and sensor-free scenarios staying sensor-free.

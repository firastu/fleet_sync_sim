# ADR-025: Terrain-referenced grid estimator

* Status: Proposed (implemented; awaiting #22 review)
* Date: 2026-11-18
* Scope: M4 #22, the estimator decision; `localization::TerrainGridEstimator`,
  `TerrainEstimate` (the first uncertainty representation),
  `robot::TerrainRobot`'s estimator, the scenario `estimator` block, and the
  `terrain_estimate` and terrain-domain `localization_error` trace events.

## Context

M3 deferred an estimator until failure modes justified one (ADR-016 to
ADR-021: replace-on-fix, no fusion). M4 now supplies them. Terrain-referenced
navigation compares measured terrain height with a map. Its likelihood is
nonlinear in position, and flat or repeating terrain gives several equally
good positions at once. A single-hypothesis tracker cannot represent that.
The sensors of ADR-024 provide odometry, gyro, barometer and (for air
platforms) rangefinder frames. The barometer bias is unknown and drifts, so
it must be estimated rather than assumed.

## Decision

### Approach: a point-mass (grid) filter

Three candidates were compared:

| | Batch profile correlation (TERCOM) | Point-mass (grid) filter | Particle filter |
|---|---|---|---|
| Idea | correlate a recorded height profile against the map along candidate offsets, then jump | a probability on every cell of a grid, updated each measurement | thousands of weighted random position samples |
| Ambiguity | one best offset per batch; ambiguity shows only as a flat correlation peak | represented exactly: several modes are several high cells | represented, but sampled |
| Randomness | none | **none** | resampling and noise injection need random draws, so a fixed consumption contract and a stream are needed |
| Continuous estimate | no: fixes come in batches, and straight legs are assumed | yes, every frame | yes, every frame |
| Inspectability | correlation surface per batch | the full probability map | a sample cloud |
| Cost | low | grows with grid area (fine at kilometre scale) | grows with sample count; scales better to more state dimensions |
| Weakness | needs straight, constant-speed legs and a separate tracker between fixes | dimension curse: position only, not heading | sample impoverishment, tuning, nondeterministic-looking behaviour |

**The point-mass filter is chosen.** It represents the ambiguity that
motivated the estimator exactly, needs no random numbers (so beliefs are
reproducible bit for bit from the frames), updates every frame, and its state
can be drawn as a probability map. Its cost is bounded by the grid (at most
401 x 401 cells). A particle filter remains the candidate once heading and
other states must be estimated jointly; it would need its own ADR.

### State and update

The grid is a square of `(2 * half_width / cell_size + 1)^2` cells in a local
east/north frame around a moving centre. Cell positions come from
`apply_en_displacement` offsets, so the half-width is limited to 2 km to keep
that local approximation accurate. Each cell holds a probability and a scalar
Kalman estimate (mean, variance) of the barometer bias given that the robot
is in that cell. Once position is fixed the bias is linear-Gaussian, so it is
marginalized per cell (Rao-Blackwellized) instead of adding a grid dimension.

Per frame, in order:

1. heading += the measured gyro turn;
2. the grid centre moves by the measured odometry distance along the new
   heading. Probability moves with the cells, so dead reckoning adds no
   numerical smearing;
3. position noise accumulates as variance (`noise^2 * |distance|`). When it
   reaches `(cell_size / 2)^2` the grid is convolved with that Gaussian
   (separable, truncated at 3 sigma), and the accumulator resets. Diffusion
   smaller than a cell is therefore delayed, never dropped. The bias moments
   are carried exactly by convolving probability, probability x mean and
   probability x (variance + mean^2);
4. every cell's bias variance grows by `walk^2 * dt`;
5. with a height measurement `z`, each live cell with map height `h` is
   weighted by `N(z - h - bias_mean; 0, bias_var + height_sigma^2)` and its
   bias is Kalman-updated. Ground: `z = baro - mount height`. Air:
   `z = baro - range`, only on a rangefinder `Return`; dropouts and
   out-of-range readings skip the update. **A cell where the robot's own map
   has no height gets probability 0**: the robot's map defines where it can
   be;
6. normalize. If no probability is left (a measurement no cell explains),
   the estimator is **lost**: the grid resets to the uniform prior around its
   current centre, and `resets` counts it;
7. when the probability-weighted mean drifts more than a quarter of the
   half-width from the centre, the grid **re-centres** by whole cells toward
   it (probability leaving the grid is dropped; entering cells are empty), so
   the belief can follow the truth far beyond the initial grid;
8. the estimate is computed.

### Prior

A terrain-only start needs an explicit prior region. The scenario's required
`prior` gives the square's centre and the initial heading; the robot is
equally likely anywhere in the square. The initial heading is assumed known,
as from a compass at power-on. The barometer bias prior is zero mean with
`baro_bias_sigma_m`.

### Output: the first uncertainty representation

`TerrainEstimate` carries:

* the `LocalizationEstimate`: probability-weighted mean position, integrated
  heading, and `estimated_at` = the frame's end;
* `most_likely`: the highest-probability cell (ties go to the lowest index);
* `spread_m`: the radial RMS distance of the probability about the mean;
* `effective_cells = 1 / sum(p^2)`: 1 when certain, about N when spread over
  N equal cells, so ambiguity is visible;
* `baro_bias_m`: the probability-weighted mean bias estimate;
* `measured` and `resets`.

When the belief is multimodal, the mean can sit between modes, away from both.
`spread_m` and `effective_cells` reveal that, and `most_likely` names the
strongest mode.

### GNSS (decided here, implemented with the GNSS scenarios of #24)

In terrain scenarios a GNSS fix will enter as a **likelihood factor**: each
cell is weighted by `N(fix - cell; 0, sigma_gnss^2)`. It will not *replace*
the belief, which is ADR-018's rule for the road-graph domain and stays
there. This lets a fix sharpen the belief, lets an inconsistent (spoofed) fix
be outweighed or detected, and makes jamming simply the absence of the
factor. The terrain domain has no GNSS sampling yet, so nothing changes in
#22.

### Ownership, configuration and trace

* The estimator belongs to the robot: `TerrainRobot` owns an optional
  `TerrainGridEstimator`, built on the robot's own terrain map, and updates
  it in `receive()`. If the estimator rejects a frame, nothing changes. Truth
  never enters: the inputs are frames and the robot's map (ADR-011).
* The scenario's per-robot `estimator` block needs `prior` (`lat`, `lon`,
  `heading_deg`). Optional tuning: `half_width_m`, `cell_size_m`,
  `position_noise_m_per_sqrt_m`, `height_sigma_m`, `baro_bias_sigma_m`,
  `baro_bias_walk_m_per_sqrt_s`. The height reference and mount height follow
  the robot's platform. The loader and the runner reject an estimator without
  odometry, gyro and barometer, or an air estimator without a rangefinder.
  These are the estimator's own assumptions and need not match the simulated
  sensors.
* After each frame, `terrain_estimate` (source: the robot) carries belief
  only: position, heading, most likely cell, spread, effective cells, bias,
  measured, resets. A `localization_error` event (source `"world"`, as in
  ADR-019) gives the simulation-side distance from truth. It is outward-only
  and never fed back.
* `scenarios/terrain_navigation.json` runs both robots with estimators whose
  priors start 75-155 m off. The drone converges to about 5 m within 81 s over
  the hills; the rover converges more slowly over flatter ground (about 6 m by
  300 s).

## Alternatives rejected

* **TERCOM-style batch correlation**, and **a particle filter** for now. See
  the comparison above.
* **An EKF/UKF.** One Gaussian cannot hold several positions; it diverges on
  ambiguous terrain.
* **Profile differencing to cancel the bias.** It eliminates the bias rather
  than estimating it, and discards absolute height information. Per-cell
  Kalman bias keeps both.
* **One global bias shared by all cells.** In multimodal beliefs it blends
  the biases implied by different positions.
* **A fixed grid with sub-cell shifts by interpolation.** Every step smears
  probability numerically, as uncontrolled extra noise. The moving grid
  translates exactly.
* **Keeping probability on cells without map data.** It would favour
  off-map cells, which are never penalized.
* **Estimating heading on a third grid axis.** Cost multiplies; that is the
  particle filter's territory.

## Consequences and limitations

* Heading is not estimated. It is integrated from the gyro, so gyro bias
  becomes direction error that `position_noise` must cover.
* **Systematic errors make the belief overconfident.** Odometry scale error
  and heading drift are not in the random-walk noise model. The belief then
  lags the truth on flat stretches, and the error can exceed `spread_m`
  until informative terrain corrects it. A 10% odometry scale error stays
  within 20-70 m over 3 km while dead reckoning drifts 300 m.
* Flat terrain cannot be matched. The belief only dead-reckons there, and
  error grows (the drone's final flat leg in the fixture reaches about 45 m).
  This is the expected failure the #24 suite reports.
* The estimate does not steer anything yet (#23).
* The trace grows by two events per frame per robot with an estimator.
* Every pre-existing trace (road-graph, `terrain_hills`, `terrain_sensors`)
  is byte-identical; estimators are opt-in.

Validation: `tests/unit/localization/terrain_estimator_test.cpp` covers:

* configuration and frame validation that changes nothing;
* pure dead reckoning on flat terrain (the exact uniform spread);
* convergence from a 150 m prior offset with an unknown 7 m bias estimated;
* air updates that skip missing returns;
* off-map cells losing probability, and reset when every hypothesis is lost;
* re-centring under a 10% odometry error;
* bit-identical replay.

`tests/unit/robot/terrain_robot_test.cpp` covers the robot-owned estimator,
and `tests/unit/scenario/terrain_scenario_test.cpp` covers estimator parsing
and rejection, convergence of both fixture robots, belief and diagnostic
events, byte-identical traces, and runner-side rejection.

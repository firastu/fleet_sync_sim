# ADR-016: The localization boundary — truth, estimate, and sensor models

- Status: Accepted
- Date: 2026-08-30
- Scope: `fleet::localization` (pose types, GnssModel); M3 opening

## Context

M2 delivered movement over real road geometry, a strict world-truth vs
robot-belief boundary (ADR-011), and operator inspection. The natural
next layer — PNT — risks being designed around an algorithm (a Kalman
filter, SLAM) instead of around the failure modes that matter:
degraded GNSS, outages, drift, stale estimates. The M3 decision is to
establish the boundary FIRST and make each failure mode observable
BEFORE choosing any estimator.

## Decision

    GROUND TRUTH POSE  (simulation-side; like World truth, ADR-011)
            |
            X  robots never read it directly
            |
      SENSOR MODEL    (GnssModel: pure (truth, rng) -> optional estimate)
            |
            v
    LOCALIZATION ESTIMATE  (robot-side belief)

- **Two distinct types, never conflated**: `GroundTruthPose` (where the
  robot actually is; derived by the simulation from RobotState +
  MapGeometry) and `LocalizationEstimate` (where the robot believes it
  is). Heading is radians clockwise from north in the WGS84 frame,
  normalized to [0, 2*pi) via `normalize_heading`, which REJECTS
  non-finite headings (`std::invalid_argument`) — NaN/inf never quietly
  become localization state.
- **Heading source caveat**: real GNSS receivers do not always provide
  heading directly (and when they do, e.g. dual-antenna, it is a
  separate quality from position). This baseline's models emit heading
  for simulation convenience; later models may source position and
  heading from different sensing channels — the interface carries both,
  it does not promise both from one physical measurement.
- **Sensor models are pure functions** of `(truth, DeterministicRng)`:
  randomness only from the passed RNG (same seed + truth sequence =>
  same measurement sequence); **models that need no randomness MUST
  consume none** (locked by twin-RNG tests: switching between perfect,
  noisy and unavailable models over a shared RNG never shifts future
  noise sequences); `nullopt` means "no fix" — the caller keeps or ages
  its previous estimate. An outage is modeled by WHICH model is active
  (scenario policy), never by special-casing call sites.
- **`estimated_at` is part of the estimate**: staleness is a
  first-class, observable state — the raw material for outage/drift
  scenarios.
- **No uncertainty representation yet.** Position + heading + timestamp
  only. Noise covariances, particles or quality flags are added only
  when a demonstrated failure mode justifies them (per this ADR's
  spirit; a follow-up ADR records the choice).
- M3 delivery ladder (each step observable in traces/scenarios before
  the next): perfect GNSS (this commit) -> noisy GNSS -> outage / stale
  estimate -> dead-reckoning drift -> reacquisition -> then decide:
  complementary filter / EKF / particle filter / map matching on
  MapGeometry / cooperative localization via the fleet channel.

## Alternatives considered

- **Start with an EKF.** Rejected: an estimator without observable
  failure modes has nothing to filter; the boundary would be shaped by
  the algorithm instead of the problem.
- **Fusion of pose into the observation boundary (ADR-011).** Rejected:
  edge-status sensing and pose estimation are different channels with
  different failure modes; they compose later (map matching), not
  merge.
- **Estimates inside Robot now.** Deferred: robots gain an estimate
  store when a consumer exists (replan-from-estimated-position,
  map matching); the boundary types precede the wiring by design.

## Consequences and limitations

- `fleet_localization` depends only on map coordinates, logical time
  and the deterministic RNG — no robot, world, network or scenario
  coupling; wiring arrives with the first consumer.
- Ground-truth derivation (RobotState -> pose) is not implemented in
  this commit; it lands with the first scenario that needs it.
- Heading on a spherical-earth approximation is sufficient for M3
  scenarios; projection refinements arrive with geospatial needs.

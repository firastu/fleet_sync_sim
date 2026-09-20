# ADR-017: Deterministic noisy GNSS — metric-frame noise, specified arithmetic

- Status: Accepted
- Date: 2026-08-30
- Scope: `fleet::localization` (GnssNoiseConfig, NoisyGnss,
  apply_en_displacement); M3 ladder step 2 of 5

## Context

ADR-016 established the truth-vs-estimate boundary with a perfect model
and an outage model. The next failure mode is DEGRADATION: GNSS fixes
that are wrong by meters, deterministically. Two hazards shape this
decision: (a) `std::normal_distribution` has unspecified algorithms
across standard-library implementations (the ADR-006 problem), and
Box-Muller as a replacement routes through `log/sqrt/sin/cos` — libm
transcendentals that are not guaranteed bit-identical across platforms,
trading one portability problem for a subtler one; (b) adding noise
directly to latitude/longitude DEGREES makes "5 m of noise" mean
different physical distances at different latitudes.

## Decision

### Noise distribution: Irwin-Hall / CLT approximation

- Each noise component (east m, north m, heading rad) is the sum of 12
  independent midpoint-centered finite-resolution uniforms,
  `(k + 0.5)/2^32 - 0.5` for the 32-bit draw k, scaled by sigma. The
  support is exactly symmetric about zero (complement draws k and
  2^32-1-k map to exactly opposite values, so the discrete mean is
  exactly 0). The 12-sample sum has variance `1 - 2^-64` — effectively
  unity at double precision. The earlier draft's "variance EXACTLY 1"
  claim was WRONG (it held only for a continuous uniform; the naive
  `k/2^32 - 0.5` has support [-0.5, +0.5), a tiny negative mean and
  variance slightly below 1/12) — the midpoint form is the honest fix.
- Every arithmetic step of the SAMPLER is fully specified: 32-bit
  integer draw -> add 0.5 -> exact division by 2^32 (power of two) ->
  subtract 0.5 -> sum -> multiply by sigma. No libm in the sampling
  path at all.
- REPRODUCIBILITY SCOPE, stated honestly in two parts: (a) the random
  SAMPLING sequence is fully specified by the above arithmetic — no
  std distributions, no transcendentals — so identical seed + inputs
  reproduce identical sample sequences on any conforming platform;
  (b) the GEODETIC TRANSFORM (apply_en_displacement) is ordinary
  floating-point geographic arithmetic including one cos() — the same
  libm class as the importer's haversine (ADR-014) — so final
  coordinate BIT-identity is guaranteed on the same toolchain and
  platform, but NOT claimed across different libm implementations.
- This is a DETERMINISTIC ENGINEERING APPROXIMATION of receiver error,
  NOT a claim of high-fidelity GNSS statistics. We are testing autonomy
  behavior under degraded positioning, not certifying a receiver noise
  model. Multipath, correlated bias, urban canyon, satellite geometry:
  out of scope; replaceable later behind the same GnssModel boundary.

### Noise geometry: local tangent frame in meters at the truth

- `apply_en_displacement(position, east_m, north_m)`: the single
  metric<->WGS84 seam of the localization module. Anchored AT the truth
  position per fix — no scenario-global anchor (independent fixes
  anchor independently; simpler and sufficient).
- Failure domain is EXPLICIT, never silently clamped (the ADR-014
  importer philosophy — never manufacture plausible-but-wrong
  positions):
  - longitude WRAPS across the antimeridian into [-180, +180):
    continuing east past +180 emerges at -180 (mirrored westward) —
    the requested displacement is preserved, test-locked;
  - east displacements at |latitude| > 89 deg are outside the
    approximation's supported domain (the east conversion divides by
    cos(latitude), which degenerates toward the poles) and throw
    std::invalid_argument;
  - north displacements that would cross a pole throw
    std::invalid_argument; non-finite displacements throw
    std::invalid_argument.
- NEVER `latitude_deg += noise`: the same east-meter offset maps to
  different longitude-degree deltas by cos(latitude) — test-locked at
  the equator vs 80 degrees.
- Valid scale: meters to tens of meters (tangent-plane error
  negligible). The conversion itself uses one cos() — see the
  reproducibility scope above; the SAMPLING path, which consumes the
  RNG, stays transcendental-free.

### Configuration: physically named, separately knobbed

- `GnssNoiseConfig{position_axis_sigma_m, heading_sigma_rad}`:
  `position_axis_sigma_m` is the PER-AXIS noise scale — east and north
  errors are drawn independently at this scale, so the radial 2D RMS
  error is approximately `position_axis_sigma_m * sqrt(2)`. Position
  quality and heading quality degrade independently in reality and in
  this model. Finite and non-negative or the constructor throws.

### RNG-consumption contract (fixed, documented, test-locked)

- Per fix: 12 draws east + 12 north (only when position_axis_sigma_m >
  0), then 12 for heading (only when heading_sigma_rad > 0), in that
  order.
- Zero total noise consumes ZERO randomness and reproduces PerfectGnss
  bit-exactly — model switches (perfect -> noisy -> outage) never shift
  future noise sequences of a shared RNG.
- `estimated_at == truth.at` preserved: staleness semantics live in the
  estimate timestamp, never in the measurement path.

## Alternatives considered

- **Box-Muller / std::normal_distribution.** Rejected: the former
  depends on libm transcendental bit-stability, the latter on
  unspecified distribution algorithms (ADR-006 class of problem).
- **Gaussian noise in degrees.** Rejected: latitude-dependent physical
  error; "5 m" would not mean 5 m.
- **Scenario-global tangent-plane anchor.** Rejected for now: per-fix
  anchoring at the truth is simpler, independent and adequate; a global
  projected frame arrives if/when a consumer needs consistency across
  large displacements (with its own ADR).
- **Realistic receiver modeling.** Deferred deliberately: observe
  failure modes first (ADR-016 ladder).

## Consequences and limitations

- Irwin-Hall(12) tails are lighter than Gaussian — acceptable for
  autonomy-under-degradation experiments; documented, not hidden.
- The sampler's 2^-64 variance deficit and its discrete support are
  invisible at any sigma used in practice (5 m scale vs 1e-9 m
  resolution); documented for honesty, not measurability.
- Heading noise wraps through normalize_heading (finite-checked).
- The local-tangent approximation is unsupported (explicit throw) for
  east displacements near the poles; legitimate for this project's
  mid-latitude OSM extracts.
- No correlation between successive fixes (white noise); temporal
  correlation/drift models arrive at ladder steps 3-4 behind the same
  boundary.

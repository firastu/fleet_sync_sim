#pragma once

#include <optional>

#include "fleet/localization/pose.hpp"
#include "fleet/simulation/deterministic_rng.hpp"

namespace fleet::localization {

// The ONLY path from GroundTruthPose to LocalizationEstimate (ADR-016),
// mirroring the observation boundary of ADR-011: truth in, belief out,
// never the reverse. Models are pure functions of (truth, rng):
//
//   - randomness is drawn ONLY from the passed DeterministicRng (never
//     wall clock, never hidden state) — same seed + same truth sequence
//     => same measurement sequence;
//   - a model that needs no randomness consumes none (PerfectGnss);
//   - nullopt means "no fix this time" — the caller keeps (or ages) its
//     previous estimate; an outage is modeled by models and scenarios,
//     not by special-casing.
//
// Later models (noisy GNSS, dead reckoning, map matching, cooperative)
// implement this interface without moving the boundary.
class GnssModel {
public:
    virtual ~GnssModel() = default;

    [[nodiscard]] virtual std::optional<LocalizationEstimate> measure(
        const GroundTruthPose& truth, simulation::DeterministicRng& rng) const = 0;
};

// Perfect GNSS: the estimate equals the truth exactly (position copied
// by value, heading preserved). Consumes NO randomness — a zero-noise
// baseline that makes the whole downstream pipeline observable before
// any degradation is introduced.
class PerfectGnss final : public GnssModel {
public:
    [[nodiscard]] std::optional<LocalizationEstimate> measure(
        const GroundTruthPose& truth, simulation::DeterministicRng& rng) const override;
};

// Noise configuration with PHYSICAL names (ADR-017) — separate knobs,
// because a real receiver's position quality and heading quality
// degrade independently.
//
// `position_axis_sigma_m` is the PER-AXIS noise scale: east and north
// errors are each drawn independently at this scale, so the radial 2D
// RMS error is approximately position_axis_sigma_m * sqrt(2) — NOT
// position_axis_sigma_m itself. Heading error is independent, in
// radians.
//
// Validation: both values must be finite and non-negative; anything
// else throws std::invalid_argument at construction.
struct GnssNoiseConfig {
    double position_axis_sigma_m = 0.0;
    double heading_sigma_rad = 0.0;
};

// Noisy GNSS (ADR-017) — a DETERMINISTIC ENGINEERING APPROXIMATION of
// receiver error, not a claim of high-fidelity GNSS statistics:
//
//   truth position
//         |
//   local tangent frame (East/North METERS, anchored at the truth)
//         |
//   deterministic noise E [m], N [m]  (and heading noise [rad])
//         |
//   noisy metric displacement -> WGS84 (apply_en_displacement)
//         |
//   LocalizationEstimate
//
// RANDOM SAMPLING — fully specified, independent of std distributions
// and of libm transcendentals: each noise component is the sum of 12
// midpoint-centered finite-resolution uniforms, (k + 0.5)/2^32 - 0.5,
// a discrete distribution that is exactly symmetric about zero
// (complement draws sum to exactly 0.0). Twelve summed samples have
// variance 1 - 2^-64 — effectively unity at double precision, and
// honestly NOT the continuous-uniform "exactly 1". No Box-Muller, no
// std::normal_distribution: both would trade the std-distribution
// portability problem for a subtler libm portability problem.
//
// REPRODUCIBILITY SCOPE: the RNG/sample sequence is fully specified
// (mt19937_64 raw output + the exact arithmetic above), so identical
// seed + inputs reproduce identical estimates on the SAME
// toolchain/platform. The geodetic transform (apply_en_displacement)
// uses floating-point geographic arithmetic including cos(), the same
// libm class as the importer's haversine — cross-libm BIT identity of
// final coordinates is NOT an architectural guarantee today.
//
// RNG-consumption contract (fixed, documented, test-locked): per fix,
// exactly 12 draws for east + 12 for north (skipped entirely when
// position_axis_sigma_m == 0), then 12 for heading (skipped when
// heading_sigma_rad == 0), in that order. Zero total noise therefore
// consumes zero randomness and reproduces PerfectGnss exactly; model
// switches never shift future noise sequences.
//
// estimated_at == truth.at is preserved: staleness semantics live in
// the estimate timestamp, not in the measurement path.
class NoisyGnss final : public GnssModel {
public:
    explicit NoisyGnss(GnssNoiseConfig config);

    [[nodiscard]] const GnssNoiseConfig& config() const noexcept { return config_; }

    [[nodiscard]] std::optional<LocalizationEstimate> measure(
        const GroundTruthPose& truth, simulation::DeterministicRng& rng) const override;

private:
    GnssNoiseConfig config_;
};

// GNSS unavailable (outage / jamming / indoor): never a fix. Outages are
// scenario policy (which model is active when), not model internals.
class UnavailableGnss final : public GnssModel {
public:
    [[nodiscard]] std::optional<LocalizationEstimate> measure(
        const GroundTruthPose& truth, simulation::DeterministicRng& rng) const override;
};

}  // namespace fleet::localization

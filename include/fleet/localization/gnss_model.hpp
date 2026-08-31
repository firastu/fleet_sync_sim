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

// GNSS unavailable (outage / jamming / indoor): never a fix. Outages are
// scenario policy (which model is active when), not model internals.
class UnavailableGnss final : public GnssModel {
public:
    [[nodiscard]] std::optional<LocalizationEstimate> measure(
        const GroundTruthPose& truth, simulation::DeterministicRng& rng) const override;
};

}  // namespace fleet::localization

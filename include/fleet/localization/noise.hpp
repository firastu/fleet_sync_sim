#pragma once

#include <cstdint>

#include "fleet/simulation/deterministic_rng.hpp"

namespace fleet::localization {

// Draws consumed by one irwin_hall_standard_noise() call.
inline constexpr std::uint64_t kIrwinHallDraws = 12;

// The project's one deterministic noise sample (ADR-017, shared by every
// noisy sensor model since ADR-024): the sum of 12 midpoint-centered
// finite-resolution uniforms (k + 0.5) / 2^32 - 0.5, where k is one
// rng.uniform_below(2^32) draw. The result is exactly symmetric about zero
// with variance 1 - 2^-64, approximately standard normal, and every operation
// is exact in double arithmetic: no transcendental, no
// std::normal_distribution.
//
// Consumption contract: exactly kIrwinHallDraws uniform_below(2^32) calls,
// which for a power-of-two bound is exactly one engine output each.
[[nodiscard]] double irwin_hall_standard_noise(simulation::DeterministicRng& rng);

}  // namespace fleet::localization

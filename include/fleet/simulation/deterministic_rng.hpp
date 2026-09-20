#pragma once

#include <cassert>
#include <cstdint>
#include <random>

namespace fleet::simulation {

// Deterministic bounded sampler over std::mt19937_64.
//
// Why not std::uniform_int_distribution & friends: the engine's output
// sequence is specified by the C++ standard, but the algorithms behind
// the standard distributions are NOT — so cross-implementation replay is
// not guaranteed. This sampler implements the tiny sampling operations
// the simulator needs with fully specified arithmetic (ADR-006), making
// replays bit-identical across standard-library implementations.
//
// RNG-consumption contract (stable; deterministic replay depends on it):
//   uniform_below(bound) consumes engine output only when bound >= 2;
//   bound <= 1 returns 0 and consumes nothing.
//
// Thread-safety: not synchronized (ADR-002).
class DeterministicRng {
public:
    explicit DeterministicRng(std::uint64_t seed) noexcept;

    // Uniform integer in [0, bound), unbiased: rejection sampling with no
    // modulo bias. Precondition: bound >= 1.
    std::uint64_t uniform_below(std::uint64_t bound);

private:
    std::mt19937_64 engine_;
};

// Deterministic sub-stream seed derivation: maps (base seed, domain,
// stream index) to a per-stream seed by FULLY SPECIFIED integer
// arithmetic — one splitmix64 round over
//   base ^ (domain + PHI * (index + 1)),  PHI = 0x9E3779B97F4A7C15.
// No std::hash (unspecified), no unordered-container iteration, no
// address dependence: the same triple yields the same seed on every
// conforming platform (ADR-018's per-robot GNSS streams rely on it).
// Domain is caller policy (a fixed constant per subsystem; NEVER change
// a published constant — replay compatibility depends on it).
//
// The contract this buys: streams are independent of one another's
// SCHEDULING — adding, removing or rescheduling one consumer never
// shifts another consumer's random sequence.
[[nodiscard]] std::uint64_t derive_stream_seed(std::uint64_t base_seed,
                                               std::uint64_t domain,
                                               std::uint64_t stream_index) noexcept;

}  // namespace fleet::simulation

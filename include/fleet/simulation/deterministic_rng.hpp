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

}  // namespace fleet::simulation

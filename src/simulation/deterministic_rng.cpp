#include "fleet/simulation/deterministic_rng.hpp"

namespace fleet::simulation {

DeterministicRng::DeterministicRng(std::uint64_t seed) noexcept : engine_{seed} {}

std::uint64_t DeterministicRng::uniform_below(std::uint64_t bound) {
    assert(bound >= 1 && "DeterministicRng::uniform_below: bound must be >= 1");
    if (bound <= 1) {
        return 0;  // consumes no engine output (part of the contract)
    }
    // Classic unbiased rejection sampling. Unsigned wraparound is fully
    // specified: (2^64 - bound) % bound == 2^64 mod bound. Draws below
    // the threshold would bias the residues and are rejected; every
    // residue in the accepted region occurs equally often.
    const std::uint64_t threshold = (0 - bound) % bound;
    std::uint64_t draw = engine_();
    while (draw < threshold) {
        draw = engine_();
    }
    return draw % bound;
}

std::uint64_t derive_stream_seed(std::uint64_t base_seed, std::uint64_t domain,
                                 std::uint64_t stream_index) noexcept {
    // One splitmix64 round — exact specified arithmetic, no libm, no
    // std::hash. Unsigned 64-bit overflow is well-defined and part of
    // the contract (ADR-018). Do NOT change these constants or this
    // composition: published replay compatibility depends on them.
    constexpr std::uint64_t kPhi = 0x9E3779B97F4A7C15ULL;
    std::uint64_t x = base_seed ^ (domain + kPhi * (stream_index + 1));
    x += kPhi;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

}  // namespace fleet::simulation

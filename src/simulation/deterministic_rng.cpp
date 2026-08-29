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

}  // namespace fleet::simulation

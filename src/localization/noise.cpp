#include "fleet/localization/noise.hpp"

namespace fleet::localization {

namespace {

// One midpoint-centered uniform: (k + 0.5)/2^32 - 0.5 for the 32-bit
// integer draw k — a symmetric discrete support strictly inside
// (-0.5, +0.5), unlike k/2^32 - 0.5 whose [-0.5, +0.5) support has a
// tiny negative mean. Every operation is exact in double arithmetic
// (power-of-two division; values are multiples of 2^-33).
[[nodiscard]] double centered_uniform(simulation::DeterministicRng& rng) {
    const double draw = static_cast<double>(rng.uniform_below(1ULL << 32));
    return (draw + 0.5) / 4294967296.0 - 0.5;
}

}  // namespace

double irwin_hall_standard_noise(simulation::DeterministicRng& rng) {
    double sum = 0.0;
    for (std::uint64_t i = 0; i < kIrwinHallDraws; ++i) {
        sum += centered_uniform(rng);
    }
    return sum;
}

}  // namespace fleet::localization

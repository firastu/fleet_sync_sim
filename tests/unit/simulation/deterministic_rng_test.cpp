#include "fleet/simulation/deterministic_rng.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <random>

namespace {

using fleet::simulation::DeterministicRng;

TEST(DeterministicRngTest, SameSeedProducesIdenticalSequences) {
    DeterministicRng first{42};
    DeterministicRng second{42};
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(first.uniform_below(1'000'000), second.uniform_below(1'000'000));
    }
}

TEST(DeterministicRngTest, DifferentSeedsProduceDifferentSequences) {
    DeterministicRng first{1};
    DeterministicRng second{2};
    bool differ = false;
    for (int i = 0; i < 100; ++i) {
        differ = differ || first.uniform_below(1'000'000) != second.uniform_below(1'000'000);
    }
    EXPECT_TRUE(differ);
}

TEST(DeterministicRngTest, BoundOneConsumesNoEngineOutput) {
    DeterministicRng with_pause{7};
    DeterministicRng without_pause{7};
    (void)with_pause.uniform_below(1);  // must consume nothing

    EXPECT_EQ(with_pause.uniform_below(1'000'000), without_pause.uniform_below(1'000'000));
}

TEST(DeterministicRngTest, SamplesStayWithinBound) {
    DeterministicRng rng{99};
    for (int i = 0; i < 1000; ++i) {
        const std::uint64_t sample = rng.uniform_below(50);
        ASSERT_LT(sample, 50U);
    }
}

TEST(DeterministicRngTest, ReducesToRawEngineModuloWhenNoRejectionOccurs) {
    // For bound = 2^63 the rejection threshold (2^64 mod bound) is zero,
    // so the sampler reduces exactly to raw engine output % bound. This
    // pins the algorithm to the specified engine sequence.
    std::mt19937_64 raw{123};
    DeterministicRng rng{123};

    for (int i = 0; i < 10; ++i) {
        const std::uint64_t expected = raw() % (1ULL << 63);
        EXPECT_EQ(rng.uniform_below(1ULL << 63), expected);
    }
}

}  // namespace

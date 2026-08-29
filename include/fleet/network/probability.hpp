#pragma once

#include <compare>
#include <cstdint>
#include <stdexcept>

namespace fleet::network {

// Exact, fixed-resolution probability in parts per million:
//   0      = never
//   kScale = always
//
// No floating point: outcomes and comparisons stay exact and
// cross-platform (ADR-006). Constructed only through the factories;
// out-of-range values throw and invalid probabilities cannot exist.
class Probability {
public:
    static constexpr std::uint32_t kScale = 1'000'000;

    constexpr Probability() noexcept = default;  // never

    static constexpr Probability never() noexcept { return Probability{0}; }
    static constexpr Probability always() noexcept { return Probability{kScale}; }

    static Probability from_parts_per_million(std::uint32_t parts) {
        if (parts > kScale) {
            throw std::invalid_argument(
                "Probability: parts per million must not exceed 1'000'000");
        }
        return Probability{parts};
    }

    [[nodiscard]] constexpr std::uint32_t parts_per_million() const noexcept { return parts_; }

    constexpr auto operator<=>(const Probability&) const noexcept = default;

private:
    constexpr explicit Probability(std::uint32_t parts) noexcept : parts_{parts} {}

    std::uint32_t parts_ = 0;
};

}  // namespace fleet::network

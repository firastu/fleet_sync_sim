#pragma once

#include <compare>
#include <cstdint>

namespace fleet::common {

// Discrete logical time unit of the simulation. One tick is the smallest unit
// of progress the simulator can observe; conversion to/from wall-clock
// seconds is the responsibility of the simulation layer, not the domain.
//
// Ticks are used for ordering and age computations only. Behavioral
// correctness never depends on wall-clock time (see ADR-002).
struct Tick {
    std::uint64_t value = 0;

    constexpr auto operator<=>(const Tick&) const noexcept = default;

    constexpr Tick& operator+=(std::uint64_t ticks) noexcept {
        value += ticks;
        return *this;
    }

    friend constexpr Tick operator+(Tick lhs, std::uint64_t ticks) noexcept {
        lhs += ticks;
        return lhs;
    }
};

}  // namespace fleet::common

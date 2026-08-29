#pragma once

#include <cassert>
#include <cstdint>

#include "fleet/common/time.hpp"

namespace fleet::simulation {

// Authoritative logical time of one simulation execution.
//
// - 1 tick == 1 millisecond (kTicksPerSecond == 1000). The tick<->second
//   mapping is a simulation-layer concern; domain types use common::Tick
//   without units.
// - Time advances only when the EventQueue processes an event — never by
//   wall clock, sleeps or OS scheduling (determinism, ADR-002).
// - Monotonic: advance_to() never moves time backward.
//
// Thread-safety: not synchronized (ADR-002).
class SimulationClock {
public:
    static constexpr std::uint64_t kTicksPerSecond = 1000;

    [[nodiscard]] constexpr common::Tick now() const noexcept { return now_; }

    constexpr void advance_to(common::Tick tick) noexcept {
        assert(tick >= now_ && "SimulationClock: time cannot move backward");
        now_ = tick;
    }

private:
    common::Tick now_{};
};

}  // namespace fleet::simulation

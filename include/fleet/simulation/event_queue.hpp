#pragma once

#include <cstdint>
#include <functional>
#include <queue>
#include <vector>

#include "fleet/common/time.hpp"
#include "fleet/simulation/simulation_clock.hpp"

namespace fleet::simulation {

// Deterministic discrete-event scheduler: the single mechanism through
// which simulated time passes in Stage 0 (ADR-002, ADR-005).
//
// Each scheduled event is (at, order, effect). Execution order is a
// strict total order: earlier Tick first; on equal ticks, the order in
// which events were scheduled (a monotonically increasing enqueue
// counter). No heap, container or pointer tie-breaks are observable.
//
// The queue owns the SimulationClock. Simulated time advances only
// through explicit EventQueue operations — to an event's tick when it
// executes, or to the horizon requested by run_until() — and never by
// wall clock. Effects may query clock().now() and may schedule further
// events, including at the current tick: such a follow-up runs after
// the currently executing effect AND after every event already queued
// at the same tick, because the enqueue counter is assigned at
// schedule() time — a reentrant follow-up can never overtake an
// already-queued event.
//
// Causality: schedule() requires at >= now() and throws
// std::invalid_argument otherwise. A past-time event is a scenario-
// construction bug that would otherwise silently break reproducibility.
//
// Exception safety (basic, no rollback): EventQueue never catches
// effect exceptions. If an effect throws, the exception propagates to
// the caller; the throwing event is already consumed; the clock remains
// at that event's tick; all other events — including any the effect
// scheduled before throwing — remain queued, and the queue stays
// usable. run_to_completion() terminates only when the queue empties —
// a scenario that always schedules new events runs forever, which is
// the scenario's bug, not the queue's.
//
// Non-copyable and non-movable: scheduled effects typically capture
// references to simulation state.
//
// Thread-safety: not synchronized (ADR-002).
class EventQueue {
public:
    EventQueue() = default;
    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;
    EventQueue(EventQueue&&) = delete;
    EventQueue& operator=(EventQueue&&) = delete;

    // Schedules `effect` at tick `at` (must be >= clock().now()).
    void schedule(common::Tick at, std::function<void()> effect);

    [[nodiscard]] bool empty() const noexcept { return queue_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return queue_.size(); }
    [[nodiscard]] const SimulationClock& clock() const noexcept { return clock_; }

    // Processes the next event, advancing time to its tick.
    // Returns false if the queue is empty.
    bool step();

    // Processes all events with tick <= until (including events scheduled
    // during processing), then advances the clock to exactly `until`.
    // Postcondition: clock().now() == until — idle time is crossed
    // deterministically so relative scheduling afterwards is exact.
    // Precondition: until >= clock().now() (throws std::invalid_argument
    // otherwise — a backward horizon is a logic error).
    void run_until(common::Tick until);

    // Processes events until the queue is empty.
    void run_to_completion();

private:
    struct Scheduled {
        common::Tick at{};
        std::uint64_t order = 0;
        std::function<void()> effect;
    };

    // std::priority_queue surfaces the entry that compares greatest under
    // this comparator, so it returns true when `a` is *later* than `b`.
    // This yields the documented pop order: (at, order) ascending.
    struct Later {
        bool operator()(const Scheduled& a, const Scheduled& b) const noexcept;
    };

    std::priority_queue<Scheduled, std::vector<Scheduled>, Later> queue_;
    SimulationClock clock_;
    std::uint64_t next_order_ = 0;
};

}  // namespace fleet::simulation

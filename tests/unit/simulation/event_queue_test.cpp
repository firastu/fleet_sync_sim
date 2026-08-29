#include "fleet/simulation/event_queue.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet/common/time.hpp"
#include "fleet/simulation/simulation_clock.hpp"

namespace {

using fleet::common::Tick;
using fleet::simulation::EventQueue;
using fleet::simulation::SimulationClock;

TEST(SimulationClockTest, IsMonotonicWithUnitDefinedTicks) {
    SimulationClock clock;
    EXPECT_EQ(clock.now(), Tick{0});
    clock.advance_to(Tick{5000});
    EXPECT_EQ(clock.now(), Tick{5000});
    clock.advance_to(Tick{5000});  // advancing to the same tick is allowed
    EXPECT_EQ(clock.now(), Tick{5000});
    static_assert(SimulationClock::kTicksPerSecond == 1000);  // 1 tick = 1 ms
}

TEST(EventQueueTest, RunsEventsInTickOrder) {
    EventQueue queue;
    std::vector<int> trace;
    queue.schedule(Tick{5000}, [&trace] { trace.push_back(3); });
    queue.schedule(Tick{100}, [&trace] { trace.push_back(1); });
    queue.schedule(Tick{3000}, [&trace] { trace.push_back(2); });

    queue.run_to_completion();

    EXPECT_EQ(trace, (std::vector<int>{1, 2, 3}));
}

TEST(EventQueueTest, EqualTickEventsRunInScheduleOrder) {
    EventQueue queue;
    std::vector<int> trace;
    queue.schedule(Tick{100}, [&trace] { trace.push_back(1); });
    queue.schedule(Tick{100}, [&trace] { trace.push_back(2); });
    queue.schedule(Tick{100}, [&trace] { trace.push_back(3); });

    queue.run_to_completion();

    EXPECT_EQ(trace, (std::vector<int>{1, 2, 3}));
}

TEST(EventQueueTest, ClockAdvancesOnlyWhenEventsRun) {
    EventQueue queue;
    queue.schedule(Tick{500}, [] {});
    queue.schedule(Tick{200}, [] {});
    EXPECT_EQ(queue.clock().now(), Tick{0});  // scheduling does not advance time

    EXPECT_TRUE(queue.step());
    EXPECT_EQ(queue.clock().now(), Tick{200});
    EXPECT_TRUE(queue.step());
    EXPECT_EQ(queue.clock().now(), Tick{500});
    EXPECT_FALSE(queue.step());  // empty queue: no time advance
    EXPECT_EQ(queue.clock().now(), Tick{500});
}

TEST(EventQueueTest, ScheduleInPastThrows) {
    EventQueue queue;
    queue.schedule(Tick{100}, [] {});
    queue.run_to_completion();

    EXPECT_THROW(queue.schedule(Tick{99}, [] {}), std::invalid_argument);
    // Scheduling at the current tick is causal and allowed.
    queue.schedule(Tick{100}, [] {});
    EXPECT_EQ(queue.size(), 1U);
}

TEST(EventQueueTest, EffectsCanScheduleFurtherEvents) {
    EventQueue queue;
    std::vector<int> trace;
    queue.schedule(Tick{100}, [&] {
        trace.push_back(1);
        queue.schedule(Tick{300}, [&trace] { trace.push_back(3); });
        queue.schedule(Tick{100}, [&trace] { trace.push_back(2); });  // same tick
    });

    queue.run_to_completion();

    // Same-tick follow-up runs after the currently executing effect.
    EXPECT_EQ(trace, (std::vector<int>{1, 2, 3}));
}

TEST(EventQueueTest, RunUntilProcessesOnlyDueEvents) {
    EventQueue queue;
    std::vector<std::uint64_t> trace;
    queue.schedule(Tick{100}, [&trace] { trace.push_back(100); });
    queue.schedule(Tick{200}, [&trace] { trace.push_back(200); });
    queue.schedule(Tick{9000}, [&trace] { trace.push_back(9000); });

    queue.run_until(Tick{1000});

    EXPECT_EQ(trace, (std::vector<std::uint64_t>{100, 200}));
    EXPECT_EQ(queue.size(), 1U);
    EXPECT_EQ(queue.clock().now(), Tick{1000});  // horizon reached exactly
}

TEST(EventQueueTest, RunUntilAdvancesAcrossIdleTime) {
    EventQueue queue;
    std::vector<std::uint64_t> trace;
    queue.schedule(Tick{10000}, [&trace] { trace.push_back(10000); });

    queue.run_until(Tick{5000});

    EXPECT_TRUE(trace.empty());                 // no event executed
    EXPECT_EQ(queue.clock().now(), Tick{5000}); // horizon reached anyway
    EXPECT_EQ(queue.size(), 1U);                // far-future event remains
    // Scheduling before the horizon is now correctly rejected as past.
    EXPECT_THROW(queue.schedule(Tick{4999}, [] {}), std::invalid_argument);
}

TEST(EventQueueTest, RunUntilBackwardHorizonThrows) {
    EventQueue queue;
    queue.schedule(Tick{100}, [] {});
    queue.run_until(Tick{500});
    EXPECT_THROW(queue.run_until(Tick{499}), std::invalid_argument);
}

TEST(EventQueueTest, ReentrantSameTickSchedulingRunsAfterAlreadyQueuedEvents) {
    EventQueue queue;
    std::vector<char> trace;
    queue.schedule(Tick{100}, [&] {
        trace.push_back('A');
        // Scheduled while A executes; B was already queued at the same
        // tick and holds an earlier enqueue order.
        queue.schedule(Tick{100}, [&trace] { trace.push_back('C'); });
    });
    queue.schedule(Tick{100}, [&trace] { trace.push_back('B'); });

    queue.run_to_completion();

    // Enqueue order is assigned at schedule() time, so C can never
    // overtake B: the order is exactly A, B, C (ADR-005).
    EXPECT_EQ(trace, (std::vector<char>{'A', 'B', 'C'}));
}

TEST(EventQueueTest, EffectExceptionLeavesQueueConsistent) {
    EventQueue queue;
    std::vector<int> trace;
    queue.schedule(Tick{100}, [&] {
        trace.push_back(1);
        queue.schedule(Tick{150}, [&trace] { trace.push_back(2); });  // pre-throw schedule
        throw std::runtime_error("effect failure");
    });
    queue.schedule(Tick{200}, [&trace] { trace.push_back(3); });

    // Basic exception safety: propagates, event consumed, no rollback.
    EXPECT_THROW(queue.run_to_completion(), std::runtime_error);
    EXPECT_EQ(queue.clock().now(), Tick{100});  // stays at the failing event's tick
    EXPECT_EQ(queue.size(), 2U);                // @150 (pre-throw) and @200 remain

    // The queue remains usable and deterministic afterwards.
    queue.run_to_completion();
    EXPECT_EQ(trace, (std::vector<int>{1, 2, 3}));
}

TEST(EventQueueTest, ExecutionTraceIsDeterministic) {
    const auto build_and_run = [] {
        EventQueue queue;
        std::vector<std::string> trace;
        for (int i = 0; i < 50; ++i) {
            // 50 events over 20 distinct ticks: guaranteed collisions
            // exercise the (tick, enqueue order) tie-break.
            const Tick tick{static_cast<std::uint64_t>((i * 7) % 20)};
            queue.schedule(tick, [&trace, i, tick] {
                trace.push_back(std::format("t{}:e{}", tick.value, i));
            });
        }
        queue.run_to_completion();
        return trace;
    };

    const std::vector<std::string> first = build_and_run();
    const std::vector<std::string> second = build_and_run();
    ASSERT_EQ(first.size(), 50U);
    EXPECT_EQ(first, second);
}

}  // namespace

#include "fleet/network/network_simulator.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/map/map_reconciler.hpp"
#include "fleet/network/probability.hpp"
#include "fleet/simulation/event_queue.hpp"

namespace {

using fleet::common::EdgeId;
using fleet::common::OverlayVersion;
using fleet::common::RobotId;
using fleet::common::SequenceNumber;
using fleet::common::Tick;
using fleet::network::EndpointId;
using fleet::network::NetworkConfig;
using fleet::network::NetworkSimulator;
using fleet::network::Probability;
using fleet::map::DynamicMapOverlay;
using fleet::map::EdgeDynamicState;
using fleet::map::EdgeStatus;
using fleet::map::MapDelta;
using fleet::map::MapReconciler;
using fleet::map::ReconcileDecision;
using fleet::simulation::EventQueue;

constexpr EndpointId kSender{1};
constexpr EndpointId kReceiver{2};

MapDelta blocked_delta(EdgeId edge, RobotId source, SequenceNumber sequence, Tick at) {
    return MapDelta{
        edge,
        EdgeDynamicState{
            .status = EdgeStatus::Blocked,
            .observed_at = at,
            .source = source,
            .source_sequence = sequence,
            .confidence = 0.9,
        },
    };
}

struct Delivery {
    std::uint64_t tick = 0;
    std::uint8_t from = 0;
    EdgeId edge{};
    SequenceNumber sequence{};

    bool operator==(const Delivery&) const = default;
};

TEST(NetworkSimulatorTest, FixedLatencyDeliversAtExpectedTick) {
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}}, /*seed=*/1};
    std::vector<Delivery> received;
    network.add_endpoint(kReceiver, [&](EndpointId from, const MapDelta& payload) {
        received.push_back(Delivery{queue.clock().now().value, from.value(), payload.edge,
                                    payload.state.source_sequence});
    });

    queue.run_until(Tick{5000});
    const auto result = network.send(kSender, kReceiver,
                                     blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7},
                                                   Tick{5000}));
    ASSERT_EQ(result.scheduled_deliveries(), 1U);
    EXPECT_FALSE(result.dropped);
    EXPECT_EQ(result.delivery_ticks.front(), Tick{5080});

    queue.run_to_completion();
    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received.front().tick, 5080U);
    EXPECT_EQ(received.front().from, kSender.value());
}

TEST(NetworkSimulatorTest, GuaranteedLossSchedulesNoDelivery) {
    EventQueue queue;
    NetworkSimulator network{
        queue,
        NetworkConfig{.min_latency = Tick{80},
                      .max_latency = Tick{80},
                      .packet_loss = Probability::always()},
        /*seed=*/1};
    bool received_any = false;
    network.add_endpoint(kReceiver, [&received_any](EndpointId, const MapDelta&) {
        received_any = true;
    });

    const auto result =
        network.send(kSender, kReceiver, blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7},
                                                        Tick{0}));
    EXPECT_TRUE(result.dropped);
    EXPECT_EQ(result.scheduled_deliveries(), 0U);

    queue.run_to_completion();
    EXPECT_FALSE(received_any);
}

TEST(NetworkSimulatorTest, GuaranteedDuplicationDeliversTwiceIdentically) {
    EventQueue queue;
    NetworkSimulator network{
        queue,
        NetworkConfig{.min_latency = Tick{80},
                      .max_latency = Tick{80},
                      .duplication = Probability::always()},
        /*seed=*/1};
    std::vector<MapDelta> received;
    network.add_endpoint(kReceiver, [&received](EndpointId, const MapDelta& payload) {
        received.push_back(payload);
    });

    const MapDelta sent = blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7}, Tick{0});
    const auto result = network.send(kSender, kReceiver, sent);
    ASSERT_EQ(result.scheduled_deliveries(), 2U);
    // Fixed latency: same tick; enqueue order resolves the tie.
    EXPECT_EQ(result.delivery_ticks.front(), Tick{80});
    EXPECT_EQ(result.delivery_ticks.back(), Tick{80});

    queue.run_to_completion();
    ASSERT_EQ(received.size(), 2U);
    // Identity never mutated: both copies are the exact same MapDelta.
    EXPECT_EQ(received.front(), sent);
    EXPECT_EQ(received.back(), sent);
}

TEST(NetworkSimulatorTest, SameSeedProducesSameTrace) {
    const auto run = [] {
        EventQueue queue;
        NetworkSimulator network{
            queue,
            NetworkConfig{.min_latency = Tick{50},
                          .max_latency = Tick{300},
                          .packet_loss = Probability::from_parts_per_million(100'000),
                          .duplication = Probability::from_parts_per_million(200'000)},
            /*seed=*/12345};
        std::vector<Delivery> trace;
        network.add_endpoint(kReceiver, [&](EndpointId from, const MapDelta& payload) {
            trace.push_back(Delivery{queue.clock().now().value, from.value(), payload.edge,
                                     payload.state.source_sequence});
        });

        for (int i = 0; i < 20; ++i) {
            queue.run_until(Tick{static_cast<std::uint64_t>(1000 + i * 100)});
            network.send(kSender, kReceiver,
                         blocked_delta(EdgeId{static_cast<std::uint32_t>(i % 17)}, RobotId{1},
                                       SequenceNumber{static_cast<std::uint64_t>(i + 1)},
                                       Tick{static_cast<std::uint64_t>(1000 + i * 100)}));
        }
        queue.run_to_completion();
        return trace;
    };

    const std::vector<Delivery> first = run();
    const std::vector<Delivery> second = run();
    EXPECT_FALSE(first.empty());
    EXPECT_EQ(first, second);
}

TEST(NetworkSimulatorTest, VariableLatencyReordersMessages) {
    // Golden deterministic test: with this seed and latency range, the
    // second send (#18) samples a shorter latency than the first (#17)
    // plus the send gap, so #18 is delivered first — reordering emerging
    // purely from independently sampled latencies, never imposed.
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{70}, .max_latency = Tick{400}}, /*seed=*/42};
    std::vector<SequenceNumber> arrival_order;
    network.add_endpoint(kReceiver, [&arrival_order](EndpointId, const MapDelta& payload) {
        arrival_order.push_back(payload.state.source_sequence);
    });

    queue.run_until(Tick{5000});
    const auto first =
        network.send(kSender, kReceiver,
                     blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{17}, Tick{5000}));
    queue.run_until(Tick{5010});
    const auto second =
        network.send(kSender, kReceiver,
                     blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{18}, Tick{5010}));

    ASSERT_EQ(first.scheduled_deliveries(), 1U);
    ASSERT_EQ(second.scheduled_deliveries(), 1U);
    EXPECT_LT(second.delivery_ticks.front(), first.delivery_ticks.front());

    queue.run_to_completion();
    ASSERT_EQ(arrival_order.size(), 2U);
    EXPECT_EQ(arrival_order.front(), SequenceNumber{18});
    EXPECT_EQ(arrival_order.back(), SequenceNumber{17});
}

TEST(NetworkSimulatorTest, ZeroLatencyDeliversAtSameTick) {
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{0}, .max_latency = Tick{0}}, /*seed=*/1};
    std::uint64_t now_at_delivery = 0;
    network.add_endpoint(kReceiver, [&now_at_delivery, &queue](EndpointId, const MapDelta&) {
        now_at_delivery = queue.clock().now().value;
    });

    queue.run_until(Tick{100});
    const auto result =
        network.send(kSender, kReceiver,
                     blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7}, Tick{100}));
    EXPECT_EQ(result.delivery_ticks.front(), Tick{100});
    queue.run_to_completion();
    EXPECT_EQ(now_at_delivery, 100U);
}

TEST(NetworkSimulatorTest, TransportSenderIsDistinctFromObservationSource) {
    // EndpointId (transport address) and RobotId (observation source)
    // are different concepts; a relay could send from endpoint 3 a delta
    // originally observed by robot 1.
    EventQueue queue;
    NetworkSimulator network{queue, NetworkConfig{}, /*seed=*/1};
    std::uint8_t transport_from = 0;
    RobotId observation_source{};
    network.add_endpoint(kReceiver,
                         [&](EndpointId from, const MapDelta& payload) {
                             transport_from = from.value();
                             observation_source = payload.state.source;
                         });

    (void)network.send(EndpointId{3}, kReceiver,
                       blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7}, Tick{0}));
    queue.run_to_completion();

    EXPECT_EQ(transport_from, 3U);
    EXPECT_EQ(observation_source, RobotId{1});
}

TEST(NetworkSimulatorTest, SendToUnknownEndpointThrows) {
    EventQueue queue;
    NetworkSimulator network{queue, NetworkConfig{}, /*seed=*/1};
    EXPECT_THROW(network.send(kSender, EndpointId{9},
                              blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{1}, Tick{0})),
                 std::invalid_argument);
}

TEST(NetworkSimulatorTest, DoubleRegistrationThrows) {
    EventQueue queue;
    NetworkSimulator network{queue, NetworkConfig{}, /*seed=*/1};
    network.add_endpoint(kReceiver, [](EndpointId, const MapDelta&) {});
    EXPECT_THROW(network.add_endpoint(kReceiver, [](EndpointId, const MapDelta&) {}),
                 std::invalid_argument);
}

TEST(NetworkSimulatorTest, InvalidConfigThrows) {
    EventQueue queue;
    EXPECT_THROW(
        (NetworkSimulator{queue,
                          NetworkConfig{.min_latency = Tick{100}, .max_latency = Tick{50}},
                          /*seed=*/1}),
        std::invalid_argument);
    EXPECT_THROW(Probability::from_parts_per_million(1'000'001), std::invalid_argument);
}

TEST(NetworkSimulatorTest, DeliveryTickOverflowThrows) {
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{200}, .max_latency = Tick{200}}, /*seed=*/1};
    network.add_endpoint(kReceiver, [](EndpointId, const MapDelta&) {});

    queue.run_until(Tick{std::numeric_limits<std::uint64_t>::max() - 100});
    EXPECT_THROW(
        network.send(kSender, kReceiver,
                     blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{1}, Tick{0})),
        std::overflow_error);
}

TEST(NetworkSimulatorTest, DuplicateDeliveryExercisesReconcilerIdempotence) {
    EventQueue queue;
    NetworkSimulator network{
        queue,
        NetworkConfig{.min_latency = Tick{80},
                      .max_latency = Tick{80},
                      .duplication = Probability::always()},
        /*seed=*/1};
    DynamicMapOverlay overlay{17};
    MapReconciler reconciler{17};
    std::vector<ReconcileDecision> decisions;
    network.add_endpoint(kReceiver, [&](EndpointId, const MapDelta& payload) {
        decisions.push_back(reconciler.reconcile(payload, overlay));
    });

    network.send(kSender, kReceiver,
                 blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7}, Tick{0}));
    queue.run_to_completion();

    ASSERT_EQ(decisions.size(), 2U);
    EXPECT_EQ(decisions.front(), ReconcileDecision::Applied);
    EXPECT_EQ(decisions.back(), ReconcileDecision::Duplicate);
    EXPECT_EQ(overlay.version(), OverlayVersion{1});
}

TEST(NetworkSimulatorTest, DroppedDeltaLeavesReceiverUnchanged) {
    EventQueue queue;
    NetworkSimulator network{
        queue,
        NetworkConfig{.min_latency = Tick{80},
                      .max_latency = Tick{80},
                      .packet_loss = Probability::always()},
        /*seed=*/1};
    DynamicMapOverlay overlay{17};
    MapReconciler reconciler{17};
    network.add_endpoint(kReceiver, [&](EndpointId, const MapDelta& payload) {
        (void)reconciler.reconcile(payload, overlay);
    });

    network.send(kSender, kReceiver,
                 blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7}, Tick{0}));
    queue.run_to_completion();

    EXPECT_EQ(overlay.version(), OverlayVersion{0});
    EXPECT_EQ(overlay.tracked_count(), 0U);
}

TEST(NetworkSimulatorTest, ReorderedDifferentEdgesArePreservedEndToEnd) {
    // The network delivers A:20 (edge X) before A:19 (edge Y); the
    // per-(source, edge) progression model (ADR-004) must keep both.
    EventQueue queue;
    NetworkSimulator network{
        queue, NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}}, /*seed=*/1};
    DynamicMapOverlay overlay{17};
    MapReconciler reconciler{17};
    std::vector<SequenceNumber> arrival;
    network.add_endpoint(kReceiver, [&](EndpointId, const MapDelta& payload) {
        arrival.push_back(payload.state.source_sequence);
        (void)reconciler.reconcile(payload, overlay);
    });

    queue.run_until(Tick{5000});
    (void)network.send(kSender, kReceiver,
                       blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{20}, Tick{5000}));
    (void)network.send(kSender, kReceiver,
                       blocked_delta(EdgeId{8}, RobotId{1}, SequenceNumber{19}, Tick{5000}));
    queue.run_to_completion();

    ASSERT_EQ(arrival.size(), 2U);
    EXPECT_EQ(arrival.front(), SequenceNumber{20});  // delivered first
    EXPECT_EQ(arrival.back(), SequenceNumber{19});
    EXPECT_EQ(overlay.tracked_count(), 2U);  // both preserved
}

TEST(NetworkSimulatorTest, ScheduledDeliveryDoesNotDependOnNetworkSimulatorLifetime) {
    // Scheduled deliveries must not capture the NetworkSimulator: the
    // queue may outlive it. Especially meaningful under ASan.
    EventQueue queue;
    std::vector<Delivery> received;
    const MapDelta sent = blocked_delta(EdgeId{4}, RobotId{1}, SequenceNumber{7}, Tick{0});

    {
        NetworkSimulator network{
            queue, NetworkConfig{.min_latency = Tick{80}, .max_latency = Tick{80}}, /*seed=*/1};
        network.add_endpoint(kReceiver, [&](EndpointId from, const MapDelta& payload) {
            received.push_back(Delivery{queue.clock().now().value, from.value(), payload.edge,
                                        payload.state.source_sequence});
        });

        queue.run_until(Tick{5000});
        const auto result = network.send(kSender, kReceiver, sent);
        ASSERT_EQ(result.scheduled_deliveries(), 1U);
    }  // NetworkSimulator destroyed; delivery @5080 still pending.

    queue.run_to_completion();
    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received.front().tick, 5080U);
    EXPECT_EQ(received.front().from, kSender.value());
    EXPECT_EQ(received.front().edge, sent.edge);
    EXPECT_EQ(received.front().sequence, sent.state.source_sequence);
}

}  // namespace

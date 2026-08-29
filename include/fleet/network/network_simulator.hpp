#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "fleet/common/time.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/network/network_config.hpp"
#include "fleet/simulation/deterministic_rng.hpp"
#include "fleet/simulation/event_queue.hpp"

namespace fleet::network {

// Delivery callback: invoked with the transport sender and the (copied)
// payload when a transmission reaches its endpoint. The payload's own
// observation source remains delta.state.source — transport sender and
// observation source are distinct concepts (EndpointId vs RobotId).
using ReceiveHandler = std::function<void(EndpointId from, const map::MapDelta& payload)>;

// Simulator-side diagnostics for one logical send(). `delivery_ticks`
// lists the ticks of the scheduled copies in *scheduling* order, not
// arrival order (a duplicate copy may arrive before the nominal first
// copy). Empty when the transmission was dropped.
struct SendResult {
    bool dropped = false;
    std::vector<common::Tick> delivery_ticks;  // 0, 1 or 2 entries

    [[nodiscard]] std::size_t scheduled_deliveries() const noexcept {
        return delivery_ticks.size();
    }
};

// Deterministic unreliable point-to-point transport simulation on top of
// the EventQueue (ADR-006). Stage 0: single-threaded, no sockets, no
// wall clock, no Robot/ControlStation knowledge.
//
// Fault model per logical send(), all randomness sampled synchronously
// inside send() — never at delivery-event execution:
//
//   1. packet-loss trial: DROP => zero delivery events, done;
//   2. sample latency        -> schedule delivery at now + latency;
//   3. duplication trial (only for non-dropped transmissions);
//   4. if duplicated: sample a second, independent latency -> schedule
//      one additional copy of the same MapDelta value (identity never
//      mutated — duplicates intentionally exercise reconciler
//      idempotence). No recursive duplication; max 2 deliveries.
//
// Reordering is emergent: two sends sample independent latencies, so a
// later send may be delivered first. There is deliberately no
// reorder_probability.
//
// RNG: exactly one std::mt19937_64 stream, seeded explicitly. Outcomes
// therefore depend on the ordered sequence of send() calls; a single
// stream is a Stage-0 decision (documented in ADR-006). Consumption
// contract: trials at 0%/100% consume nothing; a latency range of
// min == max consumes nothing — pure configs draw zero numbers.
//
// Determinism: identical (seed, config, ordered sends, queue state)
// produce identical delivery traces. Wall clock, pointer values, thread
// scheduling and container iteration never influence behavior; the
// endpoint registry is used for exact-key lookup only and is never
// iterated to make decisions.
//
// Endpoints are permanent in Stage 0: registered once via
// add_endpoint(), no unregister/offline concept (that arrives with
// partition/link-state work). The destination handler is resolved and
// copied at SEND time — registrations never change, so delivery-time
// lookup would add no semantics. send() to an unknown destination throws
// std::invalid_argument immediately; duplicate registration also throws.
//
// Delivery handlers execute as EventQueue effects: exceptions propagate
// with ADR-005 semantics (no catch, no retry, no conversion to loss).
//
// Lifetime: delivery effects capture only what they need — the transport
// sender, a copy of the payload and a copy of the destination handler.
// The NetworkSimulator itself therefore does NOT need to outlive
// deliveries it has already scheduled. The EventQueue must outlive its
// own events, and any external state referenced by a ReceiveHandler
// (robots, overlays, ...) must outlive that handler's invocation.
//
// Thread-safety: not synchronized (ADR-002).
class NetworkSimulator {
public:
    NetworkSimulator(simulation::EventQueue& queue, NetworkConfig config, std::uint64_t seed);

    // Registers `endpoint`'s delivery handler. Throws std::invalid_argument
    // on an empty handler or on double registration (endpoints are
    // permanent in Stage 0).
    void add_endpoint(EndpointId endpoint, ReceiveHandler on_receive);

    // Enqueues one transmission into the simulated network, sampling all
    // outcomes now. Returns diagnostics (ticks of the scheduled copies).
    // Precondition: `to` is a registered endpoint.
    SendResult send(EndpointId from, EndpointId to, const map::MapDelta& payload);

    [[nodiscard]] const NetworkConfig& config() const noexcept { return config_; }

private:
    [[nodiscard]] bool trial(const Probability& probability);
    [[nodiscard]] common::Tick sample_latency();
    [[nodiscard]] common::Tick delivery_tick_after(common::Tick latency) const;

    simulation::EventQueue& queue_;
    NetworkConfig config_;
    simulation::DeterministicRng rng_;
    std::uint64_t latency_span_;  // max - min + 1, precomputed
    std::unordered_map<EndpointId, ReceiveHandler> endpoints_;
};

}  // namespace fleet::network

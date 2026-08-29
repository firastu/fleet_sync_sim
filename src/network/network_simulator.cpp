#include "fleet/network/network_simulator.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace fleet::network {

NetworkSimulator::NetworkSimulator(simulation::EventQueue& queue, NetworkConfig config,
                                   std::uint64_t seed)
    : queue_{queue}, config_{config}, rng_{seed} {
    if (config_.min_latency > config_.max_latency) {
        throw std::invalid_argument("NetworkSimulator: min_latency exceeds max_latency");
    }
    if (config_.max_latency.value == std::numeric_limits<std::uint64_t>::max()) {
        // The latency span (max - min + 1) would overflow uint64.
        throw std::invalid_argument("NetworkSimulator: max_latency must leave room for the span");
    }
    latency_span_ = config_.max_latency.value - config_.min_latency.value + 1;
}

void NetworkSimulator::add_endpoint(EndpointId endpoint, ReceiveHandler on_receive) {
    if (!on_receive) {
        throw std::invalid_argument("NetworkSimulator::add_endpoint: handler must not be empty");
    }
    if (endpoints_.contains(endpoint)) {
        throw std::invalid_argument("NetworkSimulator::add_endpoint: endpoint already registered");
    }
    endpoints_.emplace(endpoint, std::move(on_receive));
}

SendResult NetworkSimulator::send(EndpointId from, EndpointId to, const map::MapDelta& payload) {
    const auto receiver = endpoints_.find(to);
    if (receiver == endpoints_.end()) {
        throw std::invalid_argument("NetworkSimulator::send: unknown destination endpoint");
    }

    SendResult result;

    // All randomness is sampled here, synchronously at send time.
    if (trial(config_.packet_loss)) {
        result.dropped = true;
        return result;  // dropped: zero delivery events
    }

    result.delivery_ticks.push_back(delivery_tick_after(sample_latency()));
    if (trial(config_.duplication)) {
        // One additional copy with an independently sampled latency; the
        // payload value (and therefore the delta identity) is unchanged.
        result.delivery_ticks.push_back(delivery_tick_after(sample_latency()));
    }

    // Resolve the destination handler at SEND time — endpoints are
    // permanent in Stage 0, so delivery-time lookup would add no
    // semantics. Each scheduled delivery owns everything it needs (the
    // transport sender, a payload copy and a handler copy); nothing
    // captures `this`, so deliveries already scheduled remain valid even
    // if the NetworkSimulator itself is destroyed. Exceptions propagate
    // per ADR-005.
    const ReceiveHandler handler = receiver->second;
    for (const common::Tick at : result.delivery_ticks) {
        queue_.schedule(at, [handler, from, payload] { handler(from, payload); });
    }
    return result;
}

bool NetworkSimulator::trial(const Probability& probability) {
    // Trivial probabilities consume no engine output, keeping the
    // consumption contract simple for pure configurations.
    if (probability.parts_per_million() == 0) {
        return false;
    }
    if (probability.parts_per_million() == Probability::kScale) {
        return true;
    }
    return rng_.uniform_below(Probability::kScale) < probability.parts_per_million();
}

common::Tick NetworkSimulator::sample_latency() {
    // uniform_below(1) consumes nothing, so min == max configurations
    // draw no numbers at all.
    const std::uint64_t offset = rng_.uniform_below(latency_span_);
    return common::Tick{config_.min_latency.value + offset};
}

common::Tick NetworkSimulator::delivery_tick_after(common::Tick latency) const {
    const std::uint64_t now = queue_.clock().now().value;
    const std::uint64_t span = latency.value;
    if (now > std::numeric_limits<std::uint64_t>::max() - span) {
        throw std::overflow_error("NetworkSimulator: delivery tick would overflow Tick");
    }
    return common::Tick{now + span};
}

}  // namespace fleet::network

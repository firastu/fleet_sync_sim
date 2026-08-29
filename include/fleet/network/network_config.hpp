#pragma once

#include "fleet/common/time.hpp"
#include "fleet/network/probability.hpp"

namespace fleet::network {

// Simulated link fault model. Latency variation is the closed interval
// [min_latency, max_latency] — jitter needs no separate field, and
// message reordering emerges naturally from independently sampled
// latencies. min_latency == 0 is legal (same-tick delivery).
//
// Validation happens in NetworkSimulator construction: invalid
// configurations throw and are never silently normalized.
struct NetworkConfig {
    common::Tick min_latency{80};
    common::Tick max_latency{80};
    Probability packet_loss = Probability::never();
    Probability duplication = Probability::never();
};

}  // namespace fleet::network

#pragma once

#include <cstdint>

#include "fleet/common/strong_type.hpp"

namespace fleet::network {

struct EndpointIdTag {};

// Transport-layer endpoint address: where a transmission comes from and
// goes to on the simulated network.
//
// Deliberately a DIFFERENT concept from common::RobotId, which is the
// observation *source* recorded inside EdgeDynamicState/MapDelta. When
// relaying arrives in later stages, the transport sender (an EndpointId)
// may differ from the original observer (delta.state.source):
//
//   envelope: from = A, to = C   carries   delta: source = B, edge = X
//
// The two types do not convert implicitly; the distinction is intentional
// (ADR-006).
using EndpointId = fleet::common::StrongType<EndpointIdTag, std::uint8_t>;

}  // namespace fleet::network

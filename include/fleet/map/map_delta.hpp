#pragma once

#include "fleet/map/dynamic_overlay.hpp"

namespace fleet::map {

// One dynamic-map observation as an exchangeable value: which edge, and
// the observation itself (status + provenance: source, source sequence,
// observation tick, confidence).
//
// Event identity and ordering fields are *derived from* the contained
// EdgeDynamicState rather than duplicated next to it, so a delta and its
// payload can never disagree (single source of truth — ADR-004).
//
// Semantic event identity at the reconciliation layer is
// (source, edge, source_sequence): each (source, edge) pair is an
// independently ordered event stream, and equal sequence numbers on
// different edges are independent events (ADR-004).
//
// This type describes the observation/event only. Transport metadata
// (delivery time, hop count, message id, ...) belongs to the future
// message envelope, not here.
//
// Producer contract: sequence numbers start at 1 and increase
// monotonically within each (source, edge) stream; observed_at is logical
// simulation time; source identifies the participant (robot or control
// station). A producer MAY allocate sequences from one source-global
// counter — a stronger policy the reconciler does not depend on.
//
// Thread-safety: plain value type.
struct MapDelta {
    common::EdgeId edge{};
    EdgeDynamicState state{};

    auto operator<=>(const MapDelta&) const = default;
};

}  // namespace fleet::map

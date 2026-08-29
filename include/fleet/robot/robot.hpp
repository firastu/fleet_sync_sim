#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/map/map_reconciler.hpp"
#include "fleet/map/map_view.hpp"
#include "fleet/planning/a_star_planner.hpp"
#include "fleet/planning/route.hpp"
#include "fleet/robot/mission.hpp"

namespace fleet::robot {

// Where a robot's outgoing deltas go. Wired by the application to the
// transport; the robot itself knows nothing about networking, transport
// addresses or delivery diagnostics (ADR-007).
using DeltaSink = std::function<void(const map::MapDelta& delta)>;

// Result of one observation reaching a robot.
struct ObservationResult {
    map::MapDelta delta;                  // the event created and emitted
    map::ReconcileDecision local_decision{};
    bool replanned = false;               // a replan was triggered (route
                                          // invalidated or improvement)
};

// One autonomous participant: knowledge (overlay + reconciler), a
// mission, and the current route planned on its own knowledge (ADR-007).
// Invariant: current_route() is the deterministic best route under the
// robot's current knowledge — not merely "a route not yet broken".
//
// Local observations and received deltas share the same reconciliation
// path (ADR-004). observe() allocates the next sequence in this robot's
// per-(source, edge) event stream (sequences start at 1), reconciles
// locally (typically Applied; Dominated when another source holds
// better knowledge), and forwards the delta to the sink regardless of
// the local outcome — the observation is valid information for other
// participants. receive() reconciles a transported delta. Received
// deltas are NOT rebroadcast: relaying/gossip is future work.
//
// Producer contract (enforced in release builds): within one of this
// robot's per-edge streams, observation ticks are non-decreasing.
// observe() throws std::invalid_argument for a tick earlier than the
// stream's last observation, BEFORE any sequence, state or emission is
// touched; same-tick observations are legal (monotonic, not strictly
// increasing). Sequence exhaustion (UINT64_MAX per stream) throws
// std::overflow_error — sequences never wrap, because 0 is reserved
// (ADR-004).
//
// Replanning is driven by the EFFECTIVE routing change of an Applied
// delta, not by the decision alone:
//   - traversal became worse (OPEN -> BLOCKED, or cost increased):
//     replan only if the current route uses the affected edge;
//   - traversal became better (BLOCKED -> OPEN, or cost decreased):
//     replan regardless of the current route — a shorter route may
//     have become available again;
//   - effective traversal unchanged (provenance/time refresh): no
//     replan.
// The route is recomputed from the mission start on current knowledge;
// Stage 0 has no movement, so replanning from the current position
// arrives with movement.
//
// DeltaSink exception semantics: the sink is invoked after the local
// observation is fully committed (sequence consumed, knowledge and
// route updated). A throwing sink propagates and nothing is rolled
// back — the observation identity stays consumed.
//
// Lifetime: borrows `base` (must outlive the robot) and calls `sink`
// synchronously inside observe(); the sink and any state it captures
// must outlive the robot's use.
//
// Thread-safety: not synchronized (ADR-002). Deterministic: no
// unordered-container iteration influences behavior (exact lookups
// only).
class Robot {
public:
    Robot(common::RobotId id, Mission mission, const map::BaseMap& base, DeltaSink sink);

    // A world observation reaches this robot at logical time `at`.
    // Throws std::invalid_argument if `at` is earlier than this edge
    // stream's last observation, and std::overflow_error if the stream's
    // sequence space is exhausted; in both cases nothing changes and the
    // sink is not invoked.
    ObservationResult observe(common::EdgeId edge, map::EdgeStatus status, common::Tick at,
                              double confidence = 1.0);

    // A delta received from the transport. Returns the local decision.
    map::ReconcileDecision receive(const map::MapDelta& delta);

    // State synchronization for reconnects and late joiners: re-emits the
    // robot's current knowledge winners through the sink, in ascending
    // EdgeId order, with their ORIGINAL event identity (source, sequence,
    // observation time unchanged). Receivers that already know a fact
    // suppress it as Duplicate; receivers that missed it Apply it —
    // ADR-004 idempotence makes re-announcement safe. This is explicit
    // state sync driven by scenario policy, not gossip of individually
    // received deltas. Returns the number of deltas emitted.
    std::size_t resynchronize();

    [[nodiscard]] common::RobotId id() const noexcept { return id_; }
    [[nodiscard]] const Mission& mission() const noexcept { return mission_; }
    [[nodiscard]] const planning::Route& current_route() const noexcept { return route_; }
    [[nodiscard]] const map::DynamicMapOverlay& overlay() const noexcept { return overlay_; }

private:
    // Production state of one of this robot's per-edge event streams.
    struct StreamProgress {
        std::uint64_t last_sequence = 0;
        common::Tick last_observed_at{};
    };

    struct ReconcileOutcome {
        map::ReconcileDecision decision{};
        bool replanned = false;
    };

    // Shared apply path for local and remote deltas: captures the edge's
    // effective traversal cost before reconciliation, reconciles, and
    // replans according to the effective change (see class comment).
    ReconcileOutcome reconcile_and_maybe_replan(const map::MapDelta& delta);

    void plan_current_route();

    common::RobotId id_;
    Mission mission_;
    const map::BaseMap& base_;
    DeltaSink sink_;
    planning::AStarPlanner planner_;  // zero heuristic: Dijkstra-equivalent
    map::DynamicMapOverlay overlay_;
    map::MapReconciler reconciler_;
    planning::Route route_;
    // This robot's own event streams, keyed by edge (sparse; identity is
    // (source, edge, sequence), ADR-004).
    std::unordered_map<common::EdgeId, StreamProgress> streams_;
};

}  // namespace fleet::robot

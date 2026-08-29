#include "fleet/robot/robot.hpp"

#include <cassert>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fleet::robot {

Robot::Robot(common::RobotId id, Mission mission, const map::BaseMap& base, DeltaSink sink)
    : id_{id},
      mission_{mission},
      base_{base},
      sink_{std::move(sink)},
      overlay_{base.graph().edge_count()},
      reconciler_{base.graph().edge_count()} {
    plan_current_route();
}

ObservationResult Robot::observe(common::EdgeId edge, map::EdgeStatus status, common::Tick at,
                                 double confidence) {
    // Producer contract first, before touching any state: within one
    // per-edge stream, observation ticks are non-decreasing (ADR-004).
    const auto stream = streams_.find(edge);
    if (stream != streams_.end() && at < stream->second.last_observed_at) {
        throw std::invalid_argument(
            "Robot::observe: observation tick moved backward in this edge's stream");
    }
    const std::uint64_t last_sequence =
        stream != streams_.end() ? stream->second.last_sequence : 0;
    if (last_sequence == std::numeric_limits<std::uint64_t>::max()) {
        // SequenceNumber{0} is reserved; wrapping would corrupt event
        // identity (ADR-004).
        throw std::overflow_error("Robot::observe: sequence space exhausted for this edge");
    }
    const std::uint64_t next = last_sequence + 1;

    // Commit stream progression before emission: a throwing sink must
    // not make the observation identity vanish.
    streams_[edge] = StreamProgress{next, at};

    const map::MapDelta delta{
        edge,
        map::EdgeDynamicState{
            .status = status,
            .observed_at = at,
            .source = id_,
            .source_sequence = common::SequenceNumber{next},
            .confidence = confidence,
        },
    };

    // Same reconciliation path as received deltas (ADR-004). A fresh
    // sequence is never Stale/Duplicate; it can be Dominated when
    // another source currently holds better knowledge for this edge.
    const ReconcileOutcome outcome = reconcile_and_maybe_replan(delta);

    // Emit regardless of the local outcome: the observation is valid
    // information for other participants even when locally dominated.
    // Sink exceptions propagate; the local observation stays committed.
    if (sink_) {
        sink_(delta);
    }
    return ObservationResult{delta, outcome.decision, outcome.replanned};
}

map::ReconcileDecision Robot::receive(const map::MapDelta& delta) {
    return reconcile_and_maybe_replan(delta).decision;
}

std::size_t Robot::resynchronize() {
    if (!sink_) {
        return 0;
    }
    // tracked_edges() is sorted (deterministic); each winner is re-emitted
    // with its original identity, so remote reconciliation is idempotent.
    const std::vector<common::EdgeId> edges = overlay_.tracked_edges();
    for (const common::EdgeId edge : edges) {
        const map::EdgeDynamicState* state = overlay_.find(edge);
        assert(state != nullptr && "Robot::resynchronize: tracked edge missing state");
        sink_(map::MapDelta{edge, *state});
    }
    return edges.size();
}

Robot::ReconcileOutcome Robot::reconcile_and_maybe_replan(const map::MapDelta& delta) {
    // The view reads the live overlay, so the same view observes the
    // effective traversal cost before and after reconciliation.
    const map::MapView view{base_, overlay_};
    const std::optional<double> before = view.traversal_cost(delta.edge);

    const map::ReconcileDecision decision = reconciler_.reconcile(delta, overlay_);
    if (decision != map::ReconcileDecision::Applied) {
        // Knowledge unchanged (Duplicate/Stale/Dominated/RejectedConflict):
        // the effective routing state cannot have changed.
        return ReconcileOutcome{decision, false};
    }

    const std::optional<double> after = view.traversal_cost(delta.edge);

    // Effective-change replanning policy:
    //   worse  (traversable -> blocked, or costlier): replan only if the
    //          current route uses the edge;
    //   better (blocked -> traversable, or cheaper): replan regardless —
    //          a shorter route may be available again;
    //   unchanged (provenance/time refresh): no replan.
    bool replan = false;
    if (before.has_value() != after.has_value()) {
        replan = !before.has_value() || (route_.found && route_.uses_edge(delta.edge));
    } else if (before.has_value() && *after < *before) {
        replan = true;
    } else if (before.has_value() && *after > *before) {
        replan = route_.found && route_.uses_edge(delta.edge);
    }

    if (replan) {
        plan_current_route();
    }
    return ReconcileOutcome{decision, replan};
}

void Robot::plan_current_route() {
    // The view borrows this robot's own members and exists only for the
    // planning call (cheap value object, never cached across changes).
    route_ = planner_.plan(map::MapView{base_, overlay_}, mission_.start, mission_.goal);
}

}  // namespace fleet::robot


#include "fleet/station/control_station.hpp"

namespace fleet::station {

ControlStation::ControlStation(const map::BaseMap& base)
    : overlay_{base.graph().edge_count()}, reconciler_{base.graph().edge_count()} {}

map::ReconcileDecision ControlStation::receive(const map::MapDelta& delta) {
    return reconciler_.reconcile(delta, overlay_);
}

}  // namespace fleet::station

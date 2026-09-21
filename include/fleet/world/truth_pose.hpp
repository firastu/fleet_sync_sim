#pragma once

#include "fleet/common/time.hpp"
#include "fleet/localization/pose.hpp"
#include "fleet/map/base_map.hpp"
#include "fleet/robot/robot_state.hpp"

namespace fleet::world {

// Simulation-side truth-pose derivation (#16, ADR-018): where a robot
// PHYSICALLY is, from its movement state and the map's geographic side.
// The mirror of ADR-011's rule for edges, applied to pose: truth lives on
// the simulation side; robots never see it — it crosses to robot-local
// belief only through a localization::GnssModel. It is NEVER synthesized
// from a localization estimate.
//
// Derivation contract (deterministic, from the existing movement timing
// contract only — planning cost is not treated as meters):
//
//   - At a topology node (not in transit): the node's canonical
//     Wgs84Coordinate, heading = at_rest_heading_rad — the caller-held
//     SIMULATION-SIDE physical orientation. This is real modeled truth,
//     not a placeholder: the orientation is an explicit physical
//     initial condition (north until first movement) that the caller
//     maintains kinematically as graph movement occurs — at every
//     arrival it becomes the completed traversal's final-segment
//     bearing. Instantaneous re-orientation at graph corners is the
//     accepted abstraction (no steering is modeled); what never happens
//     is an orientation reset merely because the robot is stationary.
//     The value must be finite (normalized internally).
//
//   - In transit on an edge over [departed_at, arrival]: progress is the
//     elapsed-tick fraction, mapped onto the edge's geometry by CUMULATIVE
//     PHYSICAL LENGTH (haversine meters) along the polyline — never by
//     point index. Without a stored polyline the two endpoint node
//     coordinates define a straight segment (the documented rendering
//     fallback, ADR-012). The live travel bearing is the heading.
//
//   - Traversal direction respects the physical motion: the stored
//     polyline is canonical a->b, but a transit from the edge's `b` to
//     `a` (Bidirectional reverse use, or EdgeDirection::Reverse —
//     ADR-013) walks the SAME canonical geometry mirrored, and heading
//     is the bearing of travel, not of stored point order.
//
//   - Heading is the initial bearing of the polyline segment containing
//     the interpolated point, in travel direction. Segments own their
//     arc-length interval half-open [start, end), and the final segment
//     additionally owns its end point — so progress exactly at an
//     interior vertex belongs to the segment that STARTS there (the
//     outgoing one), progress 0 belongs to the first segment, and the
//     arrival point belongs to the final segment. Deterministic at every
//     vertex.
//
// Failure domain (explicit, never manufactured):
//   - std::invalid_argument when the map carries no geographic side, a
//     required node has no coordinate, or the transit's endpoints do not
//     match its edge.
//
// Reproducibility scope: haversine/bearing use ordinary floating-point
// geographic arithmetic (the ADR-014/017 libm class) — same-toolchain
// bit-identity, exactly like the geodetic seam of ADR-017.
//
// Thread-safety: pure function (ADR-002).
[[nodiscard]] localization::GroundTruthPose truth_pose(const map::BaseMap& base,
                                                       const robot::RobotState& state,
                                                       double at_rest_heading_rad,
                                                       common::Tick now);

[[nodiscard]] double localization_position_error_m(
    const localization::GroundTruthPose& truth,
    const localization::LocalizationEstimate& estimate);

}  // namespace fleet::world

#pragma once

#include <map>
#include <string>
#include <string_view>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"

namespace fleet::isaac {

// The pinned experiment origin of the bounded replay fixture (degrees,
// ADR-020 / docs/isaac/ARCHITECTURE.md #3). Adapter configuration, not a
// fact any simulator infers from a stage file.
inline constexpr double kFixtureOriginLatitude = 52.370;
inline constexpr double kFixtureOriginLongitude = 9.730;

// The Isaac stage 1 replay fixture map: a purpose-built L inside the 50 m
// footprint of the pinned origin —
//
//   C
//   |
//   B     B ~22.2 m north of A; C ~20.4 m east of B;
//   |     the farthest node (C) is ~30.3 m from the origin.
//   A
//
// Straight A->B, a corner at B, straight B->C, stop at C: the movement
// sequence the stage 1 acceptance display expects. Node positions in the
// graph are abstract planning units (edge cost 1.0); physical meters exist
// only on the geographic side.
struct FixtureMap {
    map::BaseMap base;
    std::map<std::string, common::NodeId> node_ids;  // ordered map: deterministic lookups

    [[nodiscard]] common::NodeId node(std::string_view name) const {
        return node_ids.at(std::string{name});
    }
};

[[nodiscard]] FixtureMap build_fixture_map();

}  // namespace fleet::isaac

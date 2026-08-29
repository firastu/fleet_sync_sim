#pragma once

#include <map>
#include <string>
#include <string_view>

#include "fleet/common/ids.hpp"
#include "fleet/map/base_map.hpp"

namespace fleet::app {

// Application-local demo data. Deliberately not part of the production
// library: it exists so main.cpp stays small until the scenario loader
// (later commit) replaces hand-built maps.

struct DemoMap {
    fleet::map::BaseMap base;
    std::map<std::string, fleet::common::NodeId> node_ids;

    [[nodiscard]] fleet::common::NodeId node(std::string_view name) const {
        return node_ids.at(std::string{name});
    }
};

// The 3x4 demo road network (same topology as the unit-test fixture):
//
//   A -- B -- C -- D
//   |    |    |    |
//   E -- F -- G -- H
//   |    |    |    |
//   I -- J -- K -- L
//
// Unit-spaced grid; all edges default to Euclidean cost 1.0.
[[nodiscard]] DemoMap build_demo_map();

}  // namespace fleet::app

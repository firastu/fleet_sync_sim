#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "fleet/map/base_map.hpp"
#include "fleet/scenario/scenario.hpp"

namespace fleet::scenario {

// Which scenario domain a file declares (ADR-023). A file without a
// "domain" field is a Graph scenario, so every pre-M4 scenario is unchanged.
enum class ScenarioDomain : std::uint8_t {
    Graph,    // road-graph scenarios: ScenarioLoader + ScenarioRunner
    Terrain,  // terrain scenarios: TerrainScenarioLoader + TerrainScenarioRunner
};

// Parses scenario JSON into typed Scenario data (ADR-009). Parser types
// never leak past this boundary: simulation code consumes only Scenario.
//
// The loader resolves author vocabulary — node names ("A"), edge names
// ("F-G"), participant names ("robot_a", "station") — into the strong ids
// used by the domain, and validates the whole description up front:
// every failure throws std::invalid_argument with a deterministic,
// human-readable message, before any simulation state exists.
//
// Supported actions are exactly the semantics that already exist:
//   set_link_state { from, to, up }
//   observe_edge   { robot, edge, state, confidence? }
//   resynchronize  { robot }
//
// Events are stably sorted by tick; equal-tick events keep file order,
// which becomes execution order via ADR-005 enqueue order. Because an
// observation's tick IS its observation time, sorting also guarantees the
// Robot producer contract (non-decreasing per-edge observation ticks) by
// construction.
//
// A file declaring any "domain" other than "graph" is rejected with a
// message naming the domain (ADR-023): load terrain scenarios with
// TerrainScenarioLoader. domain_of() reads only the "domain" field, so
// callers can dispatch before choosing a loader.
class ScenarioLoader {
public:
    [[nodiscard]] static ScenarioDomain domain_of(const std::filesystem::path& file);
    [[nodiscard]] static Scenario load(const map::BaseMap& base,
                                       const std::filesystem::path& file);
    [[nodiscard]] static Scenario load_string(const map::BaseMap& base,
                                              const std::string& json);
};

}  // namespace fleet::scenario

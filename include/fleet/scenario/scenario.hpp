#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "fleet/common/ids.hpp"
#include "fleet/common/time.hpp"
#include "fleet/map/dynamic_overlay.hpp"
#include "fleet/network/endpoint_id.hpp"
#include "fleet/network/network_config.hpp"
#include "fleet/robot/mission.hpp"

namespace fleet::scenario {

// Seed precedence (ADR-009): CLI --seed > scenario-file seed > kDefaultSeed.
inline constexpr std::uint64_t kDefaultSeed = 0;

[[nodiscard]] constexpr std::uint64_t resolve_seed(
    std::optional<std::uint64_t> cli_seed, std::optional<std::uint64_t> scenario_seed) noexcept {
    if (cli_seed.has_value()) {
        return *cli_seed;
    }
    if (scenario_seed.has_value()) {
        return *scenario_seed;
    }
    return kDefaultSeed;
}

// One robot declaration inside a scenario. Names and endpoints are the
// scenario author's vocabulary; ids are the observation-source identity
// recorded inside every MapDelta this robot produces.
struct ScenarioRobot {
    std::string name;
    common::RobotId id{};
    network::EndpointId endpoint{};
    robot::Mission mission{};
};

// Actions are deliberately limited to semantics that already exist
// (ADR-001..ADR-008). New actions arrive with the commits that add the
// underlying semantics — never before.
struct SetLinkAction {
    network::EndpointId from{};
    network::EndpointId to{};
    bool up = true;
};

struct ObserveEdgeAction {
    common::RobotId robot{};
    common::EdgeId edge{};
    map::EdgeStatus status = map::EdgeStatus::Open;
    double confidence = 1.0;
};

struct ResynchronizeAction {
    common::RobotId robot{};
};

using ScenarioAction = std::variant<SetLinkAction, ObserveEdgeAction, ResynchronizeAction>;

struct ScenarioEvent {
    common::Tick at{};
    ScenarioAction action{};
};

// Declarative description of one deterministic simulation run. Pure data:
// no wiring, no scheduling, no domain behavior. Execution order of equal-tick
// events is file order (guaranteed by the loader's stable sort and ADR-005's
// enqueue-order tie-break).
struct Scenario {
    std::string name;
    std::optional<std::uint64_t> seed;  // file-declared; CLI overrides
    network::NetworkConfig network{};
    bool has_station = false;
    network::EndpointId station_endpoint{};
    std::vector<ScenarioRobot> robots;  // declaration order = fan-out order
    std::vector<ScenarioEvent> events;  // sorted by (at, file order)
};

}  // namespace fleet::scenario

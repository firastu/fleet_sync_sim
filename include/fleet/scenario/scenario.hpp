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
// (ADR-001..ADR-011). New actions arrive with the commits that add the
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

// A world ground-truth change (ADR-011): what actually happened, as
// opposed to observe_edge, which tells a robot what it observed. Valid
// regardless of sensing configuration — the world evolves independently
// of observers; without a sensor, robots remain unaware.
struct SetWorldEdgeStateAction {
    common::EdgeId edge{};
    map::EdgeStatus status = map::EdgeStatus::Open;
};

using ScenarioAction =
    std::variant<SetLinkAction, ObserveEdgeAction, ResynchronizeAction,
                 SetWorldEdgeStateAction>;

struct ScenarioEvent {
    common::Tick at{};
    ScenarioAction action{};
};

// Movement is scenario opt-in (ADR-010): absent = static fleet (the
// founding scenario's behavior is unchanged). Traversal time of one edge
// is ceil(effective_cost * ms_per_cost_unit) ticks; a robot with no
// usable route parks and retries every retry_ms ticks.
struct MovementSettings {
    bool enabled = false;
    std::uint64_t ms_per_cost_unit = 1000;
    std::uint64_t retry_ms = 1000;
};

// Position-based sensing is scenario opt-in (ADR-011). One mode exists in
// Stage 0: "perfect_local" (PerfectLocalEdgeSensor). Sensing triggers:
// at every world truth change (robots in range) and at every movement
// arrival. Direct observation injection (observe_edge) remains available
// as a low-level testing primitive regardless.
struct SensingSettings {
    bool enabled = false;
};

// Declarative description of one deterministic simulation run. Pure data:
// no wiring, no scheduling, no domain behavior. Execution order of equal-tick
// events is file order (guaranteed by the loader's stable sort and ADR-005's
// enqueue-order tie-break).
struct Scenario {
    std::string name;
    std::optional<std::uint64_t> seed;  // file-declared; CLI overrides
    network::NetworkConfig network{};
    MovementSettings movement{};             // opt-in (ADR-010)
    SensingSettings sensing{};               // opt-in (ADR-011)
    std::optional<std::uint64_t> duration_ms;  // run horizon; required by movement
    bool has_station = false;
    network::EndpointId station_endpoint{};
    std::vector<ScenarioRobot> robots;  // declaration order = fan-out order
    std::vector<ScenarioEvent> events;  // sorted by (at, file order)
};

}  // namespace fleet::scenario

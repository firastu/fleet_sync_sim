#include "fleet/scenario/scenario_runner.hpp"

#include <algorithm>
#include <format>
#include <optional>
#include <stdexcept>
#include <utility>

#include "fleet/map/graph.hpp"
#include "fleet/map/map_delta.hpp"
#include "fleet/network/network_simulator.hpp"
#include "fleet/scenario/scenario.hpp"
#include "fleet/simulation/deterministic_rng.hpp"
#include "fleet/simulation/event_queue.hpp"
#include "fleet/world/observation_model.hpp"
#include "fleet/world/truth_pose.hpp"
#include "fleet/world/world.hpp"

namespace fleet::scenario {

namespace {

[[nodiscard]] std::string decision_name(map::ReconcileDecision decision) {
    switch (decision) {
        case map::ReconcileDecision::Applied:
            return "applied";
        case map::ReconcileDecision::Duplicate:
            return "duplicate";
        case map::ReconcileDecision::Stale:
            return "stale";
        case map::ReconcileDecision::Dominated:
            return "dominated";
        case map::ReconcileDecision::RejectedConflict:
            return "rejected_conflict";
    }
    return "?";
}

[[nodiscard]] std::string route_summary(const planning::Route& route,
                                        const map::Graph& graph) {
    if (!route.found) {
        return "<no route>";
    }
    std::string text;
    for (std::size_t i = 0; i < route.nodes.size(); ++i) {
        if (i > 0) {
            text += "->";
        }
        text += graph.node(route.nodes[i]).name;
    }
    return text;
}

// Trace/inspection name of a GNSS model spec (ADR-018): the scenario
// vocabulary, not a C++ type name.
[[nodiscard]] std::string gnss_model_name(const GnssModelSpec& spec) {
    switch (spec.kind) {
        case GnssModelSpec::Kind::Perfect:
            return "perfect";
        case GnssModelSpec::Kind::Unavailable:
            return "unavailable";
        case GnssModelSpec::Kind::Noisy:
            return "noisy";
    }
    return "?";
}

// Fixed domain discriminator for GNSS RNG sub-streams (ADR-018). NEVER
// change: published (scenario, seed) replay compatibility depends on it.
// Value: ASCII "GNSS_STR" big-endian.
constexpr std::uint64_t kGnssStreamDomain = 0x474E53535F535452ULL;

}  // namespace

ScenarioRunner::ScenarioRunner(const map::BaseMap& base, Scenario scenario,
                               std::uint64_t resolved_seed)
    : base_{base}, scenario_{std::move(scenario)}, resolved_seed_{resolved_seed} {
    // No zero-time self-scheduling loops (ADR-010): every advance-chain
    // event must land strictly later than its predecessor. The loader
    // enforces this for files; the constructor is the single choke point
    // that also covers programmatically constructed scenarios. Traversal
    // ticks are additionally guaranteed >= 1 inside Robot::begin_transit
    // (base costs are strictly positive by a graph invariant).
    if (scenario_.movement.enabled &&
        (scenario_.movement.ms_per_cost_unit == 0 || scenario_.movement.retry_ms == 0)) {
        throw std::invalid_argument(
            "ScenarioRunner: movement ms_per_cost_unit and retry_ms must be >= 1");
    }
    // Localization choke point (ADR-018): the same rules the loader
    // enforces for files, applied to programmatically constructed
    // scenarios. Models are constructed EAGERLY through the single
    // factory so invalid noise configuration fails here, not at the
    // first sample. Each robot gets its OWN GNSS RNG, derived from the
    // resolved seed by specified arithmetic over its stable RobotId —
    // robot-local streams, schedule-independent of one another.
    // for 0.0 : initial physical orientation at scenario start is north.
    rest_heading_rad_.assign(scenario_.robots.size(), 0.0);
    if (scenario_.localization.dead_reckoning) {
        scenario_.localization.dead_reckoning->validate();
        if (!scenario_.localization.enabled) {
            throw std::invalid_argument("ScenarioRunner: dead reckoning requires localization");
        }
    }
    if (scenario_.localization.enabled) {
        if (!scenario_.duration_ms) {
            throw std::invalid_argument("ScenarioRunner: localization requires duration_ms");
        }
        if (scenario_.localization.gnss_period_ms == 0) {
            throw std::invalid_argument(
                "ScenarioRunner: localization gnss period_ms must be >= 1");
        }
        if (base_.geometry() == nullptr) {
            throw std::invalid_argument(
                "ScenarioRunner: localization requires a map with geographic "
                "geometry (truth pose is derived from it)");
        }
        for (std::size_t index = 0; index < scenario_.robots.size(); ++index) {
            gnss_models_.push_back(make_gnss_model(scenario_.localization.initial_model));
            gnss_rngs_.push_back(std::make_unique<simulation::DeterministicRng>(
                simulation::derive_stream_seed(resolved_seed_, kGnssStreamDomain,
                                               scenario_.robots[index].id.value())));
        }
    }
}

ScenarioRunner::~ScenarioRunner() = default;

void ScenarioRunner::add_sink(TraceSink& sink) {
    sinks_.push_back(&sink);
}

const robot::Robot& ScenarioRunner::robot(std::string_view name) const {
    return *robots_.at(index_of_robot(name));
}

const station::ControlStation* ScenarioRunner::station() const noexcept {
    return station_.get();
}

std::optional<double> ScenarioRunner::localization_position_error_m(std::string_view name) const {
    const auto index = index_of_robot(name);
    const auto& participant = *robots_.at(index);
    const auto& estimate = participant.localization().estimate();
    if (!estimate) {
        return std::nullopt;
    }
    return world::localization_position_error_m(
        world::truth_pose(base_, participant.state(), rest_heading_rad_[index], queue_->clock().now()),
        *estimate);
}

void ScenarioRunner::emit(TraceEvent event) {
    for (TraceSink* sink : sinks_) {
        sink->record(event);
    }
}

std::string ScenarioRunner::robot_name_of(common::RobotId id) const {
    for (const ScenarioRobot& entry : scenario_.robots) {
        if (entry.id == id) {
            return entry.name;
        }
    }
    return std::format("robot{}", id.value());
}

std::size_t ScenarioRunner::index_of_robot(std::string_view name) const {
    for (std::size_t index = 0; index < scenario_.robots.size(); ++index) {
        if (scenario_.robots[index].name == name) {
            return index;
        }
    }
    throw std::invalid_argument(std::format("scenario runner: unknown robot '{}'", name));
}

std::size_t ScenarioRunner::index_of_robot(common::RobotId id) const {
    for (std::size_t index = 0; index < scenario_.robots.size(); ++index) {
        if (scenario_.robots[index].id == id) {
            return index;
        }
    }
    throw std::invalid_argument(
        std::format("scenario runner: unknown robot id {}", id.value()));
}

std::string ScenarioRunner::edge_label(common::EdgeId edge) const {
    const map::Edge& base_edge = base_.graph().edge(edge);
    return std::format("{}-{}", base_.graph().node(base_edge.a).name,
                       base_.graph().node(base_edge.b).name);
}

std::string ScenarioRunner::node_name(common::NodeId node) const {
    return std::string{base_.graph().node(node).name};
}

void ScenarioRunner::wire_world() {
    queue_ = std::make_unique<simulation::EventQueue>();
    network_ =
        std::make_unique<network::NetworkSimulator>(*queue_, scenario_.network, resolved_seed_);
    world_ = std::make_unique<world::World>(base_);
    if (scenario_.sensing.enabled) {
        sensor_ = std::make_unique<world::PerfectLocalEdgeSensor>();
    }
    if (scenario_.has_station) {
        station_ = std::make_unique<station::ControlStation>(base_);
    }

    // Participant order everywhere: robots in declaration order, station last.
    const std::size_t robot_count = scenario_.robots.size();
    const std::size_t participant_count = robot_count + (scenario_.has_station ? 1 : 0);

    for (std::size_t index = 0; index < robot_count; ++index) {
        const ScenarioRobot& declaration = scenario_.robots[index];
        robots_.push_back(std::make_unique<robot::Robot>(
            declaration.id, declaration.mission, base_,
            [this, index, participant_count](const map::MapDelta& delta) {
                const common::Tick now = queue_->clock().now();
                for (std::size_t target = 0; target < participant_count; ++target) {
                    if (target == index) {
                        continue;  // never send to self
                    }
                    const network::SendResult result = network_->send(
                        scenario_.robots[index].endpoint, target_endpoint(target), delta);
                    TraceEvent event;
                    event.at = now;
                    event.source = scenario_.robots[index].name;
                    event.type = "send";
                    event.fields.emplace_back("to", target_name(target));
                    if (result.link_down) {
                        event.fields.emplace_back("outcome", std::string{"link_down"});
                    } else if (result.dropped) {
                        event.fields.emplace_back("outcome", std::string{"lost"});
                    } else {
                        event.fields.emplace_back("outcome", std::string{"scheduled"});
                        event.fields.emplace_back(
                            "deliveries", static_cast<std::int64_t>(result.delivery_ticks.size()));
                        event.fields.emplace_back(
                            "first_at",
                            static_cast<std::int64_t>(result.delivery_ticks.front().value));
                    }
                    emit(std::move(event));
                }
            }));
        if (scenario_.localization.dead_reckoning) {
            robots_.back()->configure_dead_reckoning(*scenario_.localization.dead_reckoning);
        }
    }

    for (std::size_t index = 0; index < robot_count; ++index) {
        wire_delivery_handler(index);
    }
    if (scenario_.has_station) {
        network_->add_endpoint(
            scenario_.station_endpoint,
            [this](network::EndpointId from, const map::MapDelta& delta) {
                const map::ReconcileDecision decision = station_->receive(delta);
                TraceEvent event;
                event.at = queue_->clock().now();
                event.source = "station";
                event.type = "reconcile";
                event.fields.emplace_back("from", static_cast<std::int64_t>(from.value()));
                event.fields.emplace_back("decision", decision_name(decision));
                emit(std::move(event));
            });
    }

    emit(TraceEvent{
        common::Tick{0}, "world", "scenario", {{"name", scenario_.name}}});
    emit(TraceEvent{
        common::Tick{0}, "world", "seed", {{"value", static_cast<std::int64_t>(resolved_seed_)}}});
    for (const std::unique_ptr<robot::Robot>& entry : robots_) {
        emit(TraceEvent{common::Tick{0},
                        robot_name_of(entry->id()),
                        "route",
                        {{"nodes", route_summary(entry->current_route(), base_.graph())},
                         {"cost", entry->current_route().cost}}});
    }

    // Movement (ADR-010): one self-rescheduling advance chain per robot.
    // The runner owns the scheduling; the robot owns the semantics.
    if (scenario_.movement.enabled) {
        for (std::size_t index = 0; index < robots_.size(); ++index) {
            queue_->schedule(common::Tick{0}, [this, index] { advance_robot(index); });
        }
    }
}

network::EndpointId ScenarioRunner::target_endpoint(std::size_t target_index) const {
    if (target_index < scenario_.robots.size()) {
        return scenario_.robots[target_index].endpoint;
    }
    return scenario_.station_endpoint;
}

std::string ScenarioRunner::target_name(std::size_t target_index) const {
    if (target_index < scenario_.robots.size()) {
        return scenario_.robots[target_index].name;
    }
    return "station";
}

void ScenarioRunner::wire_delivery_handler(std::size_t index) {
    const std::string name = scenario_.robots[index].name;
    network_->add_endpoint(
        scenario_.robots[index].endpoint,
        [this, index, name](network::EndpointId from, const map::MapDelta& delta) {
            robot::Robot& receiver = *robots_[index];
            const planning::Route route_before = receiver.current_route();
            const map::ReconcileDecision decision = receiver.receive(delta);

            TraceEvent delivery;
            delivery.at = queue_->clock().now();
            delivery.source = name;
            delivery.type = "delivery";
            delivery.fields.emplace_back("from", static_cast<std::int64_t>(from.value()));
            emit(std::move(delivery));

            TraceEvent reconcile;
            reconcile.at = queue_->clock().now();
            reconcile.source = name;
            reconcile.type = "reconcile";
            reconcile.fields.emplace_back("decision", decision_name(decision));
            emit(std::move(reconcile));

            if (receiver.current_route() != route_before) {
                emit(TraceEvent{queue_->clock().now(),
                                name,
                                "route",
                                {{"nodes",
                                  route_summary(receiver.current_route(), base_.graph())},
                                 {"cost", receiver.current_route().cost}}});
            }
        });
}

void ScenarioRunner::schedule_events() {
    for (const ScenarioEvent& event : scenario_.events) {
        queue_->schedule(event.at, [this, event]() { apply_scenario_event(event); });
    }
}

// One scenario event's effect — shared by scheduled (loaded) events and
// interactive inject() so both paths are byte-identical.
void ScenarioRunner::apply_scenario_event(const ScenarioEvent& event) {
    if (const SetLinkAction* set_link = std::get_if<SetLinkAction>(&event.action)) {
        network_->set_link_state(set_link->from, set_link->to, set_link->up);
        emit(TraceEvent{event.at,
                        "world",
                        "link_state",
                        {{"from", static_cast<std::int64_t>(set_link->from.value())},
                         {"to", static_cast<std::int64_t>(set_link->to.value())},
                         {"up", set_link->up}}});
    } else if (const ObserveEdgeAction* observation =
                   std::get_if<ObserveEdgeAction>(&event.action)) {
        const robot::ObservationResult result =
            robots_[index_of_robot(observation->robot)]
                ->observe(observation->edge, observation->status, event.at,
                          observation->confidence);
        emit(TraceEvent{
            event.at,
            robot_name_of(observation->robot),
            "observation",
            {{"edge", edge_label(observation->edge)},
             {"status",
              std::string{observation->status == map::EdgeStatus::Blocked ? "BLOCKED"
                                                                           : "OPEN"}},
             {"decision", decision_name(result.local_decision)},
             {"replanned", result.replanned}}});
    } else if (const ResynchronizeAction* resync =
                   std::get_if<ResynchronizeAction>(&event.action)) {
        const std::size_t count = robots_[index_of_robot(resync->robot)]->resynchronize();
        emit(TraceEvent{event.at,
                        robot_name_of(resync->robot),
                        "resynchronize",
                        {{"deltas", static_cast<std::int64_t>(count)}}});
    } else if (const SetWorldEdgeStateAction* world_change =
                   std::get_if<SetWorldEdgeStateAction>(&event.action)) {
        // What actually happened (ADR-011): truth first, then every robot
        // senses in declaration order.
        world_->set_edge_state(world_change->edge, world_change->status);
        emit(TraceEvent{event.at,
                        "world",
                        "world_edge",
                        {{"edge", edge_label(world_change->edge)},
                         {"status",
                          std::string{world_change->status == map::EdgeStatus::Blocked
                                          ? "BLOCKED"
                                          : "OPEN"}}}});
        if (sensor_) {
            for (std::size_t index = 0; index < robots_.size(); ++index) {
                sense_for(index, event.at);
            }
        }
    } else if (const SetGnssModelAction* gnss_switch =
                   std::get_if<SetGnssModelAction>(&event.action)) {
        // Scenario policy (ADR-016/018): an outage is WHICH model is
        // active — replace it and nothing else. No immediate sample, no
        // randomness consumed: the next SCHEDULED sample measures with
        // the model now active. In a localization-disabled run the
        // variant cannot reach here (the loader rejects the action; the
        // runner never enqueues one), so the index access is safe.
        const std::size_t index = index_of_robot(gnss_switch->robot);
        gnss_models_[index] = make_gnss_model(gnss_switch->model);
        emit(TraceEvent{event.at,
                        robot_name_of(gnss_switch->robot),
                        "gnss_model",
                        {{"model", gnss_model_name(gnss_switch->model)}}});
    }
}

void ScenarioRunner::advance_robot(std::size_t index) {
    robot::Robot& robot = *robots_[index];
    const common::Tick now = queue_->clock().now();
    const std::string name = robot_name_of(robot.id());

    if (robot.state().in_transit.has_value()) {
        if(scenario_.localization.enabled) {
            // Orientation truth (ADR-018): physical facing OUTLIVES the
            // traversal. Record the completed traversal's final-segment
            // bearing (the truth pose at the arrival tick, fraction 1) BEFORE
            // the transit commits — at-rest truth then carries the arrival
            // orientation instead of resetting. Instantaneous re-orientation
            // at graph corners is the accepted abstraction (no steering).
            rest_heading_rad_[index] =
                world::truth_pose(base_, robot.state(), rest_heading_rad_[index], now)
                    .heading_rad;
        }
        // This event is the arrival: commit the traversal.
        const bool mission_complete = robot.complete_transit();
        if (mission_complete) {
            emit(TraceEvent{now,
                            name,
                            "mission_complete",
                            {{"goal", node_name(robot.mission().goal)}}});
            return;  // chain ends
        }
        emit(TraceEvent{now, name, "arrival", {{"node", node_name(robot.state().position)}}});
        // Sensing (ADR-011): a robot at a node sees its incident edges'
        // truth; a sensed change may replan before the next departure.
        sense_for(index, now);
    }

    const std::optional<robot::RobotTransit> transit =
        robot.begin_transit(now, scenario_.movement.ms_per_cost_unit);
    if (transit.has_value()) {
        emit(TraceEvent{now,
                        name,
                        "departure",
                        {{"edge", edge_label(transit->edge)},
                         {"from", node_name(transit->from)},
                         {"to", node_name(transit->to)},
                         {"arrival", static_cast<std::int64_t>(transit->arrival.value)}}});
        queue_->schedule(transit->arrival, [this, index] { advance_robot(index); });
        return;
    }

    if (!robot.state().mission_complete) {
        // No usable route (goal unreachable under current knowledge):
        // park and retry on the fixed cadence. Knowledge changes do not
        // wake the robot — movement advances only at chain events
        // (ADR-010).
        queue_->schedule(now + scenario_.movement.retry_ms,
                         [this, index] { advance_robot(index); });
    }
}

void ScenarioRunner::sense_for(std::size_t index, common::Tick now) {
    if (!sensor_) {
        return;
    }
    robot::Robot& robot = *robots_[index];
    const std::string name = robot_name_of(robot.id());
    for (const world::EdgeObservation& observation :
         sensor_->sense(*world_, robot.state())) {
        // Unchanged-fact suppression (ADR-011): the sensor MEASURES, the
        // coordinator decides what is new. A measurement matching the
        // robot's believed status (untracked = the base-map default,
        // open) is not forwarded — the M2 observation processor; a real
        // processor (filtering, accumulation) replaces this check later
        // without touching the sensor contract.
        const map::EdgeDynamicState* known = robot.overlay().find(observation.edge);
        const map::EdgeStatus believed =
            known == nullptr ? map::EdgeStatus::Open : known->status;
        if (believed == observation.status) {
            continue;
        }
        const robot::ObservationResult result =
            robot.observe(observation.edge, observation.status, now);
        // Sensor-generated observations carry origin=sensor so the
        // physical path stays distinguishable from scripted injection
        // (which has no origin field — ADR-011).
        emit(TraceEvent{
            now,
            name,
            "observation",
            {{"edge", edge_label(observation.edge)},
             {"status",
              std::string{observation.status == map::EdgeStatus::Blocked ? "BLOCKED" : "OPEN"}},
             {"decision", decision_name(result.local_decision)},
             {"replanned", result.replanned},
             {"origin", std::string{"sensor"}}}});
    }
}

std::unique_ptr<localization::GnssModel> ScenarioRunner::make_gnss_model(
    const GnssModelSpec& spec) {
    switch (spec.kind) {
        case GnssModelSpec::Kind::Perfect:
            return std::make_unique<localization::PerfectGnss>();
        case GnssModelSpec::Kind::Unavailable:
            return std::make_unique<localization::UnavailableGnss>();
        case GnssModelSpec::Kind::Noisy:
            // NoisyGnss's constructor validates (finite, non-negative) —
            // the one validation point for noise configuration.
            return std::make_unique<localization::NoisyGnss>(spec.noise);
    }
    throw std::invalid_argument("ScenarioRunner: unknown GNSS model kind");
}

void ScenarioRunner::sample_gnss(std::size_t index) {
    robot::Robot& robot = *robots_[index];
    const common::Tick now = queue_->clock().now();
    robot.advance_localization(now);

    // Truth on the simulation side only (ADR-016/018): derived from the
    // movement state and map geometry, never from the robot's belief.
    // At rest this carries the runner-maintained physical orientation
    // (never a stationary reset); in transit it is the live travel pose.
    const localization::GroundTruthPose truth =
        world::truth_pose(base_, robot.state(), rest_heading_rad_[index], now);

    // The measurement boundary: the ACTIVE model decides fix or no fix.
    // An outage is UnavailableGnss sitting in gnss_models_[index] — no
    // special case here.
    const std::optional<localization::LocalizationEstimate> fix =
        gnss_models_[index]->measure(truth, *gnss_rngs_[index]);

    // Retention is robot-local (#16): a fix replaces the retained
    // estimate, a no-fix retains it — the sample outcome is all the
    // robot ever learns. The returned transition (ADR-021) is what the
    // reacquisition trace fields below observe.
    const localization::FixApplication applied = robot.apply_gnss_sample(fix);

    // The trace event carries BELIEF only: outcome plus the retained
    // estimate and its age. Truth coordinates are deliberately absent —
    // truth stays visible through movement events (departure/arrival),
    // never inside a robot-local localization event. Position and
    // heading are 6-decimal strings: the shared double formatter's two
    // decimals cannot separate neighboring map positions.
    TraceEvent event{now, robot_name_of(robot.id()), "gnss_sample",
                     {{"outcome", std::string{fix.has_value() ? "fix" : "no_fix"}}}};
    if (const std::optional<localization::LocalizationEstimate>& estimate =
            robot.localization().estimate();
        estimate.has_value()) {
        event.fields.emplace_back("estimated_at",
                                  static_cast<std::int64_t>(estimate->estimated_at.value));
        event.fields.emplace_back(
            "age", static_cast<std::int64_t>(robot.localization().age_at(now).value()));
        event.fields.emplace_back(
            "position", std::format("{:.6f},{:.6f}", estimate->position.latitude_deg,
                                    estimate->position.longitude_deg));
        event.fields.emplace_back("heading", std::format("{:.6f}", estimate->heading_rad));
        if (scenario_.localization.dead_reckoning) {
            event.fields.emplace_back("estimate_source",
                std::string{robot.localization().dead_reckoned() ? "dead_reckoning" : "gnss"});
            event.fields.emplace_back("last_fix_at",
                static_cast<std::int64_t>(robot.localization().last_fix_at()->value));
            event.fields.emplace_back("last_fix_age",
                static_cast<std::int64_t>(now.value - robot.localization().last_fix_at()->value));
        }
    }
    // Reacquisition observability (#18, ADR-021): these transition fields
    // exist ONLY on the sample where a fix returns after at least one miss
    // — every other gnss_sample event, in every existing fixture, is
    // byte-unchanged.
    if (applied.fix && applied.reacquisition) {
        event.fields.emplace_back("reacquired", true);
        event.fields.emplace_back("prior_source",
            std::string{applied.prior_dead_reckoned ? "dead_reckoning" : "gnss"});
        if (applied.since_last_fix_ms.has_value()) {
            event.fields.emplace_back(
                "since_last_fix_ms", static_cast<std::int64_t>(*applied.since_last_fix_ms));
        }
        if (applied.correction_distance_m.has_value()) {
            event.fields.emplace_back(
                "correction_m", std::format("{:.6f}", *applied.correction_distance_m));
        }
        if (applied.correction_heading_rad.has_value()) {
            event.fields.emplace_back("correction_heading_rad",
                std::format("{:.6f}", *applied.correction_heading_rad));
        }
    }
    emit(std::move(event));

    // Strictly later reschedule (ADR-005/010/018): period_ms >= 1 is
    // enforced at the constructor choke point — no zero-time loop.
    if (scenario_.localization.dead_reckoning && robot.localization().estimate()) {
        emit(TraceEvent{now, "world", "localization_error",
            {{"robot", robot_name_of(robot.id())},
             {"position_error_m", world::localization_position_error_m(
                 truth, *robot.localization().estimate())}}});
    }
    queue_->schedule(now + scenario_.localization.gnss_period_ms,
                     [this, index] { sample_gnss(index); });
}

void ScenarioRunner::start_localization_chains() {
    // Enqueued AFTER wire_world()'s movement chains and AFTER
    // schedule_events()'s scripted events. Loaded model switches precede
    // same-tick samples, including tick 0; other event classes retain
    // enqueue order (ADR-018).
    for (std::size_t index = 0; index < robots_.size(); ++index) {
        queue_->schedule(common::Tick{0}, [this, index] { sample_gnss(index); });
    }
}

void ScenarioRunner::begin() {
    if (begun_) {
        return;
    }
    begun_ = true;
    wire_world();
    schedule_events();
    if (scenario_.localization.enabled) {
        start_localization_chains();
    }
}

ScenarioRunner::Result ScenarioRunner::run_until(common::Tick until) {
    begin();
    queue_->run_until(until);  // throws std::invalid_argument if until < now
    return Result{resolved_seed_, queue_->clock().now(), robots_.size(),
                  scenario_.has_station};
}

common::Tick ScenarioRunner::now() {
    begin();
    return queue_->clock().now();
}

void ScenarioRunner::inject(const ScenarioEvent& event) {
    if (!begun_) {
        throw std::logic_error("ScenarioRunner::inject: call begin() first");
    }
    if (event.at < queue_->clock().now()) {
        throw std::invalid_argument(
            "ScenarioRunner::inject: event tick is in the past");
    }
    queue_->schedule(event.at, [this, event]() { apply_scenario_event(event); });
}

ScenarioRunner::Result ScenarioRunner::run_to_completion() {
    begin();
    if (scenario_.duration_ms.has_value()) {
        // Bounded horizon (ADR-010): events beyond the horizon never run;
        // the clock lands exactly on the horizon. Interactive stepping
        // may have already passed it (the operator extended the run) —
        // then nothing more runs and the snapshot is returned; draining
        // instead would chase unbounded park-and-retry chains.
        const common::Tick horizon{*scenario_.duration_ms};
        if (horizon >= queue_->clock().now()) {
            queue_->run_until(horizon);
        }
    } else {
        queue_->run_to_completion();
    }
    return Result{resolved_seed_, queue_->clock().now(), robots_.size(), scenario_.has_station};
}

}  // namespace fleet::scenario

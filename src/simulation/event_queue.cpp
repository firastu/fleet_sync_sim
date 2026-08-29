#include "fleet/simulation/event_queue.hpp"

#include <stdexcept>
#include <utility>

namespace fleet::simulation {

bool EventQueue::Later::operator()(const Scheduled& a, const Scheduled& b) const noexcept {
    if (a.at != b.at) {
        return a.at > b.at;
    }
    return a.order > b.order;
}

void EventQueue::schedule(common::Tick at, std::function<void()> effect) {
    if (at < clock_.now()) {
        throw std::invalid_argument("EventQueue::schedule: event tick is in the past");
    }
    queue_.push(Scheduled{at, next_order_++, std::move(effect)});
}

bool EventQueue::step() {
    if (queue_.empty()) {
        return false;
    }
    const Scheduled next = queue_.top();
    queue_.pop();
    clock_.advance_to(next.at);
    next.effect();  // may schedule further events, including at next.at
    return true;
}

void EventQueue::run_until(common::Tick until) {
    if (until < clock_.now()) {
        throw std::invalid_argument("EventQueue::run_until: horizon is in the past");
    }
    while (!queue_.empty() && queue_.top().at <= until) {
        step();
    }
    // Horizon semantics: advance to exactly `until`, even across idle
    // time, so the postcondition now() == until holds and subsequent
    // relative scheduling cannot land before the requested horizon.
    clock_.advance_to(until);
}

void EventQueue::run_to_completion() {
    while (step()) {
    }
}

}  // namespace fleet::simulation

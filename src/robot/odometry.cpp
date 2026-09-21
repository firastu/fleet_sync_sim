#include "fleet/robot/robot.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fleet::robot {

void Robot::configure_dead_reckoning(localization::DeadReckoningConfig config) {
    config.validate();
    if (dead_reckoning_ || state_.in_transit || state_.position != mission_.start ||
        localization_.estimate()) {
        throw std::invalid_argument("dead reckoning: configure before motion or the first fix");
    }
    if (base_.geometry() == nullptr) {
        throw std::invalid_argument("dead reckoning: local map geometry is required");
    }
    dead_reckoning_ = config;
}

void Robot::prepare_odometry(const RobotTransit& transit) {
    const auto& geometry = *base_.geometry();
    const auto& edge = base_.graph().edge(transit.edge);
    std::vector<map::Wgs84Coordinate> points;
    if (const auto* polyline = geometry.edge_polyline(transit.edge)) {
        points = *polyline;
    } else {
        const auto* first = geometry.node_position(edge.a);
        const auto* last = geometry.node_position(edge.b);
        if (first == nullptr || last == nullptr) {
            throw std::invalid_argument("dead reckoning: missing edge endpoint geometry");
        }
        points = {*first, *last};
    }
    if (transit.from == edge.b) {
        std::reverse(points.begin(), points.end());
    }
    std::vector<MotionSegment> segments;
    double length = 0.0;
    constexpr double radians_per_degree = std::numbers::pi / 180.0;
    for (std::size_t index = 1; index < points.size(); ++index) {
        const double latitude_from = points[index - 1].latitude_deg * radians_per_degree;
        const double latitude_to = points[index].latitude_deg * radians_per_degree;
        const double latitude_delta = latitude_to - latitude_from;
        const double longitude_delta =
            (points[index].longitude_deg - points[index - 1].longitude_deg) * radians_per_degree;
        const double haversine = std::pow(std::sin(latitude_delta / 2.0), 2) +
            std::cos(latitude_from) * std::cos(latitude_to) *
                std::pow(std::sin(longitude_delta / 2.0), 2);
        const double distance = 2.0 * 6371008.8 * std::asin(std::min(1.0, std::sqrt(haversine)));
        if (distance == 0.0) {
            continue;
        }
        const double heading = localization::normalize_heading(std::atan2(
            std::sin(longitude_delta) * std::cos(latitude_to),
            std::cos(latitude_from) * std::sin(latitude_to) -
                std::sin(latitude_from) * std::cos(latitude_to) * std::cos(longitude_delta)));
        segments.push_back({distance, heading});
        length += distance;
    }
    motion_segments_ = std::move(segments);
    motion_length_m_ = length;
}

void Robot::advance_localization(common::Tick now) {
    if (!dead_reckoning_) {
        return;
    }
    if (now < odometry_at_ || (state_.in_transit && now > state_.in_transit->arrival)) {
        throw std::invalid_argument("dead reckoning: advance must respect movement transitions");
    }
    auto next = localization_;
    double heading = motion_heading_rad_;
    const auto propagate = [&](double distance, double turn) {
        const auto from = next.estimate() ? next.estimate()->estimated_at : odometry_at_;
        next.propagate({distance, turn, from, now}, *dead_reckoning_);
    };
    if (state_.in_transit && motion_length_m_ > 0.0) {
        const auto& transit = *state_.in_transit;
        const double duration = static_cast<double>(transit.arrival.value - transit.departed_at.value);
        const double previous = motion_length_m_ *
            (static_cast<double>(odometry_at_.value - transit.departed_at.value) / duration);
        const double target = motion_length_m_ *
            (static_cast<double>(now.value - transit.departed_at.value) / duration);
        double start = 0.0;
        for (const auto& segment : motion_segments_) {
            const double end = start + segment.length_m;
            if (target >= start && previous < end) {
                const double distance = std::max(0.0, std::min(target, end) - std::max(previous, start));
                propagate(distance, segment.heading_rad - heading);
                heading = segment.heading_rad;
            }
            start = end;
        }
    }
    propagate(0.0, 0.0);
    localization_ = next;
    motion_heading_rad_ = heading;
    odometry_at_ = now;
}

}  // namespace fleet::robot
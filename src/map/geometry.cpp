#include "fleet/map/geometry.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace fleet::map {

namespace {

[[nodiscard]] bool is_valid_coordinate(const Wgs84Coordinate& position) {
    return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
           position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0 &&
           position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0;
}

}  // namespace

MapGeometry::Builder::Builder(std::size_t node_count, std::size_t edge_count,
                              CoordinateReferenceSystem crs)
    : node_count_{node_count},
      edge_count_{edge_count},
      crs_{crs},
      node_positions_(node_count),
      edge_polylines_(edge_count) {}

void MapGeometry::Builder::require_unbuilt() const {
    if (built_) {
        throw std::logic_error("MapGeometry::Builder: build() already called");
    }
}

void MapGeometry::Builder::set_node_position(common::NodeId node, Wgs84Coordinate position) {
    require_unbuilt();
    if (node.value() >= node_count_) {
        throw std::invalid_argument("MapGeometry::Builder: unknown NodeId");
    }
    if (!is_valid_coordinate(position)) {
        throw std::invalid_argument(
            "MapGeometry::Builder: coordinate must be finite and within WGS84 bounds");
    }
    node_positions_[node.value()] = position;
}

void MapGeometry::Builder::set_edge_polyline(common::EdgeId edge,
                                             std::vector<Wgs84Coordinate> points) {
    require_unbuilt();
    if (edge.value() >= edge_count_) {
        throw std::invalid_argument("MapGeometry::Builder: unknown EdgeId");
    }
    if (points.size() < 2) {
        throw std::invalid_argument(
            "MapGeometry::Builder: edge polyline needs at least two points");
    }
    for (const Wgs84Coordinate& point : points) {
        if (!is_valid_coordinate(point)) {
            throw std::invalid_argument(
                "MapGeometry::Builder: polyline coordinate must be finite and within "
                "WGS84 bounds");
        }
    }
    edge_polylines_[edge.value()] = std::move(points);
}

MapGeometry MapGeometry::Builder::build() {
    require_unbuilt();
    built_ = true;
    return MapGeometry{node_count_, edge_count_, crs_, std::move(node_positions_),
                       std::move(edge_polylines_)};
}

MapGeometry::MapGeometry(
    std::size_t node_count, std::size_t edge_count, CoordinateReferenceSystem crs,
    std::vector<std::optional<Wgs84Coordinate>> node_positions,
    std::vector<std::optional<std::vector<Wgs84Coordinate>>> edge_polylines) noexcept
    : node_count_{node_count},
      edge_count_{edge_count},
      crs_{crs},
      node_positions_{std::move(node_positions)},
      edge_polylines_{std::move(edge_polylines)} {}

const Wgs84Coordinate* MapGeometry::node_position(common::NodeId node) const noexcept {
    if (node.value() >= node_count_) {
        return nullptr;
    }
    const std::optional<Wgs84Coordinate>& entry = node_positions_[node.value()];
    return entry.has_value() ? &*entry : nullptr;
}

const std::vector<Wgs84Coordinate>* MapGeometry::edge_polyline(
    common::EdgeId edge) const noexcept {
    if (edge.value() >= edge_count_) {
        return nullptr;
    }
    const std::optional<std::vector<Wgs84Coordinate>>& entry = edge_polylines_[edge.value()];
    return entry.has_value() ? &*entry : nullptr;
}

}  // namespace fleet::map

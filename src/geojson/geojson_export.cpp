#include "fleet/geojson/geojson_export.hpp"

#include <format>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::geojson {

namespace {

// Minimal JSON string escaping (same policy as the trace writer).
void write_json_string(std::ostream& out, std::string_view text) {
    out << '"';
    for (const char character : text) {
        switch (character) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(character) < 0x20) {
                    static constexpr char kHex[] = "0123456789abcdef";
                    out << "\\u00" << kHex[(character >> 4) & 0xF] << kHex[character & 0xF];
                } else {
                    out << character;
                }
        }
    }
    out << '"';
}

[[nodiscard]] std::string_view direction_name(map::EdgeDirection direction) {
    switch (direction) {
        case map::EdgeDirection::Bidirectional:
            return "bidirectional";
        case map::EdgeDirection::Forward:
            return "forward";
        case map::EdgeDirection::Reverse:
            return "reverse";
    }
    return "?";
}

}  // namespace

std::string position_text(const map::Wgs84Coordinate& coordinate) {
    // THE coordinate-order boundary: GeoJSON positions are
    // [longitude, latitude] — the opposite field order of
    // Wgs84Coordinate. This is the single conversion point.
    return std::format("[{:.7f},{:.7f}]", coordinate.longitude_deg, coordinate.latitude_deg);
}

std::size_t write_base_map_geojson(const map::BaseMap& map, std::ostream& output) {
    const map::MapGeometry* geometry = map.geometry();
    if (geometry == nullptr) {
        throw std::invalid_argument(
            "geojson export: the map carries no geographic side (MapGeometry)");
    }
    const map::Graph& graph = map.graph();

    std::size_t features = 0;
    output << "{\"type\":\"FeatureCollection\",\"features\":[";
    bool first = true;

    // Node features, ascending NodeId.
    for (const map::Node& node : graph.nodes()) {
        const map::Wgs84Coordinate* position = geometry->node_position(node.id);
        if (position == nullptr) {
            continue;
        }
        if (!first) {
            output << ',';
        }
        first = false;
        output << "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":"
               << position_text(*position) << "},\"properties\":{\"name\":";
        write_json_string(output, node.name);
        output << "}}";
        ++features;
    }

    // Edge features, ascending EdgeId. Polyline when present, otherwise
    // the straight segment between endpoint coordinates (fallback).
    for (const map::Edge& edge : graph.edges()) {
        const map::Wgs84Coordinate* from = geometry->node_position(edge.a);
        const map::Wgs84Coordinate* to = geometry->node_position(edge.b);
        if (from == nullptr || to == nullptr) {
            continue;
        }
        if (!first) {
            output << ',';
        }
        first = false;
        output << "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\","
                  "\"coordinates\":[";
        const std::vector<map::Wgs84Coordinate>* polyline = geometry->edge_polyline(edge.id);
        if (polyline != nullptr) {
            for (std::size_t i = 0; i < polyline->size(); ++i) {
                if (i > 0) {
                    output << ',';
                }
                output << position_text((*polyline)[i]);
            }
        } else {
            output << position_text(*from) << ',' << position_text(*to);
        }
        output << "]},\"properties\":{\"edge\":" << edge.id.value() << ",\"from\":";
        write_json_string(output, graph.node(edge.a).name);
        output << ",\"to\":";
        write_json_string(output, graph.node(edge.b).name);
        output << ",\"direction\":";
        write_json_string(output, direction_name(edge.direction));
        output << ",\"cost\":" << std::format("{:.2f}", edge.base_cost) << "}}";
        ++features;
    }

    output << "]}\n";
    return features;
}

std::size_t write_trace_geojson(const map::BaseMap& map,
                                std::span<const scenario::TraceEvent> events,
                                std::ostream& output) {
    const map::MapGeometry* geometry = map.geometry();
    if (geometry == nullptr) {
        throw std::invalid_argument(
            "geojson export: the map carries no geographic side (MapGeometry)");
    }
    const map::Graph& graph = map.graph();

    // Name -> coordinate (deterministic; graph names are unique).
    std::map<std::string, map::Wgs84Coordinate> coordinates_by_name;
    for (const map::Node& node : graph.nodes()) {
        if (const map::Wgs84Coordinate* position = geometry->node_position(node.id);
            position != nullptr) {
            coordinates_by_name.emplace(node.name, *position);
        }
    }
    const auto coordinate_of = [&](const std::string& name,
                                   const scenario::TraceEvent& event) {
        const auto found = coordinates_by_name.find(name);
        if (found == coordinates_by_name.end()) {
            throw std::invalid_argument(std::format(
                "geojson export: trace event at tick {} references unknown or "
                "coordinate-less node '{}'",
                event.at.value, name));
        }
        return found->second;
    };

    // Per-robot trajectory, robots in first-appearance order.
    struct Trajectory {
        std::vector<map::Wgs84Coordinate> points;
        bool completed = false;
        map::Wgs84Coordinate goal{};
    };
    std::vector<std::pair<std::string, Trajectory>> robots;  // stable order
    const auto robot_slot = [&](const std::string& name) -> Trajectory& {
        for (auto& entry : robots) {
            if (entry.first == name) {
                return entry.second;
            }
        }
        robots.emplace_back(name, Trajectory{});
        return robots.back().second;
    };

    for (const scenario::TraceEvent& event : events) {
        if (event.type == "departure") {
            std::string from;
            std::string to;
            for (const auto& [key, value] : event.fields) {
                if (key == "from") {
                    from = std::get<std::string>(value);
                } else if (key == "to") {
                    to = std::get<std::string>(value);
                }
            }
            Trajectory& trajectory = robot_slot(event.source);
            const map::Wgs84Coordinate from_position = coordinate_of(from, event);
            if (trajectory.points.empty() || trajectory.points.back() != from_position) {
                trajectory.points.push_back(from_position);
            }
            trajectory.points.push_back(coordinate_of(to, event));
        } else if (event.type == "mission_complete") {
            std::string goal;
            for (const auto& [key, value] : event.fields) {
                if (key == "goal") {
                    goal = std::get<std::string>(value);
                }
            }
            Trajectory& trajectory = robot_slot(event.source);
            trajectory.completed = true;
            trajectory.goal = coordinate_of(goal, event);
        }
    }

    std::size_t features = 0;
    output << "{\"type\":\"FeatureCollection\",\"features\":[";
    bool first = true;
    for (const auto& [name, trajectory] : robots) {
        if (trajectory.points.size() >= 2) {
            if (!first) {
                output << ',';
            }
            first = false;
            output << "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\","
                      "\"coordinates\":[";
            for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
                if (i > 0) {
                    output << ',';
                }
                output << position_text(trajectory.points[i]);
            }
            output << "]},\"properties\":{\"robot\":";
            write_json_string(output, name);
            output << "}}";
            ++features;
        }
        if (trajectory.completed) {
            if (!first) {
                output << ',';
            }
            first = false;
            output << "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\","
                      "\"coordinates\":"
                   << position_text(trajectory.goal) << "},\"properties\":{\"robot\":";
            write_json_string(output, name);
            output << ",\"mission_complete\":true}}";
            ++features;
        }
    }
    output << "]}\n";
    return features;
}

}  // namespace fleet::geojson

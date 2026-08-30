#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "fleet/common/ids.hpp"

namespace fleet::map {

// The coordinate reference system of a MapGeometry's coordinates.
// Deliberately not a GIS framework: exactly one system exists in M2, and
// making it explicit prevents silent mixing when later imports add
// projected/local frames (ADR-012).
enum class CoordinateReferenceSystem : std::uint8_t {
    Wgs84,  // EPSG:4326, degrees
};

// One geographic position in WGS84 (EPSG:4326). Named fields — never a
// generic pair of doubles: {52.3, 9.7} must never be ambiguous between
// (lat, lon) and (x, y).
struct Wgs84Coordinate {
    double latitude_deg = 0.0;    // [-90, 90]
    double longitude_deg = 0.0;   // [-180, 180]

    constexpr auto operator<=>(const Wgs84Coordinate&) const noexcept = default;
};

// The geographic side of a BaseMap (ADR-012): node coordinates and edge
// polylines as IMMUTABLE side data, parallel to the topology by
// NodeId/EdgeId. Planning (A*, overlays, reconciliation, sensing,
// movement) never reads geometry — only output adapters (GeoJSON) and
// later PNT/map-matching work do.
//
// Invariants (validated at construction / BaseMap attachment):
//   - every coordinate is finite and within WGS84 bounds;
//   - a polyline, when present, has at least two points and follows its
//     graph edge's orientation from -> to (checked when both endpoint
//     node coordinates exist; BaseMap ctor);
//   - presence is per node / per edge: geometry-free entries are legal
//     (straight-line rendering is the documented fallback).
//
// Construction happens only through the Builder; after build() the
// geometry is immutable (like Graph, ADR-001).
//
// Thread-safety: immutable after construction; safe to share.
class MapGeometry {
public:
    class Builder {
    public:
        // Sizes must match the Graph this geometry will attach to.
        Builder(std::size_t node_count, std::size_t edge_count,
                CoordinateReferenceSystem crs = CoordinateReferenceSystem::Wgs84);

        Builder(const Builder&) = delete;
        Builder& operator=(const Builder&) = delete;

        // Throws std::invalid_argument for an unknown NodeId or a
        // non-finite / out-of-bounds coordinate.
        void set_node_position(common::NodeId node, Wgs84Coordinate position);

        // Throws std::invalid_argument for an unknown EdgeId or a
        // polyline with fewer than two valid points. Orientation
        // (front == the edge's `from` node, back == its `to` node) is a
        // BaseMap-level contract, checked when the geometry attaches.
        void set_edge_polyline(common::EdgeId edge, std::vector<Wgs84Coordinate> points);

        [[nodiscard]] MapGeometry build();

    private:
        void require_unbuilt() const;

        bool built_ = false;
        std::size_t node_count_;
        std::size_t edge_count_;
        CoordinateReferenceSystem crs_;
        std::vector<std::optional<Wgs84Coordinate>> node_positions_;
        std::vector<std::optional<std::vector<Wgs84Coordinate>>> edge_polylines_;
    };

    [[nodiscard]] CoordinateReferenceSystem crs() const noexcept { return crs_; }
    [[nodiscard]] std::size_t node_count() const noexcept { return node_count_; }
    [[nodiscard]] std::size_t edge_count() const noexcept { return edge_count_; }

    // nullptr when the node has no coordinate.
    [[nodiscard]] const Wgs84Coordinate* node_position(common::NodeId node) const noexcept;

    // nullptr when the edge has no polyline. Points run from the edge's
    // `from` node to its `to` node (BaseMap-validated when endpoint
    // coordinates exist).
    //
    // Identity contract for importers: endpoint validation uses EXACT
    // coordinate equality — the polyline is an attachment to the node's
    // canonical coordinate, not a geometric proximity test. Builders of
    // geometry must REUSE the stored node coordinate values (copy them),
    // never recompute equivalent coordinates independently.
    [[nodiscard]] const std::vector<Wgs84Coordinate>* edge_polyline(
        common::EdgeId edge) const noexcept;

private:
    friend class Builder;

    MapGeometry(std::size_t node_count, std::size_t edge_count,
                CoordinateReferenceSystem crs,
                std::vector<std::optional<Wgs84Coordinate>> node_positions,
                std::vector<std::optional<std::vector<Wgs84Coordinate>>> edge_polylines) noexcept;

    std::size_t node_count_ = 0;
    std::size_t edge_count_ = 0;
    CoordinateReferenceSystem crs_ = CoordinateReferenceSystem::Wgs84;
    std::vector<std::optional<Wgs84Coordinate>> node_positions_;
    std::vector<std::optional<std::vector<Wgs84Coordinate>>> edge_polylines_;
};

}  // namespace fleet::map

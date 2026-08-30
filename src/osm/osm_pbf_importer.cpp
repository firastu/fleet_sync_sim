#include "fleet/osm/osm_pbf_importer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <map>
#include <numbers>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>  // registers the PBF reader (include-time link)
#include <osmium/io/reader.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

#include "fleet/map/geometry.hpp"
#include "fleet/map/graph.hpp"

namespace fleet::osm {

namespace {

// Documented way-eligibility whitelist (ADR-014): the PBF decoding layer
// is policy-free; this private predicate is the entire road policy.
[[nodiscard]] bool eligible_highway(const std::string& value) {
    static const std::unordered_set<std::string> kWhitelist{
        "motorway", "trunk",     "primary",     "secondary", "tertiary",
        "unclassified", "residential", "service", "living_street", "road",
        "track",
    };
    return kWhitelist.count(value) != 0;
}

// oneway tag -> traversal permissions. Exotic values fail loudly.
struct WayDirectionPolicy {
    bool forward = true;  // travel in way node order
    bool reverse = true;  // travel against way node order
};

[[nodiscard]] WayDirectionPolicy parse_oneway(const osmium::Way& way) {
    const char* oneway = way.tags()["oneway"];
    const std::string_view value{oneway == nullptr ? "" : oneway};
    if (value.empty()) {
        // junction=roundabout carries an IMPLICIT one-way direction in
        // OSM. Importing it as bidirectional would confidently create
        // the wrong road topology, so the absent-oneway roundabout case
        // fails loudly instead (ADR-014). An explicit oneway tag (any
        // supported value, including a deliberate oneway=no) goes
        // through the normal parser below.
        const char* junction = way.tags()["junction"];
        if (junction != nullptr && std::string_view{junction} == "roundabout") {
            throw std::invalid_argument(std::format(
                "osm import: way {}: junction=roundabout without an explicit oneway "
                "tag (implicit direction unsupported; tag oneway explicitly or "
                "preprocess)",
                way.id()));
        }
        return {true, true};
    }
    if (value == "no" || value == "false" || value == "0") {
        return {true, true};
    }
    if (value == "yes" || value == "true" || value == "1") {
        return {true, false};
    }
    if (value == "-1") {
        return {false, true};
    }
    throw std::invalid_argument(std::format(
        "osm import: way {}: unsupported oneway value '{}' (supported: yes/no/true/"
        "false/1/0/-1)",
        way.id(), value));
}

// Geodesic length of a polyline: haversine over consecutive points with
// the mean earth radius in meters (documented approximation, ADR-014).
// The graph receives the result as a plain positive cost — it never
// knows what a meter is.
constexpr double kEarthRadiusMeters = 6371008.8;

[[nodiscard]] double haversine_meters(const map::Wgs84Coordinate& a,
                                      const map::Wgs84Coordinate& b) {
    const double lat1 = a.latitude_deg * std::numbers::pi / 180.0;
    const double lat2 = b.latitude_deg * std::numbers::pi / 180.0;
    const double dlat = (b.latitude_deg - a.latitude_deg) * std::numbers::pi / 180.0;
    const double dlon = (b.longitude_deg - a.longitude_deg) * std::numbers::pi / 180.0;
    const double h = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) *
                         std::sin(dlon / 2.0);
    return 2.0 * kEarthRadiusMeters * std::asin(std::sqrt(h));
}

[[nodiscard]] double polyline_length_meters(
    const std::vector<map::Wgs84Coordinate>& points) {
    double total = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        total += haversine_meters(points[i - 1], points[i]);
    }
    return total;
}

struct EligibleWay {
    std::int64_t way_id = 0;
    std::vector<std::int64_t> node_refs;  // OSM node ids in way order
    WayDirectionPolicy direction{};
};

// Pass 1 handler: ways only (decoding + eligibility policy).
class WayCollector final : public osmium::handler::Handler {
public:
    void way(const osmium::Way& way) {
        stats_.ways_seen++;
        const char* highway = way.tags()["highway"];
        if (highway == nullptr || !eligible_highway(highway)) {
            return;
        }
        if (way.nodes().size() < 2) {
            return;
        }
        EligibleWay eligible;
        eligible.way_id = way.id();
        eligible.direction = parse_oneway(way);
        eligible.node_refs.reserve(way.nodes().size());
        for (const osmium::NodeRef& ref : way.nodes()) {
            eligible.node_refs.push_back(ref.ref());
        }
        eligible_.push_back(std::move(eligible));
    }

    std::vector<EligibleWay> eligible_;
    OsmImportStats stats_;
};

// Pass 2 handler: nodes referenced by eligible ways. Seen ids are
// erased from `wanted`; after the pass it holds exactly the missing ids.
class NodeCollector final : public osmium::handler::Handler {
public:
    explicit NodeCollector(std::unordered_set<std::int64_t> ids)
        : wanted{std::move(ids)} {}

    void node(const osmium::Node& node) {
        if (wanted.erase(node.id()) > 0) {
            if (!node.location().valid()) {
                throw std::invalid_argument(
                    std::format("osm import: node {} has no usable location", node.id()));
            }
            coordinates[node.id()] =
                map::Wgs84Coordinate{node.location().lat(), node.location().lon()};
        }
    }

    std::unordered_set<std::int64_t> wanted;  // ids still unseen after the pass
    std::unordered_map<std::int64_t, map::Wgs84Coordinate> coordinates;
};

}  // namespace

OsmPbfImporter::OsmPbfImporter(OsmImportOptions options) : options_{options} {}

OsmImportResult OsmPbfImporter::import(const std::filesystem::path& pbf_file) const {
    // ---- pass 1: decode ways, apply the eligibility policy ----------------
    WayCollector ways;
    {
        osmium::io::Reader reader{pbf_file, osmium::osm_entity_bits::way};
        osmium::apply(reader, ways);
        reader.close();
    }
    ways.stats_.ways_imported = ways.eligible_.size();

    // ---- pass 2: coordinates for every node referenced by eligible ways ---
    std::unordered_set<std::int64_t> wanted;
    for (const EligibleWay& way : ways.eligible_) {
        for (const std::int64_t ref : way.node_refs) {
            wanted.insert(ref);
        }
    }
    NodeCollector nodes{std::move(wanted)};
    {
        osmium::io::Reader reader{pbf_file, osmium::osm_entity_bits::node};
        osmium::apply(reader, nodes);
        reader.close();
    }
    for (const std::int64_t missing : nodes.wanted) {
        throw std::invalid_argument(std::format(
            "osm import: node {} referenced by an imported way is missing or has no "
            "location",
            missing));
    }
    const auto& coordinates = nodes.coordinates;

    // ---- retained topology nodes (ordered maps => deterministic ids) -------
    // Retained: way endpoints, and nodes referenced >= 2 times across
    // eligible way refs (a way visiting a node twice is a self-intersection
    // and a decision point too).
    std::map<std::int64_t, std::size_t> ref_counts;
    for (const EligibleWay& way : ways.eligible_) {
        for (const std::int64_t ref : way.node_refs) {
            ref_counts[ref]++;
        }
    }
    std::map<std::int64_t, common::NodeId> node_ids;  // ascending OSM id -> dense id
    for (const EligibleWay& way : ways.eligible_) {
        node_ids[way.node_refs.front()];
        node_ids[way.node_refs.back()];
    }
    for (const auto& [osm_id, count] : ref_counts) {
        if (count >= 2) {
            node_ids[osm_id];
        }
    }
    std::uint32_t next_node_id = 0;
    for (auto& entry : node_ids) {
        entry.second = common::NodeId{next_node_id++};
    }

    // ---- split ways into segments between consecutive retained nodes ------
    struct Segment {
        std::int64_t way_id = 0;
        std::size_t segment_index = 0;
        common::NodeId from{};
        common::NodeId to{};
        std::vector<map::Wgs84Coordinate> points;  // way order, canonical values
        WayDirectionPolicy direction{};
    };
    std::vector<Segment> segments;
    for (const EligibleWay& way : ways.eligible_) {
        const std::vector<std::int64_t>& refs = way.node_refs;
        std::size_t segment_index = 0;
        std::size_t start = 0;
        for (std::size_t i = 1; i < refs.size(); ++i) {
            if (node_ids.count(refs[i]) == 0) {
                continue;  // intermediate node: geometry only
            }
            Segment segment;
            segment.way_id = way.way_id;
            segment.segment_index = segment_index++;
            segment.from = node_ids.at(refs[start]);
            segment.to = node_ids.at(refs[i]);
            segment.direction = way.direction;
            for (std::size_t p = start; p <= i; ++p) {
                segment.points.push_back(coordinates.at(refs[p]));
            }
            segments.push_back(std::move(segment));
            start = i;
        }
    }

    // ---- deterministic edge order: canonical (way, segment, direction) key
    // (encounter order is irrelevant; the sort makes EdgeId assignment a
    // pure function of the data + policy)
    std::sort(segments.begin(), segments.end(), [](const Segment& a, const Segment& b) {
        if (a.way_id != b.way_id) {
            return a.way_id < b.way_id;
        }
        return a.segment_index < b.segment_index;
    });

    // ---- topology + geometry construction ----------------------------------
    // Node names are the decimal OSM node ids (deterministic, traceable
    // back to the source data). NodePosition carries an inert planning
    // frame (lon/lat); costs are explicit haversine meters and MapGeometry
    // stays the authoritative geographic side (ADR-012/014).
    map::Graph::Builder graph_builder;
    for (const auto& [osm_id, id] : node_ids) {
        const map::Wgs84Coordinate& coordinate = coordinates.at(osm_id);
        graph_builder.add_node(std::to_string(osm_id),
                               map::NodePosition{coordinate.longitude_deg,
                                                 coordinate.latitude_deg});
    }

    // One physical edge per unordered retained-node pair (ADR-013): ANY
    // second segment mapping to an already-used pair — same direction,
    // opposite direction, or a revisit within one way — fails loudly
    // with way AND segment provenance for both offenders. Never merged,
    // never dropped: the segments may differ in geometry, way identity,
    // road class, distance and future metadata.
    struct UsedPair {
        std::int64_t way_id = 0;
        std::size_t segment_index = 0;
    };
    std::map<std::pair<common::NodeId, common::NodeId>, UsedPair> connected_by;
    std::vector<std::pair<common::EdgeId, std::vector<map::Wgs84Coordinate>>> polylines;
    for (const Segment& segment : segments) {
        if (segment.from == segment.to) {
            throw std::invalid_argument(std::format(
                "osm import: way {} segment {} starts and ends at the same node (an "
                "unsplit closed way cannot be represented)",
                segment.way_id, segment.segment_index));
        }
        const double cost = polyline_length_meters(segment.points);
        if (!std::isfinite(cost) || cost <= 0.0) {
            throw std::invalid_argument(std::format(
                "osm import: way {} segment {} has a non-positive computed length",
                segment.way_id, segment.segment_index));
        }
        const auto pair_key = std::minmax(segment.from, segment.to);
        if (const auto existing = connected_by.find(pair_key);
            existing != connected_by.end()) {
            throw std::invalid_argument(std::format(
                "osm import: way {} segment {} and way {} segment {} both connect "
                "the same retained node pair (one graph edge per node pair, "
                "ADR-013)",
                existing->second.way_id, existing->second.segment_index, segment.way_id,
                segment.segment_index));
        }
        connected_by.emplace(pair_key,
                             UsedPair{segment.way_id, segment.segment_index});

        const map::EdgeDirection direction =
            segment.direction.forward && segment.direction.reverse
                ? map::EdgeDirection::Bidirectional
                : (segment.direction.forward ? map::EdgeDirection::Forward
                                             : map::EdgeDirection::Reverse);
        const common::EdgeId edge =
            graph_builder.connect(segment.from, segment.to, cost, direction);
        polylines.emplace_back(edge, segment.points);
    }

    const map::Graph graph = graph_builder.build();

    // ---- geographic side: canonical coordinate VALUES reused everywhere ---
    map::MapGeometry::Builder geometry_builder{graph.node_count(), graph.edge_count()};
    for (const auto& [osm_id, id] : node_ids) {
        geometry_builder.set_node_position(id, coordinates.at(osm_id));
    }
    for (const auto& [edge, points] : polylines) {
        geometry_builder.set_edge_polyline(edge, points);
    }

    OsmImportStats stats = ways.stats_;
    stats.topology_nodes = node_ids.size();
    stats.directed_edges = graph.edge_count();
    return OsmImportResult{
        map::BaseMap{graph, options_.map_version, geometry_builder.build()}, stats};
}

}  // namespace fleet::osm

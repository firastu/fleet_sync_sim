// Generates the committed OSM PBF test fixtures from embedded data.
//
// The fixtures' source of truth is THIS FILE plus the human-readable
// .osm XML committed next to the binaries. Regenerate with:
//
//   cmake --build build/debug --target fleet_osm_fixture_writer
//   ./build/debug/tests/fleet_osm_fixture_writer tests/fixtures/osm
//
// CI never runs this: the .osm.pbf artifacts are committed. Rebuilding
// them changes binary bytes (PBF header timestamps) but never the
// logical content the importer consumes.

#include <filesystem>
#include <format>
#include <iostream>
#include <vector>

#include <osmium/builder/attr.hpp>
#include <osmium/io/pbf_output.hpp>  // registers the PBF writer (include-time link)
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/location.hpp>

namespace {

using namespace osmium::builder::attr;

struct FixtureNode {
    std::int64_t id;
    double lat;
    double lon;
};

struct FixtureWay {
    std::int64_t id;
    const char* highway;
    const char* oneway;    // nullptr = no oneway tag
    const char* junction;  // nullptr = no junction tag
    std::vector<std::int64_t> refs;
};

struct Fixture {
    const char* file_name;
    std::vector<FixtureNode> nodes;
    std::vector<FixtureWay> ways;
};

// Main fixture: A(100) --101--102--103-- B(104) with a branch structure
// exercising every contract (see ADR-014). Node 102 is deliberately OFF
// the 100->104 line so way 500 is a BENT segment: its summed-polyline
// length must exceed the endpoint-to-endpoint geodesic (regression
// guard for full-polyline costs).
//   way 500: residential, bidirectional, intermediate geometry 101..103
//   way 600: residential, oneway=yes, intermediate node 200
//   way 700: unclassified, oneway=-1 (travel 300 -> 400 -> 500)
//   way 800: footway (excluded by the whitelist; node 600 never imported)
const Fixture kTinyNetwork{
    "tiny_network.osm.pbf",
    {{100, 53.55000, 9.99000}, {101, 53.55001, 9.99010}, {102, 53.55006, 9.99020},
     {103, 53.55003, 9.99030}, {104, 53.55004, 9.99040}, {200, 53.55014, 9.99050},
     {300, 53.55024, 9.99060}, {400, 53.55034, 9.99070}, {500, 53.55044, 9.99080},
     {600, 53.55054, 9.99090}},
    {},  // ways are declared in main (see below)
};

// Error fixture: an oneway value outside the supported subset.
const Fixture kInvalidOneway{
    "tiny_invalid_oneway.osm.pbf",
    {{10, 53.55000, 9.99000}, {11, 53.55001, 9.99001}},
    {},
};

// Error fixture: two distinct ways connecting the same node pair
// in opposite directions.
const Fixture kAntiparallel{
    "tiny_antiparallel.osm.pbf",
    {{20, 53.55000, 9.99000}, {21, 53.55001, 9.99001}},
    {},
};

// Error fixture: two distinct ways connecting the same node pair in the
// SAME direction — still two physical roads, still unrepresentable.
const Fixture kParallelSameDirection{
    "tiny_parallel_same_direction.osm.pbf",
    {{40, 53.55000, 9.99000}, {41, 53.55001, 9.99001}},
    {},
};

// Error fixture: junction=roundabout without an explicit oneway tag —
// implicit direction is unsupported and must never import silently.
const Fixture kImplicitRoundabout{
    "tiny_roundabout.osm.pbf",
    {{30, 53.55000, 9.99000}, {31, 53.55001, 9.99001}},
    {},
};

// Error fixture: one way whose refs revisit the same retained node pair
// (segments 60->61 and 61->60 from a single way).
const Fixture kSelfLoopPair{
    "tiny_self_loop_pair.osm.pbf",
    {{60, 53.55000, 9.99000}, {61, 53.55010, 9.99010}},
    {},
};

void write_fixture(const std::filesystem::path& directory, const Fixture& fixture) {
    osmium::memory::Buffer buffer{1024 * 1024, osmium::memory::Buffer::auto_grow::yes};
    for (const FixtureNode& node : fixture.nodes) {
        osmium::builder::add_node(buffer, _id(node.id), _version(1),
                                  _location(osmium::Location{node.lon, node.lat}));
    }
    for (const FixtureWay& way : fixture.ways) {
        std::vector<std::pair<const char*, const char*>> tags{{"highway", way.highway}};
        if (way.oneway != nullptr) {
            tags.emplace_back("oneway", way.oneway);
        }
        if (way.junction != nullptr) {
            tags.emplace_back("junction", way.junction);
        }
        osmium::builder::add_way(buffer, _id(way.id), _version(1), _tags(tags),
                                 _nodes(way.refs));
    }

    const std::filesystem::path output = directory / fixture.file_name;
    osmium::io::Writer writer{osmium::io::File{output, "osm.pbf"}};
    writer(std::move(buffer));
    writer.close();
    std::cout << std::format("wrote {}\n", output.string());
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: fleet_osm_fixture_writer <output-directory>\n";
        return 2;
    }
    const std::filesystem::path directory{argv[1]};
    std::filesystem::create_directories(directory);

    // Main fixture (ways declared here to keep the table literal simple).
    const std::vector<FixtureWay> network_ways{
        FixtureWay{500, "residential", nullptr, nullptr, {100, 101, 102, 103, 104}},
        FixtureWay{600, "residential", "yes", nullptr, {104, 200, 300}},
        FixtureWay{700, "unclassified", "-1", nullptr, {500, 400, 300}},
        FixtureWay{800, "footway", nullptr, nullptr, {100, 600}},
    };
    const Fixture network{kTinyNetwork.file_name, kTinyNetwork.nodes, network_ways};

    const std::vector<FixtureWay> invalid_ways{
        FixtureWay{900, "residential", "alternating", nullptr, {10, 11}},
    };
    const Fixture invalid{kInvalidOneway.file_name, kInvalidOneway.nodes, invalid_ways};

    const std::vector<FixtureWay> antiparallel_ways{
        FixtureWay{910, "residential", "yes", nullptr, {20, 21}},
        FixtureWay{920, "residential", "yes", nullptr, {21, 20}},
    };
    const Fixture antiparallel{kAntiparallel.file_name, kAntiparallel.nodes,
                               antiparallel_ways};

    const std::vector<FixtureWay> parallel_ways{
        FixtureWay{930, "residential", "yes", nullptr, {40, 41}},
        FixtureWay{940, "residential", "yes", nullptr, {40, 41}},
    };
    const Fixture parallel{kParallelSameDirection.file_name, kParallelSameDirection.nodes,
                           parallel_ways};

    const std::vector<FixtureWay> roundabout_ways{
        FixtureWay{950, "residential", nullptr, "roundabout", {30, 31}},
    };
    const Fixture roundabout{kImplicitRoundabout.file_name, kImplicitRoundabout.nodes,
                             roundabout_ways};

    // One way whose refs revisit retained nodes: segments 60->61 and
    // 61->60 map to the same unordered pair (duplicate from a SINGLE way).
    const std::vector<FixtureWay> self_loop_ways{
        FixtureWay{960, "residential", nullptr, nullptr, {60, 61, 60, 61, 60}},
    };
    const Fixture self_loop{kSelfLoopPair.file_name, kSelfLoopPair.nodes, self_loop_ways};

    write_fixture(directory, network);
    write_fixture(directory, invalid);
    write_fixture(directory, antiparallel);
    write_fixture(directory, parallel);
    write_fixture(directory, roundabout);
    write_fixture(directory, self_loop);
    return 0;
}

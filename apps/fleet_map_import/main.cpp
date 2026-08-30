// fleet_map_import — thin CLI over the OSM PBF importer (ADR-014).
//
//   fleet_map_import <map.osm.pbf>
//
// Imports the extract deterministically and prints the statistics. The
// library owns everything; this binary only parses one argument.

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>

#include "fleet/osm/osm_pbf_importer.hpp"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: fleet_map_import <map.osm.pbf>\n";
        return 2;
    }
    try {
        const fleet::osm::OsmImportResult result =
            fleet::osm::OsmPbfImporter{}.import(std::filesystem::path{argv[1]});
        std::cout << std::format(
            "ways seen: {}\nways imported: {}\ntopology nodes: {}\ngraph edges: {}\n",
            result.stats.ways_seen, result.stats.ways_imported,
            result.stats.topology_nodes, result.stats.directed_edges);
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << std::format("fleet_map_import: {}\n", error.what());
        return EXIT_FAILURE;
    }
}

// fleet_map_import — thin CLI over the OSM PBF importer (ADR-014).
//
//   fleet_map_import <map.osm.pbf> [--map-geojson <out.geojson>]
//
// Imports the extract deterministically, prints the statistics and,
// with --map-geojson, writes the imported map's debug GeoJSON export
// (ADR-015). The libraries own everything; this binary only parses
// arguments.

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <span>
#include <string>

#include "fleet/geojson/geojson_export.hpp"
#include "fleet/osm/osm_pbf_importer.hpp"

int main(int argc, char* argv[]) {
    std::span<const char* const> args{argv + 1, argv + argc};
    if (args.empty() || args.size() > 3) {
        std::cerr << "usage: fleet_map_import <map.osm.pbf> [--map-geojson <out.geojson>]\n";
        return 2;
    }
    std::filesystem::path geojson_output;
    if (args.size() == 3 && std::string{args[1]} == "--map-geojson") {
        geojson_output = args[2];
    } else if (args.size() != 1) {
        std::cerr << "usage: fleet_map_import <map.osm.pbf> [--map-geojson <out.geojson>]\n";
        return 2;
    }

    try {
        const fleet::osm::OsmImportResult result =
            fleet::osm::OsmPbfImporter{}.import(std::filesystem::path{args[0]});
        std::cout << std::format(
            "ways seen: {}\nways imported: {}\ntopology nodes: {}\ngraph edges: {}\n",
            result.stats.ways_seen, result.stats.ways_imported,
            result.stats.topology_nodes, result.stats.directed_edges);

        if (!geojson_output.empty()) {
            std::ofstream output{geojson_output, std::ios::out | std::ios::trunc};
            if (!output.is_open()) {
                throw std::runtime_error(
                    std::format("cannot open GeoJSON output '{}'", geojson_output.string()));
            }
            const std::size_t features =
                fleet::geojson::write_base_map_geojson(result.map, output);
            std::cout << std::format("geojson features written: {}\n", features);
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << std::format("fleet_map_import: {}\n", error.what());
        return EXIT_FAILURE;
    }
}

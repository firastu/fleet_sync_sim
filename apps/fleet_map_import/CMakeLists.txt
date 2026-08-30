# fleet_map_import: thin CLI over the OSM importer (ADR-014). Prints
# import statistics; contains no importer logic.
add_executable(fleet_map_import main.cpp)
target_link_libraries(fleet_map_import PRIVATE fleet::osm_import fleet::geojson)
fleet_target_warnings(fleet_map_import)
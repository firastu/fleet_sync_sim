include(FetchContent)

# Scenario JSON parsing: one small, well-contained dependency, pinned and
# PRIVATE to the loader — parser types never appear in public headers.
find_package(nlohmann_json CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
  )
  # Header-only use; skip the library's own tests and install rules.
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  set(JSON_Install OFF CACHE INTERNAL "")
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# Header-only shared vocabulary: strongly-typed ids, logical time.
add_library(fleet_common INTERFACE)
add_library(fleet::common ALIAS fleet_common)
target_include_directories(fleet_common INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

# Immutable map core: graph, base map, dynamic overlay, composed view,
# delta reconciliation.
add_library(fleet_map STATIC
  map/graph.cpp
  map/base_map.cpp
  map/dynamic_overlay.cpp
  map/map_view.cpp
  map/map_reconciler.cpp)
add_library(fleet::map ALIAS fleet_map)
target_include_directories(fleet_map PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(fleet_map PUBLIC fleet_common)
fleet_target_warnings(fleet_map)

# Deterministic planning over MapView.
add_library(fleet_planning STATIC
  planning/a_star_planner.cpp)
add_library(fleet::planning ALIAS fleet_planning)
target_link_libraries(fleet_planning PUBLIC fleet::map)
fleet_target_warnings(fleet_planning)

# Deterministic simulation core: logical clock, discrete event queue,
# deterministic bounded sampler.
add_library(fleet_simulation STATIC
  simulation/event_queue.cpp
  simulation/deterministic_rng.cpp)
add_library(fleet::simulation ALIAS fleet_simulation)
target_link_libraries(fleet_simulation PUBLIC fleet::common)
fleet_target_warnings(fleet_simulation)

# Deterministic unreliable point-to-point transport simulation.
add_library(fleet_network STATIC
  network/network_simulator.cpp)
add_library(fleet::network ALIAS fleet_network)
target_link_libraries(fleet_network PUBLIC fleet::simulation fleet::map)
fleet_target_warnings(fleet_network)

# Autonomous participants: mission, local knowledge, routing autonomy.
add_library(fleet_robot STATIC
  robot/robot.cpp)
add_library(fleet::robot ALIAS fleet_robot)
target_link_libraries(fleet_robot PUBLIC fleet::planning fleet::map)
fleet_target_warnings(fleet_robot)

# Local control station: fleet-wide knowledge aggregation.
add_library(fleet_station STATIC
  station/control_station.cpp)
add_library(fleet::station ALIAS fleet_station)
target_link_libraries(fleet_station PUBLIC fleet::map)
fleet_target_warnings(fleet_station)

# Scenario runner + deterministic structured trace: wiring and scheduling
# over existing public APIs only (ADR-009). The JSON parser is PRIVATE to
# the loader; everything downstream consumes typed Scenario data.
add_library(fleet_scenario STATIC
  scenario/trace.cpp
  scenario/scenario_loader.cpp
  scenario/scenario_runner.cpp)
add_library(fleet::scenario ALIAS fleet_scenario)
target_include_directories(fleet_scenario PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(fleet_scenario
  PUBLIC fleet::robot fleet::network fleet::station
  PRIVATE nlohmann_json::nlohmann_json)
fleet_target_warnings(fleet_scenario)

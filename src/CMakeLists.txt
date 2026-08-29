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

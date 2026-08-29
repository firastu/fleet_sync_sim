include(FetchContent)

# Prefer a system/SDK GoogleTest if present; fetch a pinned release otherwise.
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

add_executable(fleet_unit_tests
  unit/map/graph_test.cpp
  unit/map/base_map_test.cpp
  unit/map/dynamic_overlay_test.cpp
  unit/map/map_view_test.cpp
  unit/map/map_reconciler_test.cpp
  unit/planning/a_star_planner_test.cpp
  unit/simulation/event_queue_test.cpp
  unit/simulation/deterministic_rng_test.cpp
  unit/network/network_simulator_test.cpp
  unit/robot/robot_test.cpp
  unit/robot/robot_movement_test.cpp
  unit/station/control_station_test.cpp
  unit/scenario/scenario_test.cpp
  unit/scenario/scenario_runner_test.cpp
  unit/scenario/scenario_movement_test.cpp
  unit/scenario/scenario_sensing_test.cpp
  unit/world/world_test.cpp)
target_include_directories(fleet_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/support)
target_link_libraries(fleet_unit_tests PRIVATE fleet::station fleet::robot fleet::network
                                           fleet::planning fleet::scenario
                                           GTest::gtest_main)
fleet_target_warnings(fleet_unit_tests)

# Scenario files under test are part of the declarative contract: copy them
# next to the test binary so loader tests run the real on-disk artifacts.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../scenarios
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

include(GoogleTest)
gtest_discover_tests(fleet_unit_tests)

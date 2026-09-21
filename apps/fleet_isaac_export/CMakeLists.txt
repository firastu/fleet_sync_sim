# fleet_isaac_export — Isaac stage 1 read-only replay exporter (ADR-020).
# CPU-only: this target never locates or links Isaac, CUDA, USD, Python or
# ROS in either configuration of FLEET_BUILD_ISAAC_TOOLS. The Isaac-side
# viewer is tools/isaac/replay.py and runs through NVIDIA's own Python.
#
# The core is a library so tests share the exact adapter code without any
# of it entering public domain headers; nlohmann_json stays PRIVATE.
add_library(fleet_isaac_export_core STATIC
  isaac_fixture_map.cpp
  exporter.cpp)
add_library(fleet::isaac_export_core ALIAS fleet_isaac_export_core)
target_include_directories(fleet_isaac_export_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(fleet_isaac_export_core
  PUBLIC fleet::scenario
  PRIVATE nlohmann_json::nlohmann_json)
fleet_target_warnings(fleet_isaac_export_core)

#   fleet_isaac_export --scenario <file> --out <dir> [--seed <uint64>]
#                       [--period-ms <n>] [--footprint-m <m>] [--trace <file>]
add_executable(fleet_isaac_export main.cpp)
target_link_libraries(fleet_isaac_export PRIVATE fleet::isaac_export_core)
fleet_target_warnings(fleet_isaac_export)

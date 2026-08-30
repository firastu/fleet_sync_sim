# fleet_console: interactive stepping/injection console over the runner
# (#13). Thin REPL — all semantics live in the libraries.
add_executable(fleet_console main.cpp ../fleet_sim/demo_map.cpp)
target_include_directories(fleet_console PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../fleet_sim)
target_link_libraries(fleet_console PRIVATE fleet::scenario)
fleet_target_warnings(fleet_console)
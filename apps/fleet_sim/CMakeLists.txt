# fleet_sim: CLI over the scenario runner (ADR-009).
#   fleet_sim                                run the built-in founding scenario
#   fleet_sim --scenario <file> [--seed <n>] [--trace <file>]
add_executable(fleet_sim main.cpp demo_map.cpp)
target_link_libraries(fleet_sim PRIVATE fleet::scenario)
fleet_target_warnings(fleet_sim)

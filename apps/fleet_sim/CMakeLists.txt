# Minimal developer executable: two robots, one station, one link model.
add_executable(fleet_sim main.cpp demo_map.cpp)
target_link_libraries(fleet_sim PRIVATE fleet::station fleet::robot fleet::network
                                        fleet::planning fleet::map)
fleet_target_warnings(fleet_sim)

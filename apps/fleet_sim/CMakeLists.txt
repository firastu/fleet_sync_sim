# Minimal developer executable. Links the map library only; robots,
# planning, networking and the scenario runner are later milestones.
add_executable(fleet_sim main.cpp)
target_link_libraries(fleet_sim PRIVATE fleet::map)
fleet_target_warnings(fleet_sim)

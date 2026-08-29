# Minimal developer executable: map core, planning, reconciliation and
# network transport demo.
add_executable(fleet_sim main.cpp demo_map.cpp)
target_link_libraries(fleet_sim PRIVATE fleet::network fleet::planning fleet::map)
fleet_target_warnings(fleet_sim)

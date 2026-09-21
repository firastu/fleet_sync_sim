add_subdirectory(fleet_sim)
add_subdirectory(fleet_map_import)
add_subdirectory(fleet_console)

# Isaac stage 1 read-only replay exporter (ADR-020): opt-in, default OFF.
if(FLEET_BUILD_ISAAC_TOOLS)
  add_subdirectory(fleet_isaac_export)
endif()

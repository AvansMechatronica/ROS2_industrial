# Add sources for each plugin to be registered.
add_library(VacuumGripper VacuumGripper.cc )
set_property(TARGET VacuumGripper PROPERTY CXX_STANDARD 17)

target_link_libraries(VacuumGripper
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
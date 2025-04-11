#set(GZ_LIB_INSTALL_DIR  '.')
#set(GZ_INCLUDE_INSTALL_DIR_FULL  'include/${PROJECT_NAME}')
#set(GZ_LIB_INSTALL_DIR  'lib/${PROJECT_NAME}')
#set(GZ_PLUGIN_INSTALL_DIR  'lib/${PROJECT_NAME}')
#set(GZ_PLUGIN_INSTALL_DIR  'lib/${PROJECT_NAME}')
# Add sources for each plugin to be registered.
add_library(VacuumGripper VacuumGripper.cc)
#gz_add_component(VacuumGripper SOURCES  VacuumGripper.cc)
set_property(TARGET VacuumGripper PROPERTY CXX_STANDARD 17)

#target_compile_definitions(${logical_camera_target} PUBLIC LogicalCameraSensor_EXPORTS)


target_link_libraries(VacuumGripper
  PRIVATE
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(TARGETS
  VacuumGripper
  DESTINATION lib/${PROJECT_NAME}
)
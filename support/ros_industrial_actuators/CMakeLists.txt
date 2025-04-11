cmake_minimum_required(VERSION 3.8)
project(ros_industrial_actuators)

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)

#--------------------------------------
# Find gz-common
#--------------------------------------
gz_find_package(gz-common5
                 COMPONENTS profiler
                 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
#--------------------------------------
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-msgs
#--------------------------------------
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})


#--------------------------------------
# Find gz-sim
#--------------------------------------
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#gz_configure_project(VERSION_SUFFIX)


#============================================================================
# Set project-specific options
#============================================================================
set (DRI_TESTS TRUE CACHE BOOL "True to enable DRI tests")

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DGZ_PROFILER_ENABLE=1")
else()
  add_definitions("-DGZ_PROFILER_ENABLE=0")
endif()

add_subdirectory(vacuum_gripper)

# Install C++ components
#install(DIRECTORY include/
#        DESTINATION include)

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Python package installation
ament_python_install_package(${PROJECT_NAME})

# Python installation
install(PROGRAMS
  ros_industrial_actuators/vacuum_gripper.py
  launch
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
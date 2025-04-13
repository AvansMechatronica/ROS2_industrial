cmake_minimum_required(VERSION 3.8)
project(ros_industrial_actuators)

#cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-GUI_CXX_FLAGS}")

qt5_add_resources(resources_rcc Turtlebot4Hmi.qrc)

include_directories(SYSTEM
  ${GZ-COMMON_INCLUDE_DIRS}

)

link_directories(
  ${GZ-COMMON_LIBRARY_DIRS}
)

add_library(VacuumGripper SHARED vacuum_gripper/VacuumGripper.cc)

target_link_libraries(VacuumGripper
    ${GZ-COMMON_LIBRARIES}
    ${GZ-PLUGIN_LIBRARIES}
    ${GZ-SIM_LIBRARIES}
)



install(TARGETS
  VacuumGripper
  DESTINATION lib/${PROJECT_NAME}
)
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

install(
  TARGETS VacuumGripper
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
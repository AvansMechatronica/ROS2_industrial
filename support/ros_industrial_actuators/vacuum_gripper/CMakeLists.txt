cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)
set(OpenGL_GL_PREFERENCE LEGACY)



# Find the Gz gui library
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz-plugin REQUIRED)

#find_package(gz-sim REQUIRED)  # Adjust version as needed

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-GUI_CXX_FLAGS}")


include_directories(SYSTEM
  ${GZ-COMMON_INCLUDE_DIRS}
  ${GZ-PLUGIN_INCLUDE_DIRS}
)

link_directories(
  ${GZ-COMMON_LIBRARY_DIRS}
)

# Generate examples
# MOC parsing is broken on Ionic: https://github.com/gazebosim/gz-msgs/issues/463
add_library(VacuumGripper VacuumGripper.cc
)


target_link_libraries(VacuumGripper
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

install(
  TARGETS VacuumGripper
  DESTINATION share/${PROJECT_NAME}/lib
)

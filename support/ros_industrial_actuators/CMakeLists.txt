cmake_minimum_required(VERSION 3.8)
project(ros_industrial_actuators)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros_industrial_msgs REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Custom Logical Camera Plugin
add_library(MyVacuumGripperPlugin SHARED
  src/my_vacuum_gripper_plugin.cpp
)
target_include_directories(MyVacuumGripperPlugin PUBLIC include)
ament_target_dependencies(MyVacuumGripperPlugin
  "gazebo_ros"
  "ros_industrial_msgs"
  "std_msgs"
  "std_srvs"
)
ament_export_libraries(MyVacuumGripperPlugin)


# Install C++ components
install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    MyVacuumGripperPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Python package installation
ament_python_install_package(${PROJECT_NAME})

# Python installation
install(PROGRAMS
  #ros_industrial_actuators/custom_logical_camera.py
  ros_industrial_actuators/vacuum_gripper.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

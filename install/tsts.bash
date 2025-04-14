#!/bin/bash


# tempory patch for vacuum gripper plugin
if ! env | grep -q "GZ_SIM_SYSTEM_PLUGIN_PATH"; then
 echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ros2_industrial_ws/install/ros_industrial_actuators/lib/ros_industrial_actuators/" >> ~/.bashrc
fi
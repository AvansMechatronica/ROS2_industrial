#!/bin/bash
sudo apt update

# Update rosdep
rosdep update

# Environment Setup
source /opt/ros/$ROS_DISTRO/setup.bash
CURRENT_DIR=$(pwd)

# Install TurtleBot 3 Packages
TURTLEBOT_DIR=~/turtlebot3_ws
if ! ros2 pkg list | grep -q "turtlebot3"; then

    echo "Installing TurtleBot 3 packages"
    mkdir -p "$TURTLEBOT_DIR/src"
    cd "$TURTLEBOT_DIR/src"
    git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd "$TURTLEBOT_DIR"
    rosdep install --from-paths . --ignore-src -r -y
    colcon build --symlink-install  
    
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
    echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
    #echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
    source $TURTLEBOT_DIR/install/setup.bash
else
    echo "TurtleBot 3 packages already installed"
fi

cd "$CURRENT_DIR"

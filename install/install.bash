#!/bin/bash

# Environment Setup
source /opt/ros/$ROS_DISTRO/setup.bash
CURRENT_DIR=$(pwd)

pip install numpy==1.24.3 scipy==1.10.1

# Update rosdep
sudo rosdep init
rosdep update

# Install dependencies for the current workspace
cd $CURRENT_DIR/..
rosdep install --from-paths install --ignore-src -r -y

# Install xArm Packages
XARM_DIR=~/xarm_ws
if ! ros2 pkg list | grep -q "xarm_description"; then
    echo "Cloning xarm packages"
    mkdir -p "$XARM_DIR/src"
    cd "$XARM_DIR/src"
    git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
    git pull
    git submodule sync
    git submodule update --init --remote
    cd "$XARM_DIR"
    colcon build --symlink-install
    echo "source $XARM_DIR/install/setup.bash" >> ~/.bashrc
    source $XARM_DIR/install/setup.bash
else
    echo "xArm packages already installed"
fi

# Install pymoveit2
cd "$CURRENT_DIR"/../..
if ! ros2 pkg list | grep -q "pymoveit2"; then
    echo "Cloning pymoveit2"
    git clone https://github.com/AvansMechatronica/pymoveit2.git 
else
    echo "pymoveit2 already installed"
fi

# Install My MoveIt Python
if ! ros2 pkg list | grep -q "my_moveit_python"; then
    echo "Cloning my_moveit_python"
    git clone https://github.com/AvansMechatronica/my_moveit_python.git 
else
    echo "my_moveit_python already installed"
fi

# Install TurtleBot 3 Packages
TURTLEBOT_DIR=~/turtlebot_ws
if ! ros2 pkg list | grep -q "turtlebo3"; then
    echo "Installing TurtleBot 3 packages"
    mkdir -p "$TURTLEBOT_DIR/src"
    cd "$TURTLEBOT_DIR/src"
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b $ROS_DISTRO
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b $ROS_DISTRO
    cd "$TURTLEBOT_DIR"
    rosdep install --from-paths . --ignore-src -r -y
    colcon build --symlink-install
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
    echo "source $TURTLEBOT_DIR/install/setup.bash" >> ~/.bashrc
    source $TURTLEBOT_DIR/install/setup.bash
else
    echo "TurtleBot 3 packages already installed"
fi

cd "$CURRENT_DIR"

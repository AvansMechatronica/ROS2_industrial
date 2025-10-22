#!/bin/bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-gz-ros2-control -y
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui -y
sudo apt install ros-$ROS_DISTRO-moveit* -y
sudo apt install ros-$ROS_DISTRO-geometric-shapes
sudo apt install ros-$ROS_DISTRO-srdfdom -y
sudo apt update
sudo apt install ros-$ROS_DISTRO-eigen-stl-containers
sudo apt -y install ros-$ROS_DISTRO-tf-transformations
sudo apt install python3-pip -y

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



if ! env | grep -q "QT_QPA_PLATFORM=xcb"; then
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc
fi

# tempory patch for vacuum gripper plugin
if ! env | grep -q "GZ_SIM_SYSTEM_PLUGIN_PATH"; then
 echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ros2_industrial_ws/install/ros_industrial_actuators/lib/ros_industrial_actuators/" >> ~/.bashrc
fi

# Install xArm Packages
XARM_DIR=~/xarm_ws
if ! ros2 pkg list | grep -q "xarm_description"; then
    # Missing in xarm dependencys
    sudo apt install ros-$ROS_DISTRO-moveit
    echo "Cloning xarm packages"
    mkdir -p "$XARM_DIR/src"
    cd "$XARM_DIR/src"
    git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
    git pull
    git submodule sync
    git submodule update --init --remote
    cd "$XARM_DIR"
    rosdep install --from-paths install --ignore-src -r -y
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


cd "$CURRENT_DIR"

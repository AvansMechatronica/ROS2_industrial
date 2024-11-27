# ROS2 industrial (Humble-version)

## Pre-requistions ROS2-Humble:
Following instructions for [Installing ROS-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Choose: ros-humble-desktop

## Cloning this repositry:
```bash
mkir -p /ros_inbdustrial_ws/src
cd /ros_inbdustrial_ws/src
gir clone https://github.com/AvansMechatronica/ROS2_industrial.git
```

## Installing nessery packages etc.
```bash
cd install
.install
```

## Use for build: 
```bash
colcon build --symlink-install
```

## Note for windows users:
You can use the WSL Ubuntu-22.04 distribution form the Microsoft Store. Use as development environment [Visual Studio Code](https://code.visualstudio.com/download). Alter installing add the WSL-plugin to Visual Studio Code. Open the distribution with <F1>WSL: Connect to WSL. Don't forget to install ROS-Humble into the distribution. Clone this repositry into te WSL distribution

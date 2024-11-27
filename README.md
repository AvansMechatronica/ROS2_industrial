# ROS2 industrial (Humble-version)

## Pre-requistions ROS2-Humble:
Following instructions for [Installing ROS-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Choose: ros-humble-desktop

## Cloning this repositry:
```bash
mkir -p /ros2_industrial_ws/src
cd /ros2_industrial_ws/src
git clone https://github.com/AvansMechatronica/ROS2_industrial.git
```

## Install nessery packages etc.
```bash
cd install
.install
```

## Use for build: 
```bash
cd ~/ros2_industrial_ws/
colcon build --symlink-install
source install/setup.bash
echo "source ~/ros2_industrial_ws/install/setup.bash" >> ~/.bashrc
```
Note: Execute last line only once

## Note for windows users:
You can use the WSL Ubuntu-22.04 distribution form the Microsoft Store. Use as development environment [Visual Studio Code](https://code.visualstudio.com/download). Alter installing add the WSL-plugin to Visual Studio Code. Open the distribution with <F1>WSL: Connect to WSL. Don't forget to install ROS-Humble into the distribution. Clone this repositry into te WSL distribution


## Workshops:

[ROS2 Basics](1_basics/README.md)


[ROS2 Urdf](2_urdf/README.md)

[ROS2 Navigation](3_navigation/README.md)

[ROS2 Manipulation](4_manipulation/README.md)

[ROS2 Transferframes](5_transferframes/README.md)

[ROS2 Project](6_project/README.md)
## Verantwoording:

Deze workshop is geinspireerd op [hello real world ros robot operating system](https://ocw.tudelft.nl/courses/hello-real-world-ros-robot-operating-system/) van de Technische Universiteit Delft/Nederland(TUD)

## [Licentie](licence.md)
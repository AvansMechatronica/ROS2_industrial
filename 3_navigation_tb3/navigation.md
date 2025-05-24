# ROS2 navigation
## Commando's
Under contruction


```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc

```


Starten van de omgeving
```bash
ros2 launch navigation_tb3 environmet.launch.py
```

```bash
ros2 launch navigation_tb3 environmet.launch.py without_obstacles:=True
```

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

```bash
ros2 launch navigation_tb3 cartographer.launch.py use_sim_time:=True
```

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```



```bash
ros2 launch navigation_tb3 navigate.launch.py use_sim_time:=True map:=~/home/student~/ros2_industrial_ws/src/ROS2_industrial/3_navigation/navigation/maps/map_factory_v1.yaml
```


# ROS2 navigation
## Commando's
Under contruction




Patch orignal Turtlebut4 simulation
```bash
cd ~/ros2_industrial_ws/src/ROS2_industrial/3_navigation/patch_turtlebot4_simulation/
./patch.bash
```



Remove Patch
```bash
cd ~/ros2_industrial_ws/src/ROS2_industrial/3_navigation/patch_turtlebot4_simulation/
./unpatch.bash
```


Bekijken van de omgeving
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:maze
```

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:depot
```



Rondrijden
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

met slam
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py slam:=true nav2:=true rviz:=true <world:=warehouse/maze/depot>
```

save map
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```

navigatie
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=true slam:=false localization:=true rviz:=true 

```



maze
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=maze map:=/home/<user_id>/turtlebot_ws/src/turtlebot4/turtlebot4_navigation/maps/maze.yaml
```

depot
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=depot map:=/home/<user_id>/turtlebot_ws/src/turtlebot4/turtlebot4_navigation/maps/depot.yaml
```
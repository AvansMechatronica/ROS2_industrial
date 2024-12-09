# ROS2 navigation

## Under development

Starten van de omgeving
```bash
ros2 launch navigation environment.launch.py
```

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

```bash
ros2 launch navigation cartographer.launch.py use_sim_time:=True
```

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```



```bash
ros2 launch navigation navigate.launch.py use_sim_time:=True map:=~/home/student~/ros2_industrial_ws/src/ROS2_industrial/3_navigation/navigation/maps/map_factory_v1.yaml
```

[Back](../README.md)
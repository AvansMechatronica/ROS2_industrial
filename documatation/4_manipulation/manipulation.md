# ROS2 industrial manipulation
## Commando's
Under contruction

Bekijken van de omgeving
```bash
ros2 launch manipulation view_environment.launch.py
```

Starten nieuwe moveit-configratie in setup assistant
```bash
ros2 launch moveit_setup_assistant  setup_assistant.launch.py
```

Starten bestaande moveit-configuratie met setup assistant
```bash
ros2 launch manipulation_moveit_config setup_assistant.launch.py 
```

## Opdracht 1
```bash
ros2 launch manipulation environment.launch.py
```

starten opdracht1
```bash
ros2 run manipulation assignment1 
```
## Opdracht 2
```bash
ros2 launch manipulation environment_w_gazebo.launch.py
```
```bash
ros2 launch manipulation spawn_battery.launch.py
```
```bash
ros2 run manipulation assignment2
```

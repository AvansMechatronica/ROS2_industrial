# Patch for simulated lidar

```bash
sudo gedit /opt/ros/jazzy/share/irobot_create_description/urdf/create3.urdf.xacro
```

Open the file using your preferred text editor and locate the following line:

```text
<render_engine>ogre</render_engine> 
```

Modify it to:

```text
<render_engine>ogre2</render_engine> 
```

Save the file and restart your simulation.

Why This Works:
Switching the rendering engine to ogre2 resolves the issue by leveraging better rendering capabilities using CPU, which corrects the LiDAR's range behavior.

## Thanks to: Rahgir Arefin Rafi

[Info/Link](https://www.linkedin.com/posts/rahgirrafi_ros2-turtlebot4-gazebo-activity-7284297279061028864-JWBu/?utm_source=share&utm_medium=member_desktop)
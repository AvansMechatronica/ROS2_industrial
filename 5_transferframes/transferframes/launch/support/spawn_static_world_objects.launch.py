from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import xacro
import os
import random
import subprocess

def spawn_gazebo_object(path, entity, x = 0, y = 0, z = 0):

    command = ["xacro", path]
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    description = result.stdout  # Capture the output of the command

    tmp_file = "tmp.urdf"+ str(random.randint(0,1000))
    with open(tmp_file, "w") as file:
        file.write(description)


    node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=[LaunchConfiguration('workshop'), 'spawner'],
            output='screen',
            arguments=[
                '-entity', entity+str(random.randint(0,100)),
                '-file', tmp_file,
                '-timeout', '10.0',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
            ],
        )
    #os.remove(tmp_file)
    return node

def generate_launch_description():
    # Declare launch arguments
        # Declare launch arguments
    launch_args = [
      DeclareLaunchArgument('workshop', default_value='workshop_'),
      DeclareLaunchArgument('workshop_parent_name', default_value='world_interface'),
      DeclareLaunchArgument('hall_prefix', default_value='hall_'),
      DeclareLaunchArgument('hall_parent', default_value='world_interface'),
      DeclareLaunchArgument('robot1_prefix', default_value='robot1_'),
      DeclareLaunchArgument('robot1_pedestal', default_value='robot1_pedestal_'),
      DeclareLaunchArgument('vacuum_gripper1_prefix', default_value='vacuum_gripper1_'),
      DeclareLaunchArgument('bin_1', default_value='bin_1_'),
    ]

    gazebo_models_package_dir = os.path.join(get_package_share_directory('ros_industrial_support'), 'urdf')

    robot1_pedestal = os.path.join(gazebo_models_package_dir, 'robot_pedestal', 'robot1_pedestal.xacro')
    #robot1_pedestal_spawner = spawn_gazebo_object(robot1_pedestal, "robot1_pedestal", 0 ,0 ,0)
    bin1 = os.path.join(gazebo_models_package_dir, 'bin', 'bin.xacro')
    bin_spawner = spawn_gazebo_object(bin1, "bin1", 0 ,0 ,0)



    if 0:
      bin_1_spawner = Node(
          package='gazebo_ros',
          executable='spawn_entity.py',
          name=[LaunchConfiguration('bin_1'), 'spawner'],
          output='screen',
          arguments=[
              '-x', '-4',
              '-y', '-2',
              '-urdf',
              '-model', LaunchConfiguration('bin_1'),
              '-param', [LaunchConfiguration('bin_1'), 'description']
          ],
      )

      robot1_pedestal_spawner = Node(
          package='gazebo_ros',
          executable='spawn_entity.py',
          name=[LaunchConfiguration('robot1_pedestal'), 'spawner'],
          output='screen',
          arguments=[
              '-x', '-4',
              '-y', '-1',
              '-urdf',
              '-model', LaunchConfiguration('robot1_pedestal'),
              '-param', [LaunchConfiguration('robot1_pedestal'), 'description']
          ],
      )

      hall_spawner = Node(
          package='gazebo_ros',
          executable='spawn_entity.py',
          name=[LaunchConfiguration('hall_prefix'), 'spawner'],
          output='screen',
          arguments=[
              '-urdf',
              '-model', LaunchConfiguration('hall_prefix'),
              '-param', [LaunchConfiguration('hall_prefix'), 'description']
          ],
      )

    # Return launch description
    return LaunchDescription(launch_args + [
        #workshop_arg,
        #workshop_parent_name_arg,
        #hall_prefix_arg,
        #hall_parent_arg,
        #robot1_prefix_arg,
        #robot1_pedestal_arg,
        #vacuum_gripper1_prefix_arg,
        #bin_1_arg,
        #robot1_pedestal_spawner,
        bin_spawner,
        #bin_1_spawner,
        #robot1_pedestal_spawner,
        #hall_spawner

    ])

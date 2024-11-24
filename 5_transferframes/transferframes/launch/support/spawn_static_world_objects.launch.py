from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import xacro
import os

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
    bin_path = os.path.join(gazebo_models_package_dir, 'bin', 'bin.xacro')
    bin_desc = xacro.process_file(bin_path)
    bin_desc_xml = bin_desc.toxml()

    bin_description = Command(['xacro ', bin_path])
    

    # Spawn nodes
    workshop_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=[LaunchConfiguration('workshop'), 'spawner'],
        output='screen',
        arguments=[
            '-entity', "tmp3",
            '-file', bin_path,
            '-timeout', '10.0',
            '-x', '-4',
            '-y', '-2',
        ],
    )
  #<node name="$(arg workshop)spawner" pkg="gazebo_ros" type="spawn_model" output="screen"
  #  args="-urdf -model $(arg workshop) -param $(arg workshop)description"/>


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
        workshop_spawner,
        #bin_1_spawner,
        #robot1_pedestal_spawner,
        #hall_spawner
    ])

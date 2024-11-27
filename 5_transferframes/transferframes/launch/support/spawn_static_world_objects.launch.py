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
import math

def spawn_gazebo_object(path, entity, x = 0, y = 0, z = 0, R = 0, P = 0, Y = 0):

    command = ["xacro", path]
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    description = result.stdout  # Capture the output of the command

    tmp_file = path + ".kanweg"

    #os.remove(tmp_file)
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
                '-R', str(R),
                '-P', str(P),
                '-Y', str(Y),
            ],
        )

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

    #robot1_pedestal = os.path.join(gazebo_models_package_dir, 'robot_pedestal', 'robot1_pedestal.xacro')
    #robot1_pedestal_spawner = spawn_gazebo_object(robot1_pedestal, "robot1_pedestal", 0 ,0 ,0)

    robot1_pedestal_path = os.path.join(gazebo_models_package_dir, 'robot_pedestal', 'robot1_pedestal.xacro')
    robot1_pedestal_spawner = spawn_gazebo_object(robot1_pedestal_path, "robot1_pedestal", 0 ,0 ,0)

    bin1_path = os.path.join(gazebo_models_package_dir, 'bin', 'bin.xacro')
    bin_spawner = spawn_gazebo_object(bin1_path, "bin1", -0.5 ,0.5 ,0)

    computer_mobile_path = os.path.join(gazebo_models_package_dir, 'computer_mobile', 'computer.xacro')
    computer_mobile_spawner = spawn_gazebo_object(computer_mobile_path, "computer_mobile", 1.5 ,-0.5 ,0, Y=math.radians(45))

    assembly_station_path = os.path.join(gazebo_models_package_dir, 'assembly_station', 'assembly_station.xacro')
    assembly_station_spawner = spawn_gazebo_object(assembly_station_path, "assembly_station", 0.5 ,-0.5 ,Y=math.radians(90))

    ufactory_xarm6_w_vacuum_gripper_path = os.path.join(gazebo_models_package_dir, 'ufactory', 'ufactory_xarm6_w_vacuum_gripper.xacro')
    ufactory_xarm6_w_vacuum_gripper_spawner = spawn_gazebo_object(ufactory_xarm6_w_vacuum_gripper_path, "ufactory_xarm6_w_vacuum_gripper", 0.0 ,0.0 ,1.15)


    if 0:

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
        robot1_pedestal_spawner,
        bin_spawner,
        computer_mobile_spawner,
        assembly_station_spawner,
        ufactory_xarm6_w_vacuum_gripper_spawner,
    ])

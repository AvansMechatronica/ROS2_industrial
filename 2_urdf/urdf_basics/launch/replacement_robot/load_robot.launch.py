from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('urdf_basics'),
        'urdf',
        'robot',
        'lrmate200ic_environment.urdf.xacro'
    )

    return LaunchDescription([
        # Run xacro to generate the robot description parameter
        ExecuteProcess(
            cmd=['xacro', xacro_file],
            output='screen'
        ),
        # Robot State Publisher with the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file])
            }]
        ),
        #Node(
        #    package='robot_state_publisher',
        #    executable='robot_state_publisher',
        #    name='robot_state_publisher',
        #    parameters=[{
        #        'robot_description': Command(['xacro ', xacro_file])
        #    }]
        #),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Paths
    package_share_directory = get_package_share_directory('urdf_basics')
    xacro_file = os.path.join(package_share_directory, 'urdf', 'assignment1.urdf.xacro')
    rviz_config_file = os.path.join(package_share_directory, 'config', 'assignment1.rviz')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file])
            }]
        ),
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': True},
                        {'zeros.robot1_joint1': 0.0},
                        {'zeros.robot1_joint2': 0.785},
                        {'zeros.robot1_joint3': -1.57},
                        {'zeros.robot1_joint4': 0.0},
                        {'zeros.robot1_joint5': 0.785},
                        {'zeros.robot1_joint6': 0.0},
                        {'zeros.robot2_joint1': 0.0},
                        {'zeros.robot2_joint2': 0.785},
                        {'zeros.robot2_joint3': -1.57},
                        {'zeros.robot2_joint4': 0.0},
                        {'zeros.robot2_joint5': 0.785},
                        {'zeros.robot2_joint6': 0.0},
            
            ],
        ),
       # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])

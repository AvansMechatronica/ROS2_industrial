from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to xacro, SRDF, and RViz configuration files
    ros_industrial_support_dir = get_package_share_directory('ros_industrial_support')
    hrwros_moveit_config_dir = get_package_share_directory('hrwros_moveit_config')
    urdf_file = os.path.join(ros_industrial_support_dir, 'urdf', 'hrwros.xacro')
    srdf_file = os.path.join(hrwros_moveit_config_dir, 'config', 'hrwros.srdf')
    rviz_config_file = os.path.join(ros_industrial_support_dir, 'config', 'hrwros.rviz')

    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui', 
        default_value='true', 
        description='Use GUI for Joint State Publisher'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'robot_description_semantic': Command(['xacro ', srdf_file]),
        }]
    )

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': LaunchConfiguration('gui')}],
        ros_parameters={
            'zeros': {
                'robot1_shoulder_pan_joint': 0.6207787083493841,
                'robot1_shoulder_lift_joint': -1.004681330618082,
                'robot1_elbow_joint': 1.6983449885307538,
                'robot1_wrist_1_joint': -2.301530778020034,
                'robot1_wrist_2_joint': -1.625460038967466,
                'robot1_wrist_3_joint': 0.0,
                'robot2_shoulder_pan_joint': 1.5707963267950005,
                'robot2_shoulder_lift_joint': -1.5525750894041779,
                'robot2_elbow_joint': 1.5525750894041783,
                'robot2_wrist_1_joint': -1.570796326795,
                'robot2_wrist_2_joint': -1.534353852013356,
                'robot2_wrist_3_joint': 0.0,
            }
        }
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        gui_arg,
        
        # Add nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])

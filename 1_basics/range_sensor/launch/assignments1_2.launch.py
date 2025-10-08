from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition   

def generate_launch_description():
    # Declare the `sim` argument with a default value of `false`
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='true', # Shuold be false
        description='Simulation mode'
    )
    sim = LaunchConfiguration('sim')

    # Define the sensor_info_publisher_simulation node
    sensor_info_publisher_node = Node(
        package='range_sensor',
        executable='sensor_info_publisher_simulation',
        name='sensor_info_publisher_simulation',
        output='screen',
        condition=IfCondition(sim),
    )
        
    # Define the BoxHeightInformation publisher node (Assignment 1)
    box_height_metres_node = Node(
        package='range_sensor',
        executable='assignment1',
        name='box_height_metres',
        output='screen'
    )

    # Define the metres_to_feet service server node
    metres_to_feet_node = Node(
        package='range_sensor',
        executable='metres_to_inches_server',
        name='metres_to_inches',
        output='screen'
    )

      # Define the BoxHeightInformation subscriber / convert to inches node (Assignment 2)
    box_height_feet_node = Node(
        package='range_sensor',
        executable='assignment2',
        name='box_height_inches',
        output='screen'
    )

    # Combine all launch actions
    return LaunchDescription([
        sim_arg,
        sensor_info_publisher_node,
        box_height_metres_node,
        metres_to_feet_node,
        box_height_feet_node
    ])

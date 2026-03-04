from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        name='sensor_info_publisher',
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

    # Todo 1: Define the metres_to_inches service server node


    # Todo 2: Define the BoxHeightInformation subscriber / convert to inches node (Assignment 2)


    # Combine all launch actions
    return LaunchDescription([
        sim_arg,
        sensor_info_publisher_node,
        box_height_metres_node,
        # Todo 1: Add the metres_to_inches service server node to the launch description

        # Todo 2: Add the BoxHeightInformation subscriber / convert to inches node to the launch description

    ])

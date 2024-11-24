from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_navigation_dir = get_package_share_directory('turtlebot3_navigation')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', 
        default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger'), 
        description='Model type [burger, waffle, waffle_pi]'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file', 
        default_value=os.path.join(turtlebot3_navigation_dir, 'maps', 'map.yaml'), 
        description='Path to the map file'
    )
    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', 
        default_value='true', 
        description='Whether to open RViz'
    )
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', 
        default_value='0.0', 
        description='Initial pose X'
    )
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', 
        default_value='0.0', 
        description='Initial pose Y'
    )
    move_forward_only_arg = DeclareLaunchArgument(
        'move_forward_only', 
        default_value='false', 
        description='Restrict navigation to forward motion only'
    )

    # Include TurtleBot3 Bringup
    turtlebot3_remote_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_dir, 'launch', 'turtlebot3_remote.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model')
        }.items()
    )

    # Map Server Node
    map_server_node = Node(
        package='map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )

    # Include AMCL Launch
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_navigation_dir, 'launch', 'amcl.launch.py')
        ),
        launch_arguments={
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
        }.items()
    )

    # Include Move Base Launch
    move_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_navigation_dir, 'launch', 'move_base.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'move_forward_only': LaunchConfiguration('move_forward_only'),
        }.items()
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            '-d', 
            os.path.join(turtlebot3_navigation_dir, 'rviz', 'turtlebot3_navigation.rviz')
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    return LaunchDescription([
        # Declare arguments
        model_arg,
        map_file_arg,
        open_rviz_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        move_forward_only_arg,
        
        # Include nodes and launches
        turtlebot3_remote_launch,
        map_server_node,
        amcl_launch,
        move_base_launch,
        rviz_node
    ])

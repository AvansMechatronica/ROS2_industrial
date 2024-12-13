from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def setup_gzserver_launch(context, *args, **kwargs):
    # Resolve the value of 'without_obstacles' at runtime
    without_obstacles = LaunchConfiguration('without_obstacles').perform(context)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    navigation_package_share_directory = get_package_share_directory('navigation')

    if without_obstacles == 'True':  # Compare the string value
        world_file = PathJoinSubstitution([navigation_package_share_directory, 'worlds', 'warehouse.sdf'])
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
                ),
                launch_arguments={'world': world_file}.items()
            )
        ]
    else:
        world_file = PathJoinSubstitution([navigation_package_share_directory, 'worlds', 'warehouse_with_obstacles.sdf'])
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
                ),
                launch_arguments={'world': world_file}.items()
            )
        ]


def generate_launch_description():

    selector_arg = DeclareLaunchArgument('without_obstacles', default_value='False',
                                       description='Choose True to load: warehouse_with_obstacles or False to load warehouse')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        selector_arg,
        OpaqueFunction(function=setup_gzserver_launch),
    	gzclient_cmd,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
    ])
    

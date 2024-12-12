from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import subprocess
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():

    # Declare launch arguments
    world_name = LaunchConfiguration('world_name')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='warehouse_with_obstacles',
                                       description='Choose the world to load: warehouse or warehouse_with_obstacles')

    #launch_arguments={'gz_args': '-r {}'.format(world_name)}.items()
    #print(launch_arguments)
    #print(world_name.parse('world_name'))

    navigation_package_share_directory = get_package_share_directory('navigation')
    #world_file = PathJoinSubstitution([navigation_package_share_directory, 'worlds', str(world_name) + '.sdf'])
    #print(world_file)

    world_file = PathJoinSubstitution([navigation_package_share_directory, 'worlds', 'warehouse.sdf'])
    world2 = PathJoinSubstitution([navigation_package_share_directory, 'worlds', 'warehouse_with_obstacles.sdf'])
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths to xacro and RViz config files
    xacro_file = os.path.join(navigation_package_share_directory, 'urdf', 'environmet.urdf.xacro')

    result = subprocess.run(['xacro', xacro_file], stdout=subprocess.PIPE, text=True)
    
    with open('tmp.urdf', "w") as urdf_file:
        urdf_file.write(result.stdout)
    

    # Robot State Publisher Node
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True,
       }]
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # gazebo spawn entity node
    environment_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-file', 'tmp.urdf',
            '-entity', 'environmet',
            '-timeout', str(50.0),
        ],
        parameters=[{'use_sim_time': True}],
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
        world_name_arg,
        gzserver_cmd,
    	gzclient_cmd,
        #environment_node,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
    ])
    

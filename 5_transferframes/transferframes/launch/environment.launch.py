from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('paused', default_value='true', description='Start Gazebo paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false', description='Enable RVIZ'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('extra_gazebo_args', default_value='--verbose', description='Extra arguments for Gazebo'),
    ]

    # Paths to included launch files
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    #casus_support_launch_dir = os.path.join(get_package_share_directory('casus_support'), 'launch')
    #casus_moveit_config_launch_dir = os.path.join(get_package_share_directory('casus_moveit_config'), 'launch')
    #ros_industrial_gazebo_launch_dir = os.path.join(get_package_share_directory('ros_industrial_gazebo'), 'launch')

    world_path =  os.path.join(get_package_share_directory('transferframes'), 'worlds', 'casus.world')
    print(world_path)

    # Include other launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'world': world_path,
            'server_required': 'true',
            'gui_required': 'true',
        }.items(),
    )
    package_dir = get_package_share_directory('transferframes')

    # Define the robot description using xacro command

    # Define the path to the URDF file within the package
    urdf_file_path = os.path.join(
        get_package_share_directory('transferframes'),
        'urdf',
        'environment.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_file_path])
    # Set robot description parameter
    
    urdf_description_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    
    spawn_world_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'support', 'spawn_static_world_objects.launch.py'))
    )



    if 0:

        move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(casus_moveit_config_launch_dir, 'move_group.launch.py'))
        )

        spawn_world_objects = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_industrial_gazebo_launch_dir, 'spawn_static_world_objects.launch.py'))
        )

        spawn_robots = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_industrial_gazebo_launch_dir, 'spawn_robots.launch.py'))
        )

        # Execute timed processes
        unpause_gazebo = ExecuteProcess(
            cmd=['timed_roslaunch.sh', '20', 'ros_industrial_gazebo', 'unpause.launch'],
            output='screen'
        )

        spawn_parts = ExecuteProcess(
            cmd=['timed_roslaunch.sh', '7', 'ros_industrial_gazebo', 'spawn_parts_transferframes.launch'],
            output='screen'
        )

        # Combine joint state information
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['/robot1/joint_states']}],
            remappings=[('/joint_states', '/combined_joint_states')]
        )



        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robots_state_publisher',
            parameters=[{'publish_frequency': 50.0}],
            remappings=[('/joint_states', '/combined_joint_states')]
        )

    # Define the path to the RViz configuration file within the package
    rviz_config_path = os.path.join(
        get_package_share_directory('transferframes'),
        'config',
        'environment.rviz'
    )

    # Start RVIZ if enabled
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint State Publisher GUI Node
    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True},
                        {'zeros.robot1_joint1': 0.0},
                        {'zeros.robot1_joint2': 0.785},
                        {'zeros.robot1_joint3': -1.57},
                        {'zeros.robot1_joint4': 0.0},
                        {'zeros.robot1_joint5': 0.785},
                        {'zeros.robot1_joint6': 0.0},         
            ],
        )



    return LaunchDescription(
        launch_args + [
            gazebo_launch,
            urdf_description_node,
            #move_group,
            #spawn_world_objects,
            #spawn_robots,
            #unpause_gazebo,
            #spawn_parts,
            joint_state_publisher,
            #rviz,
            robot_state_publisher
        ]
    )

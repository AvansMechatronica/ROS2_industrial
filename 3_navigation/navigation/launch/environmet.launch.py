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
    # Paths to xacro and RViz config files
    package_share_directory = get_package_share_directory('navigation')
    xacro_file = os.path.join(package_share_directory, 'urdf', 'environmet.urdf.xacro')
    rviz_config_file = os.path.join(package_share_directory, 'config', 'environment.rviz')

    result = subprocess.run(['xacro', xacro_file], stdout=subprocess.PIPE, text=True)
    
    with open('tmp.urdf', "w") as urdf_file:
        urdf_file.write(result.stdout)
    
    #print(result.stdout)

    # Robot State Publisher Node
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            	'use_sim_time': 'true',
       }]
    )
    # Joint State Publisher Node
    joint_state_node = Node(
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
    )
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
    )


    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )
    world = PathJoinSubstitution([package_share_directory, 'worlds', 'empty.world'])

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    if 0:
        gazebo_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
            launch_arguments={
                'world': world,
                'server_required': 'true',
                'gui_required': 'true',
            }.items(),
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

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')   
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
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


    return LaunchDescription([
        robot_state_node,
        joint_state_node,
        gzserver_cmd,
    	gzclient_cmd,
        
        #rviz_node,
        #gazebo_node,
        environment_node,
        spawn_turtlebot_cmd,
    ])

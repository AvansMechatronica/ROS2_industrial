import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

def launch_setup(context, *args, **kwargs):
    # Declare launch arguments

    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.162')
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='xarm_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
   
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'gazebo_ros2_control/GazeboSystem'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('my_lite6_moveit_config'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    pkg_path = os.path.join(get_package_share_directory('my_lite6_moveit_config'))
    urdf_file = os.path.join(pkg_path, 'config', 'lite6_robot.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'config', 'lite6_robot.srdf')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            robot_ip=robot_ip,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
        )
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controllers_file)
        .planning_pipelines(config_folder=pipeline_filedir)
        .to_moveit_configs()
    )
    
    if 0:
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
    casus_moveit_config_launch_dir = os.path.join(get_package_share_directory('transferframes_moveit_config'), 'launch')

    world_path =  os.path.join(get_package_share_directory('transferframes'), 'worlds', 'casus.world')

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

    if 0:
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

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(casus_moveit_config_launch_dir, 'move_group.launch.py'))
    )


    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        #parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    if 0:
        ros2_control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
            launch_arguments={
                'robot_description': yaml.dump(moveit_config.robot_description),
                'ros2_control_params': ros2_control_params,
            }.items(),
        )

    controllers = [
        'joint_state_broadcaster',
        '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
    ]

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
            parameters=[{'use_sim_time': True}],
        ))

    # Define the path to the RViz configuration file within the package
    rviz_config_path = os.path.join(
        get_package_share_directory('transferframes'),
        'config',
        'environment.rviz'
    )

    # Start RVIZ if enabled
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )


    if len(controller_nodes) > 0:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=spawn_world_objects,
                )
            ),
            RegisterEventHandler(
                #condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=spawn_world_objects,
                    on_exit=rviz2_node,
                )
            ),
            #RegisterEventHandler(
            #    event_handler=OnProcessExit(
            #        target_action=spawn_world_objects,
            #        on_exit=controller_nodes,
            #    )
            #),
            robot_state_publisher_node,
            #move_group
        ]
    else:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=spawn_world_objects,
                )
            ),
            RegisterEventHandler(
                #condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=spawn_world_objects,
                    on_exit=rviz2_node,
                )
            ),
            robot_state_publisher_node,
            #move_group
        ]



def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
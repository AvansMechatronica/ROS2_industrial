#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction
#from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
#from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


    
def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gz_ros2_control/GazeboSimSystem')
    
    attach_to = LaunchConfiguration('attach_to', default='xarm_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    load_controller = LaunchConfiguration('load_controller', default=True)
    show_rviz = LaunchConfiguration('show_rviz', default=True)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    pkg_path = os.path.join(get_package_share_directory('manipulation_moveit_config'))

    if 0:
        ros2_control_params = generate_ros2_control_params_temp_file(
            os.path.join(pkg_path, 'config', 'ros2_controllers.yaml'),
            prefix=prefix.perform(context),
            add_gripper=add_gripper.perform(context) in ('True', 'true'),
            add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
            ros_namespace=ros_namespace,
            update_rate=1000,
            use_sim_time=True,
            robot_type=robot_type.perform(context)
        )


    urdf_file = os.path.join(pkg_path, 'config', 'manipuation_environment.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'config', 'manipuation_environment.srdf')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    moveit_config = MoveItConfigsBuilder("manipuation_environment", package_name="manipulation_moveit_config").to_moveit_configs()
    move_group_node = generate_move_group_launch(moveit_config)

    if 0:
        moveit_config = (
            MoveItConfigsBuilder(
                context=context,
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
    moveit_config_dump = yaml.dump(moveit_config.to_dict())
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}

    if 0:
        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config_dict,
                {'use_sim_time': use_sim_time},
            ],
        )

 


    robot_description = {'robot_description': moveit_config_dict['robot_description']}
    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # gazebo launch
    # ignition gazebo launch
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('manipulation'), 'worlds', 'casus.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ' -r -v 3 {}'.format(xarm_gazebo_world.perform(context)),
        }.items(),
    )

    # ignition gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-name', 'xarm',
            '-topic', 'robot_description',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare('manipulation'), 'rviz', 'environment.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict.get('robot_description', ''),
                'robot_description_semantic': moveit_config_dict.get('robot_description_semantic', ''),
                'robot_description_kinematics': moveit_config_dict.get('robot_description_kinematics', {}),
                'robot_description_planning': moveit_config_dict.get('robot_description_planning', {}),
                'planning_pipelines': moveit_config_dict.get('planning_pipelines', {}),
                'use_sim_time': True
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
    ]
    if 0:
        if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true'):
            controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
        elif robot_type.perform(context) != 'lite' and add_bio_gripper.perform(context) in ('True', 'true'):
            controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
    
    controller_nodes = []
    if load_controller.perform(context) in ('True', 'true'):
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
                    on_start=gazebo_spawn_entity_node,
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=rviz2_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=controller_nodes,
                )
            ),
            robot_state_publisher_node,
            move_group_node,
            #robot_moveit_common_launch,
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
                    on_start=gazebo_spawn_entity_node,
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=rviz2_node,
                )
            ),
            robot_state_publisher_node,
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

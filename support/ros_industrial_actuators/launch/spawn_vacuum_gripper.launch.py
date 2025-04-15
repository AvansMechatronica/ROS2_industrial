from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():


    use_sim_time=LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('ros_industrial_actuators')
    vacuum_gripper_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        name='camera_spawner',
        arguments=[

            '-entity', 'vacuum_gripper',
            '-file', pkg_path+'/models/vacuum_gripper/model.sdf',
            '-timeout', '50'
        ],
    )


    vacuum_gripper_bridge = Node(package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='vacuum_gripper_bridge',
        output='screen',
        arguments=[
            'vacuum_gripper/status/attached' + '@std_msgs/msg/Int32' + '[gz.msgs.Int32',
            'vacuum_gripper/control/enable' + '@std_msgs/msg/Int32' + ']gz.msgs.Int32',
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        #remappings=[
        #    ('/vacuum_gripper_control_enable', '/ros_industrial/actuators/vacuum_gripper/control'),
        #    ('/vacuum_gripper_status_attached', '/ros_industrial/actuators/vacuum_gripper/status'),
        #]
    )


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(vacuum_gripper_node)
    ld.add_action(vacuum_gripper_bridge)
    return ld
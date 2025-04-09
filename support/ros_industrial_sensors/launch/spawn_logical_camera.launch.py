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

    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')
    R = LaunchConfiguration('R', default='0.0')
    P = LaunchConfiguration('P', default='0.0')
    Y = LaunchConfiguration('Y', default='0.0')
    use_sim_time=LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('ros_industrial_sensors')
    camera_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        name='camera_spawner',
        arguments=[
            '-x', x,
            '-y', y, 
            '-z', z,
            '-R', R,
            '-P', P,
            '-Y', Y,
            '-entity', 'logical_camera_1',
            '-file', pkg_path+'/models/logical_camera/model.sdf',
            '-timeout', '50'
        ],
    )

    logical_camera_bridge = Node(package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='logical_camera_bridge',
        output='screen',
        arguments=[
            #'/ros_industrial/sensors/custom_logical_camera/image' + '@rosgraph_msgs/msg/my_logical_camera_topic' + '[gz.msgs.LogicalCameraImage',
            'custom_logical_camera_objects' + '@ros_gz_interfaces/msg/LogicalCameraImage' + '[gz.msgs.LogicalCameraImage'
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/custom_logical_camera_objects', '/ros_industrial/sensors/custom_logical_camera/objects'),
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(camera_node)
    ld.add_action(logical_camera_bridge)
    return ld
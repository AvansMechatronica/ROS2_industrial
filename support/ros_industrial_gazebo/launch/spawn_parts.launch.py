from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('ros_industrial_gazebo')
    print(pkg_path)
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='gear_part_0_spawner',
            output='screen',
            arguments=[
                '-x', '0.5', '-y', '-0.8', '-z', '1.5', 
                '-entity', 'battery_0',
                '-file', pkg_path+'/models/battery/model.sdf'
            ],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='gear_part_0_spawner',
            output='screen',
            arguments=[
                '-x', '0.5', '-y', '-0.6', '-z', '1.5', 
                '-entity', 'regulator_0',
                '-file', pkg_path+'/models/regulator/model.sdf'
            ],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='gear_part_0_spawner',
            output='screen',
            arguments=[
                '-x', '0.3', '-y', '-0.8', '-z', '1.5', 
                '-entity', 'pump_0',
                '-file', pkg_path+'/models/pump/model.sdf'
            ],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='gear_part_0_spawner',
            output='screen',
            arguments=[
                '-x', '0.3', '-y', '-0.6', '-z', '1.5', 
                '-entity', 'sensor_0',
                '-file', pkg_path+'/models/sensor/model.sdf'
            ],
        ),
    ])

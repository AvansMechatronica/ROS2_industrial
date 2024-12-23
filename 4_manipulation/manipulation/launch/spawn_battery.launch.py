from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import random
import math

def generate_launch_description():
    pkg_path = get_package_share_directory('ros_industrial_gazebo')
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='battery_part_0_spawner',
            output='screen',
            arguments=[
                '-x', '0.4', '-y', '-0.4', '-z', '1.5', '-P', str(math.radians(180)),
                '-entity', 'battery_' + str(random.randint(0, 1000)),
                '-file', pkg_path+'/models/battery/model.sdf'
            ],
        ),
    ])

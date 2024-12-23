from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import random
import math

x_width = 0.5
x_base = 0.1
x_offset = random.random() * x_width
y_width = 0.5
y_base = -0.5
y_offset = random.random() * y_width
x_pos = x_base + x_offset
y_pos = y_base + y_offset


def generate_launch_description():
    parts = ["battery", "regulator", "pump", "sensor"]
    pkg_path = get_package_share_directory('ros_industrial_gazebo')
    part = parts[random.randint(0,3)]
    entity = part + "_" + str(random.randint(0, 1000))
    file = pkg_path +'/models/' + part + '/model.sdf'

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='part_0_spawner',
            output='screen',
            arguments=[
                '-x', str(x_pos), '-y', str(y_pos), '-z', '1.5', '-P', str(math.radians(180)),
                '-entity', entity,
                '-file', file
            ],
        ),
    ])

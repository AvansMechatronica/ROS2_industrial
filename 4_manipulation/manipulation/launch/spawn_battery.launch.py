from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory
import random
import math

def generate_launch_description():
    pkg_path = get_package_share_directory('ros_industrial_gazebo')
    model_path = pkg_path + '/models/battery/model.sdf'

    entity_name = 'battery_' + str(random.randint(0, 1000))
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ign', 'service', '-s', '/world/default/create',
                '--reqtype', 'ignition.msgs.EntityFactory',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'name: "{entity_name}" sdf_filename: "{model_path}" pose: {{position: {{x: 0.4, y: -0.4, z: 1.5}}, orientation: {{w: {math.cos(math.radians(180) / 2)}, x: 0, y: 0, z: {math.sin(math.radians(180) / 2)}}}}}'
            ],
            output='screen'
        )
    ])

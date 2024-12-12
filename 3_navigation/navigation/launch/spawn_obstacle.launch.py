from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import random
import math

def spawn_object(object, x, y , p):
    pkg_path = get_package_share_directory('ros_industrial_gazebo')
    file = pkg_path+'/models/' + object + '/model.sdf'
    entity = object + '_' + str(random.randint(0, 1000))

    print(file)
    print(entity)


    return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=entity,
            output='screen',
            arguments=[
                '-x', str(x), '-y', str(y), '-z', '0', '-P', str(p),
                '-entity', entity,
                '-file', file
            ],
        )

def generate_launch_description():
    
    chair1 = spawn_object('chair', 14.3, -5.5, 3)
    chair2 = spawn_object('chair', 14.3, -4, -3)
    chair3 = spawn_object('foldable_chair', -11.6, 6.4, -1.8)
    chair4 = spawn_object('foldable_chair', -14, 6.5, 1.9)
    table = spawn_object('table', -12.6, 6.5, 1.9)
    #malevisitoronphone = spawn_object('malevisitoronphone', 14.64, -10, -1.57)
    #casual_female = spawn_object('casual_female', 14.64, -10, -1.57)

    return LaunchDescription([
        chair1,
        chair2,
        chair3,
        chair4,
        table,
        #malevisitoronphone,
        #casual_female,

    ])

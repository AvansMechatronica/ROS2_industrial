from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import random
import math

def spawn_object(object_name, x, y , z, p, html_link=''):

    if len(html_link):
        file = html_link
    else:
        pkg_path = get_package_share_directory('ros_industrial_support')
        file = pkg_path+'/models/' + object_name + '/model.sdf'

    entity = object_name + '_' + str(random.randint(0, 1000))

    print(file)
    print(entity)
    
    # ignition gazebo spawn entity node
    return Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
                '-x', str(x), '-y', str(y), '-z', str(z), '-P', str(p),
                '-name', entity,
                '-file', file
        ],
    )


def generate_launch_description():
    
    box1 = spawn_object('box', 1, 1, 0.0, 0, 'https://fuel.gazebosim.org/1.0/paserv/models/rigid%20cubic%20cardbox%20medium%20size')
    #chair1 = spawn_object('chair', 0.5, 1.5, 0.0, 0)
    #chair2 = spawn_object('chair', 14.3, -4, 0, -3)
    #chair3 = spawn_object('foldable_chair', -11.5, 6.4, 0, -1.8)
    #chair4 = spawn_object('foldable_chair', -14, 6.5, 0, 1.9)
    #table = spawn_object('table', -12.6, 6.5, 0, 1.9)
    #malevisitoronphone = spawn_object('malevisitoronphone', 14.64, -10, 0, -1.57)
    #casual_female = spawn_object('casual_female', 14.64, -10, 0, -1.57)

    return LaunchDescription([
        box1,
        #chair1,
        #chair2,
        #chair3,
        #chair4,
        #table,
        #malevisitoronphone,
        #casual_female,

    ])

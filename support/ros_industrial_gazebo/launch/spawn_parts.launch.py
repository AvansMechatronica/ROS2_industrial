from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('ros_industrial_gazebo')
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='gear_part_0_spawner',
            output='screen',
            arguments=[
                '-x', '-3.2', '-y', '-0.7', '-z', '1.5', 
                '-entity', 'gear_part_0',
                '-file', pkg_path+'/meshes/part/gear_part/model.sdf'
            ],
        ),
        if 0:
          Node(
              package='gazebo_ros',
              executable='spawn_entity.py',
              name='gasket_part_0_spawner',
              output='screen',
              arguments=[
                  '-x', '-3.2', '-y', '-0.3', '-z', '1.5', 
                  '-entity', 'gasket_part_0',
                  '-file', pkg_path+'/meshes/part/gasket_part/model.sdf'
              ],
          ),
          Node(
              package='gazebo_ros',
              executable='spawn_entity.py',
              name='pulley_part_0_spawner',
              output='screen',
              arguments=[
                  '-x', '-2.8', '-y', '-0.7', '-z', '1.5',
                  '-entity', 'pulley_part_0',
                  '-file', pkg_path+'/meshes/part/pulley_part/model.sdf'
              ],
          ),
          Node(
              package='gazebo_ros',
              executable='spawn_entity.py',
              name='piston_rod_part_0_spawner',
              output='screen',
              arguments=[
                  '-x', '-2.8', '-y', '-0.3', '-z', '1.5', 
                  '-entity', 'piston_rod_part_0',
                  '-file', pkg_path+'/meshes/part/piston_rod_part/model.sdf'
              ],
          ),
    ])

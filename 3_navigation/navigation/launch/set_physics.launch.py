from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gz', 'service',
                '-s', '/world/maze/set_physics',
                '--reqtype', 'gz.msgs.Physics',
                '--reptype', 'gz.msgs.Boolean',
                '--req',
                'max_step_size: 0.01, real_time_update_rate: 1000'
            ],
            output='screen'
        )
    ])
# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

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

    logical_camera_bridge = Node(package='ros_gz_bridge', 
                        executable='parameter_bridge',
                        name='logical_camera_bridge',
                        output='screen',
                        arguments=[
                            #'/ros_industrial/sensors/custom_logical_camera/image' + '@rosgraph_msgs/msg/my_logical_camera_topic' + '[gz.msgs.LogicalCameraImage',
                            '/ros_industrial/sensors/custom_logical_camera/image' + '@my_logical_camera_topic' + '[gz.msgs.LogicalCameraImage'
                        ],
                        parameters=[{'use_sim_time': use_sim_time}],
)



#/ros_industrial/sensors/custom_logical_camera/image


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(logical_camera_bridge)
    return ld
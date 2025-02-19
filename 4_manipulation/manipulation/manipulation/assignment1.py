#!/usr/bin/env python3

# Naam Student:
# Studentnummer:
# Datum:

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

#from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from my_moveit_python import srdfGroupStates
from my_moveit_python import MovegroupHelper

prefix = ''
joint_names = [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]
base_link_name = "link_base"
end_effector_name = "link_eef"
group_name = "xarm6"
package_name = 'manipulation_moveit_config'
srdf_file_name = 'config/manipuation_environment.srdf'

joint_states = ['left', 'right', 'home']

class Assignment(Node):
    def __init__(self, node):
        super().__init__('Assignment')

        # Create node for this example
        self.node = node

        self.node.tf_buffer = Buffer()
        self.node.tf_listener = TransformListener(self.node.tf_buffer, node)

        self.lite6_groupstates = srdfGroupStates(package_name, srdf_file_name, group_name)
        self.move_group_helper = MovegroupHelper(self.node, joint_names, base_link_name, end_effector_name, group_name)
    
    def execute(self):
        for joint_state in joint_states:
            # Move to joint configuration
            result, joint_values = self.lite6_groupstates.get_joint_values(joint_state)
            if result:
                print("Move to " + joint_state)
                self.move_group_helper.move_to_configuration(joint_values)
            else:
                print( "Failed to get joint_values of " + joint_state)


            print("Move to fixed pose")
            translation = [0.5, 0.2, 0.25]
            rotation = [1.0, 0.0, 0.0, 0.0]
            self.move_group_helper.move_to_pose(translation, rotation)

        pass

def main():
    rclpy.init()
    # Create node for this example
    node = Node("assignment1")

    assignment = Assignment(node) # Note must be placed bevore creating executer

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()
    
    assignment.execute()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
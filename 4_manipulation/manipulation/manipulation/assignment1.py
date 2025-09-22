#!/usr/bin/env python3

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

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

    assignment = Assignment(node) # Note must be placed before creating executer


    # Create a MultiThreadedExecutor that can use up to 2 threads
    # to process callbacks (e.g., subscriptions, timers, services).
    executor = rclpy.executors.MultiThreadedExecutor(2)

    # Register the node with the executor so its callbacks can be scheduled.
    executor.add_node(node)

    # Create a separate background thread that will run the executor's spin loop.
    # This allows ROS callbacks to be handled without blocking the main thread.
    # Setting daemon=True ensures this thread will automatically stop when the main program exits.
    executor_thread = Thread(target=executor.spin, daemon=True, args=())

    # Start the executor thread so it begins processing callbacks in parallel.
    executor_thread.start()

    # Create a Rate object set to 1 Hz (once per second).
    # This call blocks the main thread for ~1 second before continuing.
    # Typically used in a loop to control the frequency of main-thread tasks.
    node.create_rate(1.0).sleep()

    assignment.execute()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
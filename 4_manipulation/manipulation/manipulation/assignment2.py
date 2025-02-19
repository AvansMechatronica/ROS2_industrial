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
from rclpy.executors import MultiThreadedExecutor
import time
from std_srvs.srv import SetBool


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
end_effector_name = "vacuum_gripper1_suction_cup"
group_name = "xarm6"
package_name = 'transferframes_moveit_config'
srdf_file_name = 'config/manipuation_environment.srdf'


joint_states = ['left', 'right', 'home']


class VacuumGripper(Node):
    def __init__(self):
        super().__init__('vacuum_gripper')
        self.client_name = '/gripper/custom_switch'
        self.client = self.create_client(SetBool, self.client_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service is available.')

    def send_request(self, on_state):
        request = SetBool.Request()
        request.data = on_state
        self.future = self.client.call_async(request)
        return self.future
    
    def pull(self):
        self.send_request(True)
    def release(self):
        self.send_request(False)


class PickAndDrop(Node):
    def __init__(self, node):
        super().__init__('PickAndDrop')

        # Create node for this example
        self.node = node

        self.vacuum_gripper = VacuumGripper()
        self.vacuum_gripper.release()

        # Initialize SRDF and MoveGroup helper
        self.lite6_groupstates = srdfGroupStates(package_name, srdf_file_name, group_name)
        self.move_group_helper = MovegroupHelper(self.node, joint_names, base_link_name, end_effector_name, group_name)


    def execute(self):

        # Move to joint configuration
        result, joint_values = self.lite6_groupstates.get_joint_values('home')
        if result:
            self.node.get_logger().info("Move to " + 'home')
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            self.node.get_logger().error( "Failed to get joint_values of " + 'home')

        #self.node.get_logger().info("Move to published fransfer frame")
        ## goto pre-grasp
        self.move_to_object(0.015)
        ## goto grasp
        self.move_to_object(0.0)
        time.sleep(1.0)
        ## gripper enable
        self.vacuum_gripper.pull()
        time.sleep(1.0)
        ## goto post-grasp
        self.move_to_object(0.015)
    
        # Move to joint configuration
        result, joint_values = self.lite6_groupstates.get_joint_values('home')
        if result:
            self.node.get_logger().info("Move to " + 'home')
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            self.node.get_logger().error( "Failed to get joint_values of " + 'home')

        # Move to joint configuration
        result, joint_values = self.lite6_groupstates.get_joint_values('drop')
        if result:
            self.node.get_logger().info("Move to " + 'drop')
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            self.node.get_logger().error( "Failed to get joint_values of " + 'drop')

        ## gripper release
        self.vacuum_gripper.release()

        # Move to joint configuration
        result, joint_values = self.lite6_groupstates.get_joint_values('home')
        if result:
            self.node.get_logger().info("Move to " + 'home')
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            self.node.get_logger().error( "Failed to get joint_values of " + 'home')

        result, joint_values = self.lite6_groupstates.get_joint_values('resting')
        if result:
            self.node.get_logger().info("Move to " + 'resting')
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            self.node.get_logger().error( "Failed to get joint_values of " + 'resting')

    def move_to_object(self, z_offset = 0.0):
        translation = [0.0, 0.0, 0.0]
        rotation= [0.0, 0.0, 0.0, 0.0]

        translation[0] = 0.4
        translation[1] = -0.4
        translation[2] = 0.1 + z_offset
        rotation[0] = 1.0
        rotation[1] = 0.0
        rotation[2] = 0.0
        rotation[3] = 0.0
        self.move_group_helper.move_to_pose(translation, rotation)
        
    def __del__(self):
        # Safe cleanup of executor and thread
        #self.camera.destroy_node()
        pass

# Define the main entry point
def main(args=None):
    rclpy.init(args=args)

    # Create the ROS 2 node
    node = Node("assignment1")

    # Instantiate the PickAndDrop class and execute
    pick_and_drop = PickAndDrop(node) # Note must be placed bevore creating executer


    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    pick_and_drop.execute()

    pick_and_drop.destroy_node()
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()

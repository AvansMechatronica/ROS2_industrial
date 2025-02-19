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
from ros_industrial_sensors.custom_logical_camera import Camera
from rclpy.executors import MultiThreadedExecutor
import time
from ros_industrial_actuators import VacuumGripper

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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

joint_states = ['left', 'right', 'home']

class PickAndDrop(Node):
    def __init__(self, node):
        super().__init__('PickAndDrop')

        # Create node for this example
        self.node = node

        self.vacuum_gripper = VacuumGripper("/gripper")
        self.vacuum_gripper.release()

        # Initialize the TransformListener and buffer
        self.node.tf_buffer = Buffer()
        self.node.tf_listener = TransformListener(self.node.tf_buffer, self.node)
        # Initialize SRDF and MoveGroup helper
        self.lite6_groupstates = srdfGroupStates(package_name, srdf_file_name, group_name)
        self.move_group_helper = MovegroupHelper(self.node, joint_names, base_link_name, end_effector_name, group_name)
        # Initialize the camera
        self.camera = Camera()

    def execute(self):

        # Move to joint configuration
        result, joint_values = self.lite6_groupstates.get_joint_values('home')
        if result:
            self.node.get_logger().info("Move to " + 'home')
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            self.node.get_logger().error( "Failed to get joint_values of " + 'home')


        result, photo = self.camera.take_photo()
        if not result:
            self.node.get_logger().info(f'No parts found')
            return
        
        parts = photo['parts']
        #camera_frame = ['camera_frame']
        #self.node.get_logger().info("Parts detected: {parts}")

        
        for part in parts:
            self.node.get_logger().info(f'Handeling: {part}')

            #self.node.get_logger().info("Move to published fransfer frame")
            ## goto pre-grasp
            self.move_to_object(part, 0.015)
            ## goto grasp
            self.move_to_object(part)
            time.sleep(1.0)
            ## gripper enable
            self.vacuum_gripper.pull()
            time.sleep(1.0)
            #self.gripper_release() 
            ## goto post-grasp
            self.move_to_object(part, 0.015)
        

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

    def move_to_object(self, part, z_offset = 0.0):
        to_frame_rel = 'base_link'
        from_frame_rel = part
        try:
            t = self.node.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            #node.get_logger().info(t)
            translation = [0.0, 0.0, 0.0]
            rotation = [0.0, 0.0, 0.0, 0.0]

            translation[0] = t.transform.translation.x
            translation[1] = t.transform.translation.y
            translation[2] = t.transform.translation.z + z_offset
            rotation[0] = t.transform.rotation.w
            rotation[1] = t.transform.rotation.x
            rotation[2] = t.transform.rotation.y
            rotation[3] = t.transform.rotation.z
            self.move_group_helper.move_to_pose(translation, rotation)
        
        except TransformException as ex:
            self.node.get_logger().error(
                f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')

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

    if 0:
        try:
            executor = MultiThreadedExecutor(num_threads=2)
            if executor is None:
                node.get_logger().error("Failed to initialize MultiThreadedExecutor: executor is None!")
            else:
                executor.add_node(node)  # Add node to executor
                node.get_logger().info("Executor initialized successfully.")
        except Exception as e:
            node.get_logger().error(f"Error initializing executor: {str(e)}")

        # Debugging: Check if executor exists before proceeding
        if executor:
            try:
                # Spin the executor in a background thread
                executor_thread = Thread(target=executor.spin, daemon=True)
                executor_thread.start()
                node.get_logger().info("Executor thread started.")
            except Exception as e:
                node.get_logger().error(f"Error starting executor thread: {str(e)}")
        else:
            node.get_logger().error("Executor is None, cannot start thread.")

        # Wait for initialization
        node.create_rate(1.0).sleep()
    

    pick_and_drop.execute()

    pick_and_drop.destroy_node()
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()

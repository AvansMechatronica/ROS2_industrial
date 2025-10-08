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
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PickAndDrop(Node):

    def __init__(self):
        super().__init__("PickAndDrop")
        # Robot parameters
        prefix = ""
        self.joint_names = [
            prefix + "joint1",
            prefix + "joint2",
            prefix + "joint3",
            prefix + "joint4",
            prefix + "joint5",
            prefix + "joint6",
        ]
        self.base_link_name = "link_base"
        self.end_effector_name = "link_eef"
        self.group_name = "xarm6"
        self.package_name = "manipulation_moveit_config"
        self.srdf_file_name = "config/manipuation_environment.srdf"

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt helpers
        self.group_states = srdfGroupStates(
            self.package_name, self.srdf_file_name, self.group_name
        )
        self.move_group = MovegroupHelper(
            self, self.joint_names, self.base_link_name, self.end_effector_name, self.group_name
        )

        self.vacuum_gripper = VacuumGripper()
        self.vacuum_gripper.release()

        # Initialize the camera
        self.camera = Camera()

        # --- Create subscribers, publishers, clients, timers here ---

        self.get_logger().info("PickAndDrop node has been initialized.")

    # --- Create callback functions here ---

    # --- Motion primitives ------------------------------------------------
    def move_to_state(self, state_name: str):
        result, joint_values = self.group_states.get_joint_values(state_name)
        if not result:
            self.get_logger().error(f"Failed to get joint values for state '{state_name}'.")
        self.get_logger().info(f"Moving to state '{state_name}'.")
        self.move_group.move_to_configuration(joint_values)

    def move_to_pose(self, translation, rotation):
        self.get_logger().info(f"Moving to pose: {translation}, {rotation}")
        self.move_group.move_to_pose(translation, rotation)

    def move_to_tf(self, from_frame: str, to_frame: str):
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time()
            )
            translation = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
            rotation = [
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
            ]
            self.get_logger().info(f"Moving to transform: {from_frame} → {to_frame}")
            self.move_to_pose(translation, rotation)
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {to_frame} to {from_frame}: {ex}")

    def move_to_object(self, part, z_offset = 0.0):
        to_frame_rel = 'base_link'
        from_frame_rel = part
        try:
            t = self.tf_buffer.lookup_transform(
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
            self.move_to_pose(translation, rotation)
        
        except TransformException as ex:
            self.node.get_logger().error(
                f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')

    # --- App sequence ----------------------------------------------------

    def execute_app(self):

        self.move_to_state('home')
        # Move to joint configuration
        self.get_logger().info("Move to home")


        result, photo = self.camera.take_photo()
        if not result:
            self.get_logger().info(f'No parts found')
            return
        
        parts = photo['parts']
        parts_to_pick = ['pump', 'sensor', 'battery', 'regulator']
        #camera_frame = ['camera_frame']
        #self.get_logger().info("Parts detected: {parts}")

        
        for part in parts:
            if part in parts_to_pick:
                self.get_logger().info(f'Handeling: {part}')

                #self.get_logger().info("Move to published fransfer frame")
                ## goto pre-grasp
                self.move_to_object(part, 0.15)
                ## goto grasp
                self.move_to_object(part)
                time.sleep(1.0)
                ## gripper enable
                self.vacuum_gripper.pull()
                time.sleep(1.0)
                #self.gripper_release() 
                ## goto post-grasp
                self.move_to_object(part, 0.15)
            
                if 0:
                    self.move_to_state('home')
                    # Move to joint configuration
                    self.get_logger().info("Move to home")


                self.move_to_state('drop')
                # Move to joint configuration
                self.get_logger().info("Move to drop")

                ## gripper release
                self.vacuum_gripper.release()

                self.move_to_state('home')
                # Move to joint configuration
                self.get_logger().info("Move to home")

            self.move_to_state('resting')
            # Move to joint configuration
            self.get_logger().info("Move to resting")



    def __del__(self):
        # Safe cleanup of executor and thread
        #self.camera.destroy_node()
        pass

# Define the main entry point
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the PickAndDrop class and execute
    node = PickAndDrop() # Note must be placed bevore creating executer

    # Create a multithreaded executor with 2 threads
    # This allows the node to handle multiple callbacks concurrently (e.g., subscriptions, timers)
    executor = MultiThreadedExecutor(num_threads=2)

    # Add the node to the executor so it can process its callbacks
    executor.add_node(node)

    # Start the executor in a separate background thread
    # This keeps the ROS event loop (callback processing) running
    # while your main thread can still execute custom logic (like execute_app)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create a 1 Hz rate object and sleep once to allow initialization
    # Equivalent to "rclpy.spin_once(node)" but gives time for system setup (e.g., MoveIt, TF)
    node.create_rate(1.0).sleep()

    # Run your custom main logic (defined inside the Assignment class)
    # This typically executes the robot’s motion, computation, or control behavior
    node.execute_app()

    # Shutdown ROS gracefully once the main logic finishes
    rclpy.shutdown()

    # Wait for the executor thread to exit cleanly before terminating the program
    executor_thread.join()

if __name__ == '__main__':
    main()

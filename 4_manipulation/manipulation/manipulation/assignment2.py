#!/usr/bin/env python3

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

from threading import Thread

import rclpy
from rclpy.node import Node


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from my_moveit_python import srdfGroupStates
from my_moveit_python import MovegroupHelper
from rclpy.executors import MultiThreadedExecutor
import time
import tf_transformations
# TODO 2: Defineer hier de benodigde imports voor de gripper
from std_msgs.msg import Bool


class VacuumGripper(Node):
    def __init__(self):
        super().__init__('vacuum_gripper')

        # TODO 2: Plaats hier de topic naam van de gripper
        self.enable_topic_name =  '/vacuum_gripper/control/enable'

        # TODO 2: Maak hier de publisher voor de gripper aan
        self.enable_topic_publisher = self.create_publisher(Bool, self.enable_topic_name, 10)
   
    def pull(self):
        # TODO 2: Activeer de gripper door een topic te publiceren
        msg = Bool()
        msg.data = True
        self.enable_topic_publisher.publish(msg)

    def release(self):
        # TODO 2: Deactiveer de gripper door een topic te publiceren
        msg = Bool()
        msg.data = False
        self.enable_topic_publisher.publish(msg)



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

    def move_to_object(self, z_offset = 0.0):
        translation = [0.0, 0.0, 0.0]
        #rotation= [0.0, 0.0, 0.0, 0.0]
        # RPY angles in radians
        
        roll = 3.1415927
        pitch = 0.0
        yaw = 0.0
        # Convert RPY to quaternion
        rotation = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        translation[0] = 0.4
        translation[1] = -0.4
        translation[2] = 0.18 + z_offset
        rotation[0] = 1.0
        rotation[1] = 0.0
        rotation[2] = 0.0
        rotation[3] = 0.0
        self.move_to_pose(translation, rotation)

    # --- App sequence ----------------------------------------------------

    def execute_app(self):

        # TODO 1: Ga naar de home positie
        self.move_to_state('home')
        # Move to joint configuration
        self.get_logger().info("Move to home")

        #self.get_logger().info("Move to published fransfer frame")
        ## goto pre-grasp
        # TODO 1: Ga naar de pre-grasp positie boven het object
        self.move_to_object(0.03)
        ## goto grasp
        # TODO 1: Ga naar de grasp positie op het object
        self.move_to_object(0.0)
        # TODO: Wacht 1 seconde
        time.sleep(1.0)
        ## Activeer gripper
        self.vacuum_gripper.pull()
        time.sleep(1.0)

        ## goto post-grasp
        # TODO 1: Ga naar de pre-grasp positie boven het object
        self.move_to_object(0.1)
        
        #TODO 1: Ga naar de home positie
        self.move_to_state('home')
        # Move to joint configuration
        self.get_logger().info("Move to home")

        # TODO 1: Ga naar de drop positie
        self.move_to_state('drop')
        # Move to joint configuration
        self.get_logger().info("Move to drop")

        ## deactiveer gripper
        self.vacuum_gripper.release()

        # TODO 1: Ga naar de home positie
        self.move_to_state('home')
        # Move to joint configuration
        self.get_logger().info("Move to home")

        # TODO 1: Ga naar de resting positie
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

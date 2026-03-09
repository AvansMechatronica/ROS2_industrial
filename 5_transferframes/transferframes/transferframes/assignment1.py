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

class manipulatorController(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
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

        self.get_logger().info("manipulatorController node has been initialized.")

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


    def movo_to_part(self, part, z_offset = 0.0):
        pass
    # --- App sequence ----------------------------------------------------

    def execute_app(self):

        # TODO 2: Beweeg naar de 'home' positie


        # TODO 1: Maak een foto met de camera

        # TODO 1: Controleer of er onderdelen zijn gedetecteerd, zo niet verlaat dan deze functie
        
        # TODO 1: Druk de gedetecteerde onderdelen af in de logger

        parts = [] # Haal deze informatie uit de camera
        parts_to_pick = ['pump', 'sensor', 'battery', 'regulator']


         # inereer over alle gedetecteerde onderdelen
        for part in parts:
            # Controleer of het onderdeel in de lijst van te pakken onderdelen zit
            # Als het onderdeel in de lijst zit, pak het onderdeel en plaats
            if part in parts_to_pick:
                # TODO 2: Druk het onderdeel af dat wordt opgepakt in de logger

                # TODO 3: Berken de positie van het onderdeel met behulp van TF

                # TODO 4: Beweeg naar de 'transfer' positie

                # TODO 3: Beweeg naar het onderdeel

                # TODO 3: Berken de positie van het onderdeel met behulp van TF

                # TODO 3: Beweeg naar het onderdeel (pre-graps positie)

                # TODO 3: Beweeg naar het onderdeel (graps positie)

                # Wacht even
                time.sleep(1.0)
                ## gripper enable
                # TODO 4: Pak het onderdeel vast

                # TODO 3: Beweeg weg van het onderdeel (post-graps positie)

                # TODO 2: Beweeg naar de 'drop' positie

                ## gripper release
                # TODO 5: Laat het onderdeel los

                # TODO 2: Beweeg terug naar de 'home' positie

            # TODO 2: Beweeg naar de 'resting' positie

    def __del__(self):
        # Safe cleanup of executor and thread
        #self.camera.destroy_node()
        pass

# --------------------------------------------------------------------------
# Do not modify the main function unless necessary.
# -------------------------------------------------------------------------
# Define the main entry point
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the manipulatorController node.
    # NOTE: This must be done before creating the executor to ensure callbacks are registered correctly.
    node = manipulatorController("pick_and_drop")

    # Create a multithreaded executor with 2 threads.
    # Allows the node to handle multiple callbacks concurrently (e.g., subscriptions, timers).
    executor = MultiThreadedExecutor(num_threads=2)

    # Add the node to the executor so it can process its callbacks.
    executor.add_node(node)

    # Start the executor in a separate background thread.
    # Keeps the ROS event loop running while allowing the main thread to execute custom logic.
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create a 1 Hz rate object and sleep once to allow initialization.
    # Provides time for system setup (e.g., MoveIt, TF) before running main logic.
    node.create_rate(1.0).sleep()

    # Execute the main application logic defined in the node.
    # Typically runs robot motion, computations, or control behaviors.
    node.execute_app()

    # Shutdown ROS gracefully after main logic completes.
    rclpy.shutdown()

    # Wait for the executor thread to exit cleanly before terminating the program.
    executor_thread.join()

if __name__ == '__main__':
    main()

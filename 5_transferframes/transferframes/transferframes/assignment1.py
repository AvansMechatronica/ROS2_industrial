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

        # TODO 2: Beweeg naar de 'home' positie
        self.move_to_state('home')
        # Move to joint configuration
        self.get_logger().info("Move to home")

        # TODO 1: Maak een foto met de camera
        result, photo = self.camera.take_photo()

        # TODO 1: Controleer of er onderdelen zijn gedetecteerd, zo niet verlaat dan deze functie
        if not result:
            self.get_logger().info(f'No parts found')
            return
        
        # TODO 1: Druk de gedetecteerde onderdelen af in de logger

        parts = photo['parts']
        parts_to_pick = ['pump', 'sensor', 'battery', 'regulator']
        #camera_frame = ['camera_frame']
        #self.get_logger().info("Parts detected: {parts}")

         # inereer over alle gedetecteerde onderdelen
        for part in parts:
            # Controleer of het onderdeel in de lijst van te pakken onderdelen zit
            # Als het onderdeel in de lijst zit, pak het onderdeel en plaats
            if part in parts_to_pick:
                # TODO 2: Druk het onderdeel af dat wordt opgepakt in de logger
                self.get_logger().info(f'Handeling: {part}')

                # TODO 3: Berken de positie van het onderdeel met behulp van TF

                # TODO 4: Beweeg naar de 'transfer' positie

                # TODO 3: Beweeg naar het onderdeel

                if 1:
                    # Verbeterde versie
                    #self.get_logger().info("Move to published fransfer frame")
                    ## goto pre-grasp
                    self.move_to_object(part, 0.2)
                    ## goto grasp
                    self.move_to_object(part)
                    time.sleep(1.0)
                    ## gripper enable
                    self.vacuum_gripper.pull()
                    time.sleep(1.0)
                    #self.gripper_release() 
                    ## goto post-grasp
                    self.move_to_object(part, 0.2)
                    self.get_logger().info(f'Moved to and picked up {part}')
                else:

                    # TODO 3: Berken de positie van het onderdeel met behulp van TF
                    to_frame_rel = 'base_link'
                    from_frame_rel = part
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())
                    #node.get_logger().info(t)
                    translation = [0.0, 0.0, 0.0]
                    rotation = [0.0, 0.0, 0.0, 0.0]

                    translation[0] = t.transform.translation.x
                    translation[1] = t.transform.translation.y
                    translation[2] = t.transform.translation.z + 0.15
                    rotation[0] = t.transform.rotation.w
                    rotation[1] = t.transform.rotation.x
                    rotation[2] = t.transform.rotation.y
                    rotation[3] = t.transform.rotation.z

                    # TODO 3: Beweeg naar het onderdeel (pre-graps positie)
                    self.move_to_pose(translation, rotation)

                    # TODO 3: Beweeg naar het onderdeel (graps positie)
                    translation[2] = t.transform.translation.z - 0.15
                    self.move_to_pose(translation, rotation)

                    # Wacht even
                    time.sleep(1.0)
                    ## gripper enable
                    # TODO 4: Pak het onderdeel vast
                    self.vacuum_gripper.pull()

                    # TODO 3: Beweeg weg van het onderdeel (post-graps positie)
                    translation[2] = t.transform.translation.z + 0.15
                    self.move_to_pose(translation, rotation)
            
                if 0:
                    # TODO 2: Beweeg naar de 'home' positie
                    self.move_to_state('home')
                    # Move to joint configuration
                    self.get_logger().info("Move to home")

                # TODO 2: Beweeg naar de 'drop' positie
                self.move_to_state('drop')
                # Move to joint configuration
                self.get_logger().info("Move to drop")

                ## gripper release
                # TODO 5: Laat het onderdeel los
                self.vacuum_gripper.release()

                # TODO 2: Beweeg terug naar de 'home' positie
                self.move_to_state('home')
                # Move to joint configuration
                self.get_logger().info("Move to home")

            # TODO 2: Beweeg naar de 'resting' positie
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

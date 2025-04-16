#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class VacuumGripper(Node):
    def __init__(self):
        super().__init__('vacuum_gripper')

        self.enable_topic_name =  '/vacuum_gripper/control/enable'
        self.attached_topic_name =  '/vacuum_gripper/status/attached'
        self.enable_topic_publisher = self.create_publisher(Bool, self.enable_topic_name, 10)
   
        self.attached = False

        self.attached_topic_subscription = self.create_subscription(
            Bool,
            self.attached_topic_name,
            self.attached_topic_callback,
            10)
        self.attached_topic_subscription  # prevent unused variable warning

    def attached_topic_callback(self, msg):
        self.attached = msg.data

    def pull(self):
        msg = Bool()
        msg.data = True
        self.enable_topic_publisher.publish(msg)

    def release(self):
        msg = Bool()
        msg.data = False
        self.enable_topic_publisher.publish(msg)

    def is_attached(self):
        return self.attached

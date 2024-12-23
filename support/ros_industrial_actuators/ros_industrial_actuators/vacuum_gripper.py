#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class VacuumGripper(Node):
    def __init__(self, namespace):
        super().__init__('vacuum_gripper')
        self.client_name = namespace + '/custom_switch'
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

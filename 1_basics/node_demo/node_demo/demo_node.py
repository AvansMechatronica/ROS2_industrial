import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from sensor_msgs.msg import Range
from range_sensors_interfaces.msg import SensorInformation
import random

MAX_RANGE = 1.00
MIN_RANGE = 0.10

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i = self.i + 1
        self.get_logger().info('DemoNode Running: "%i"' % self.i)


def main(args=None):
    rclpy.init(args=args)

    demo_node = DemoNode()

    rclpy.spin(demo_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
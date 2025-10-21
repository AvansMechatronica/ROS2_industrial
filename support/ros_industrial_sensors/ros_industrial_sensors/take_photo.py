#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_logical_camera import Camera

class CameraApp:
    """An object-oriented wrapper around the Camera node."""

    def __init__(self, args=None):
        rclpy.init(args=args)
        self.camera = Camera()
        self.photo_data = None

    def capture_and_process(self):
        """Captures a photo and processes it."""
        result, photo = self.camera.take_photo()
        if result:
            self.photo_data = photo
            self._display_results()
        else:
            self.camera.get_logger().error("Failed to capture photo.")
        rclpy.spin_once(self.camera, timeout_sec=0.1)

    def _display_results(self):
        """Display parts detected in the photo."""
        parts = self.photo_data.get('parts', [])
        print("Parts detected:")
        for part in parts:
            print(f" - {part}")

    def shutdown(self):
        """Cleanly shut down the ROS node and client library."""
        self.camera.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Entry point for the program."""
    app = CameraApp(args=args)
    try:
        app.capture_and_process()
    finally:
        app.shutdown()


if __name__ == '__main__':
    main()


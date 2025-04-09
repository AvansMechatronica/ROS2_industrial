#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_logical_camera import Camera


# Define a subscriber node

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin
    camera = Camera()


    result, photo = camera.take_photo()
    if result:
        #print(photo)

        camera.destroy_node()
        parts = photo['parts']
        #camera_frame = ['camera_frame']
        print("Parts detected: ")
        print(parts)


    rclpy.shutdown()

if __name__ == '__main__':
    main()

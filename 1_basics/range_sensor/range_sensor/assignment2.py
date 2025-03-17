#! /usr/bin/env python
# In this assignment you will subscribe to the topic that
# publishes information on the box height in metres and use the metres_to_inches
# service to convert this height in metres to height in inches.

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

import rclpy
from rclpy.node import Node
from range_sensors_interfaces.msg import BoxHeightInformation
from range_sensors_interfaces.srv import ConvertMetresToInches

class Assignment2(Node):

    def __init__(self):
        super().__init__('assignment2_async')
        # Todo 1: Create a client for the service that converts metres to inches
        self.convert_meters_to_inches_client = self.create_client(ConvertMetresToInches, 'metres_to_inches')
        
        # Todo 2: Wait for the service to be available
        while not self.convert_meters_to_inches_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # Create a request object of the type ConvertMetresToInches.Request
        self.request = ConvertMetresToInches.Request()

        self.subscription = self.create_subscription(
            BoxHeightInformation,
            'box_height_info',
            self.box_height_callback,
            10)
        self.subscription  # prevent unused variable warning

    def send_request(self, metres):
        self.request.distance_metres = metres
        
        # Todo 3: Call the service to convert the height in metres to height in inches
        response = self.convert_meters_to_inches_client.call_async(self.request)

        # Test if the response is successful
        if response.success:
            # Todo 4: Print the height of the box in inches
            self.get_logger().info('I heard: "%s"' % response.Response)
            #self.get_logger().info("The hight of the box is %4.2f metres or %4.2f inches" %(metres, response.distance_inches)) 
        else:
            # Todo 5: Print an error message if the request value is invalid
            metres_to_inches_client.get_logger().info("Invalis request value")

    # Callback function that is called when a message is received on the box_height_info topic
    def box_height_callback(self, box_height):
        # Call the send_request function to convert the height in metres to height
        self.send_request(box_height.box_height)


def main():
    rclpy.init()

    # Create an instance of the Assignment2 class
    assignment2 = Assignment2()

    # Spin the node so the callback function is called.
    rclpy.spin(assignment2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    assignment2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

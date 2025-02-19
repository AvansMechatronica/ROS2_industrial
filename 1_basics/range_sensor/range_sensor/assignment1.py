# In this assignment you will subscribe to the topic that
# publishes sensor information. Then, you will transform the sensor reading from
# the reference frame of the sensor to compute the height of a box based on the
# illustration shown in the assignment document. Then, you will publish the box height
# on a new message type ONLY if the height of the box is more than 10cm.

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Range
from range_sensors_interfaces.msg import SensorInformation
#< Assignment 1.4, importeer hier de message type die je hebt aangemaakt voor de box hoogte>  


class BoxHeightCalculator(Node):

    def __init__(self):
        super().__init__('box_height_calculator')
        #<Assignment 1.4, Creëer hier de publisher voor het publiceren van de box hoogte>

        #< Assignment 1.1, creëer hier de subscriber op het topic /sensor_info >


    def sensor_info_callback(self, sensor_info):
        self.get_logger().info('I heard: "%s"' % sensor_info)
        box_distance = sensor_info.sensor_data.range
        
        #<Assignment 1.2, bereken hier de hoogte van de box>
        # Compute the height of the box.
        # Boxes that are detected to be shorter than 10cm are due to sensor noise.
        # Do not publish information about them.
 

def main(args=None):
    rclpy.init(args=args)

    box_height_calculator = BoxHeightCalculator()

    rclpy.spin(box_height_calculator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    box_height_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import StaticTransformBroadcaster
from math import pi
from scipy.spatial.transform import Rotation

# Define a subscriber node
class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Define the topic name and message type
        self.subscription = self.create_subscription(
            LogicalCameraImage,  # Replace with your actual custom array message type
            '/ros_industrial/sensors/custom_logical_camera/objects',       # Topic name
            self.listener_callback,    # Callback function
            qos_profile                         # QoS depth
        )
        self.subscription  # Prevent unused variable warning
        self.camera_objects = None

        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def listener_callback(self, msg):
        # Process the array of PartPoses
        self.camera_objects = msg
        #self.camera_objects = LogicalCameraImage()

        if 0:
            for i, part_pose in enumerate(msg.part_poses):
                self.get_logger().info(f'PartPose {i}: Frame ID = {part_pose.part.data}, Pose = {part_pose.pose}')
            
            # Process the sensor pose
            camera_frame = msg.camera_frame.data
            self.get_logger().info(f'Camera Frame {camera_frame}')

    def take_photo(self, timeout=5.0):
        """
        Take a snapshot of the current camera image with a timeout.
        Spins the node to process messages during the timeout period.
        
        Args:
            timeout (float): Maximum time to wait for an image in seconds.

        Returns:
            (bool, dict): Success status and photo details as a dictionary.
        """
        #self.get_logger().info(f"Waiting for an image for up to {timeout} seconds...")

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < timeout * 1e9:
            # Spin once with a short timeout to process incoming messages
            rclpy.spin_once(self, timeout_sec=0.1)


            # Check if a new image has been received
            if self.camera_objects:
                print('parts detected')
                logical_camera_frame_name = 'logical_camera_frame'

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id =  'world'
                t.child_frame_id = logical_camera_frame_name
                t.transform.translation.x = self.camera_objects.pose.position.x
                t.transform.translation.y = self.camera_objects.pose.position.y
                t.transform.translation.z = self.camera_objects.pose.position.z
                t.transform.rotation.x = self.camera_objects.pose.orientation.x
                t.transform.rotation.y = self.camera_objects.pose.orientation.y
                t.transform.rotation.z = self.camera_objects.pose.orientation.z
                t.transform.rotation.w = self.camera_objects.pose.orientation.w
                self.tf_broadcaster.sendTransform(t)


                parts = []
                for index, model in enumerate(self.camera_objects.model):
                    parts.append(model.name)  # Collect part data

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id =  logical_camera_frame_name
                    t.child_frame_id = model.name
                    if 0:
                        t.transform.translation = Vector3(
                            x=model.pose.position.x,
                            y=model.pose.position.y,
                            z=model.pose.position.z
                        )
                    t.transform.translation.x = model.pose.position.x
                    t.transform.translation.y = model.pose.position.y
                    t.transform.translation.z = model.pose.position.z
                    t.transform.rotation.x = model.pose.orientation.x
                    t.transform.rotation.y = model.pose.orientation.y
                    t.transform.rotation.z = model.pose.orientation.z
                    t.transform.rotation.w = model.pose.orientation.w

                    if 1: # Patch for transfer_framse workshop
                        # Original quaternion from part_pose
                        q_orig = [
                            model.pose.orientation.w,
                            model.pose.orientation.x,
                            model.pose.orientation.y,
                            model.pose.orientation.z
                        ]

                        # Convert ROS quaternion (w, x, y, z) to scipy quaternion format (x, y, z, w)
                        r_orig = Rotation.from_quat([q_orig[1], q_orig[2], q_orig[3], q_orig[0]])

                        # Rotation quaternion: 180 degrees about the Y-axis
                        r_rot = Rotation.from_euler('xyz', [0, pi, 0], degrees=False)

                        # Combine rotations
                        r_new = r_rot * r_orig  # Composition of rotations
                        q_new = r_new.as_quat()  # Resulting quaternion in (x, y, z, w)

                        # Assign combined quaternion to transformation
                        t.transform.rotation.w = q_new[3]  # Scipy returns (x, y, z, w), so reorder to ROS format
                        t.transform.rotation.x = q_new[0]
                        t.transform.rotation.y = q_new[1]
                        t.transform.rotation.z = q_new[2]


                    #print(t)
                    self.tf_broadcaster.sendTransform(t)

                # Extract part information and camera frame
                photo = {
                    #'camera_frame': self.camera_objects.pose,
                    'parts': parts
                }

                # Clear the stored image after taking the photo
                self.camera_objects = None
                self.get_logger().info("Photo successfully captured.")
                return True, photo

        # Timeout reached without receiving an image
        self.get_logger().warn("Timeout reached. No image data available.")
        return False, None

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MoveTurtleBotNode(Node):
    def __init__(self):
        super().__init__('move_turtlebot')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for the action server to become available...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available!')

    def send_goal(self, x, y, orientation_w=1.0):
        """Send a goal to the action server."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = orientation_w  # Default facing forward

        self.get_logger().info(f'Sending goal to position x={x}, y={y}')
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for when the goal response is received."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback for feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Current position - x={feedback.current_pose.pose.position.x}, y={feedback.current_pose.pose.position.y}')

    def result_callback(self, future):
        """Callback for when the action is complete."""
        result = future.result().result
        if result is None:
            self.get_logger().error('Goal failed to execute.')
        else:
            self.get_logger().info('Goal successfully reached!')

        # Shutdown after achieving all goals
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtleBotNode()

    # Define multiple goals
    goals = [
        (0.5, 1.0),  # First goal (x, y)
        (2.0, 2.5),  # Second goal (x, y)
    ]

    # Send goals sequentially
    for x, y in goals:
        node.send_goal(x, y)

    rclpy.spin(node)


if __name__ == '__main__':
    main()

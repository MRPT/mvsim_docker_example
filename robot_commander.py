#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import LaserScan
import time
import subprocess
from tf2_ros import Buffer, TransformListener


class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_commander')
        # Publisher to send velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Buffer to store transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("RobotCommander initialized")

        # Subscribe to the /scanner1 LaserScan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scanner1',
            self.scan_callback,
            10
        )

        # Create a TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.iter = 0

        # Wait a bit for frames to populate
        self.create_timer(5.0, self.on_timer)  # Run after 2 seconds

    def on_timer(self):
        try:
            self.iter += 1

            # Run different code based on the current value of the counter
            if self.iter == 1:
                # Publish a velocity command
                twist = Twist()
                twist.linear.x = 1.0  # Move forward
                twist.angular.z = 0.0  # No rotation

                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Velocity command sent: move forward")
                self.get_logger().info(
                    f"INITIAL LIDAR RANGE: {self.middle_range}")

            elif self.iter == 2:
                # Publish a velocity command
                twist = Twist()
                twist.linear.x = 0.0  # Stop
                twist.angular.z = 0.0  # No rotation

                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Velocity command sent: stop")
                self.get_logger().info(
                    f"FINAL LIDAR RANGE: {self.middle_range}")

            elif self.iter == 3:
                # For debugging, you can print all tf frames.
                # frames = self.tf_buffer.all_frames_as_string()
                # self.get_logger().info("Available TF frames:\n" + frames)

                # Query the /tf transformation from /base_link to /map
                trans = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                self.get_logger().info(
                    f"Robot's position: {trans.transform.translation}")
                self.get_logger().info(
                    f"Robot's orientation: {trans.transform.rotation}")

            elif self.iter == 4:
                print("Cleaning up...")
                rclpy.shutdown()
            else:
                print("Run", self.iter)

        except Exception as e:
            self.get_logger().error(
                f"Error: {str(e)}")

    def scan_callback(self, msg):
        # Print the middle scan range
        ranges = msg.ranges
        if len(ranges) > 0:
            middle_index = len(ranges) // 2
            middle_range = ranges[middle_index]
            # self.get_logger().info(f"Middle scan range: {middle_range:.2f}")
            self.middle_range = middle_range
        else:
            # self.get_logger().warn("No ranges in LaserScan message.")
            self.middle_range = middle_range


def main(args=None):
    rclpy.init(args=args)
    node = RobotCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

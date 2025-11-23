#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotControlLidar(Node):
    def __init__(self):
        super().__init__('robot_control_lidar_node')

        # Declare and get parameters
        self.declare_parameter('vx', 0.3)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('w', 0.0)
        self.declare_parameter('stop_distance', 0.30)

        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.stop_distance = self.get_parameter('stop_distance').value

        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Lidar subscriber
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.robot_stopped = False

        self.get_logger().info(
            f"Robot LIDAR control started. Moving until obstacle < {self.stop_distance} m"
        )

    def lidar_callback(self, msg):
        if self.robot_stopped:
            return

        # Determine angle depending on direction of movement
        angle = self.get_forward_angle()

        # Convert angle to index
        index = int((angle - msg.angle_min) / msg.angle_increment)
        index = max(0, min(index, len(msg.ranges)-1))

        distance = msg.ranges[index]

        self.get_logger().info(
            f"Distance in movement direction ({math.degrees(angle):.1f}°): {distance:.2f} m"
        )

        # Check stop condition
        if distance < self.stop_distance:
            self.get_logger().warn("Obstacle detected! Stopping robot.")
            self.publisher.publish(Twist())
            self.robot_stopped = True
            return

        # Otherwise keep moving
        vel = Twist()
        vel.linear.x = self.vx
        vel.linear.y = self.vy
        vel.angular.z = self.w
        self.publisher.publish(vel)

    def get_forward_angle(self):
        """Mapping movement direction → LIDAR angle"""
        if abs(self.vx) > abs(self.vy):
            return 0.0 if self.vx > 0 else math.pi  # front or back
        return math.pi/2 if self.vy > 0 else -math.pi/2  # left or right


def main():
    rclpy.init()
    node = RobotControlLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

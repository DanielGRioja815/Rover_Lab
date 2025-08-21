"""Rover_Node controller."""

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

TIME_STEP = 16
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0
OBSTACLE_MIN = 0.75   # metros
OBSTACLE_MAX = 1.8    # metros

class RoverRos2:
    def init(self, webots_node, properties):
        """Initialize the Rover ROS2 controller for Webots."""
        self.robot = webots_node.robot  # Acceso al robot Webots si lo necesitas
        rclpy.init(args=None)
        self.node = rclpy.create_node('rover_ros2')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_subscriber = self.node.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.obstacle_detected = False
        self.turn_left = True

    def lidar_callback(self, msg):
        """Process lidar data and detect obstacles."""
        num_points = len(msg.ranges)
        front_width = int(num_points * 0.2)
        start = (num_points - front_width) // 2
        end = start + front_width
        front_ranges = msg.ranges[start:end]

        # Filter obstacles in the desired range
        obstacles_in_range = [d for d in front_ranges if OBSTACLE_MIN <= d <= OBSTACLE_MAX]
        self.obstacle_detected = bool(obstacles_in_range)

        if self.obstacle_detected:
            left_ranges = msg.ranges[:num_points // 3]
            right_ranges = msg.ranges[-num_points // 3:]
            avg_left = sum(left_ranges) / len(left_ranges)
            avg_right = sum(right_ranges) / len(right_ranges)
            self.turn_left = avg_left > avg_right

    def publish_velocity(self):
        """Publish velocity command."""
        msg = Twist()
        if self.obstacle_detected:
            if self.turn_left:
                msg.linear.x = MAX_SPEED * 0.5
                msg.angular.z = 1.0
                self.node.get_logger().info("Obstacle detected, turning left")
            else:
                msg.linear.x = MAX_SPEED * 0.5
                msg.angular.z = -1.0
                self.node.get_logger().info("Obstacle detected, turning right")
        else:
            msg.linear.x = MAX_SPEED * 0.5
            msg.angular.z = 0.0
            self.node.get_logger().info("No obstacles, moving forward")
        self.publisher.publish(msg)

    def step(self):
        """Called every simulation step by Webots."""
        rclpy.spin_once(self.node, timeout_sec=0)
        self.publish_velocity()


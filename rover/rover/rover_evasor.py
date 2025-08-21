import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025
MAX_SPEED = 7.0
OBSTACLE_MIN = 0.75   # metros
OBSTACLE_MAX = 1.8    # metros

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__obstacle_detected = False
        self.__turn_left = True

        rclpy.init(args=None)
        self.__node = rclpy.create_node('rover_ros2')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(LaserScan, 'scan', self.__lidar_callback, 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __lidar_callback(self, msg):
        num_points = len(msg.ranges)
        front_width = int(num_points * 0.2)
        start = (num_points - front_width) // 2
        end = start + front_width
        front_ranges = msg.ranges[start:end]

        obstacles_in_range = [d for d in front_ranges if OBSTACLE_MIN <= d <= OBSTACLE_MAX]
        self.__obstacle_detected = bool(obstacles_in_range)

        if self.__obstacle_detected:
            left_ranges = msg.ranges[:num_points // 3]
            right_ranges = msg.ranges[-num_points // 3:]
            avg_left = sum(left_ranges) / len(left_ranges)
            avg_right = sum(right_ranges) / len(right_ranges)
            self.__turn_left = avg_left > avg_right

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__obstacle_detected:
            forward_speed = MAX_SPEED * 0.5
            angular_speed = 1.0 if self.__turn_left else -1.0
        else:
            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

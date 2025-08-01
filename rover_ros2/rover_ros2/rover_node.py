#!/usr/bin/env python3
"""Rover_Controller controller."""

from controller import Robot
import math, random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TIME_STEP = 16
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0
OBSTACLE_MIN = 0.75   # metros
OBSTACLE_MAX = 1.8    # metros

robot = Robot()

# Configuración de ruedas (ajusta según tu robot)
motors = []
motorsNames = ['left_motor_1', 'left_motor_2', 'left_motor_3', 'right_motor_1', 'right_motor_2', 'right_motor_3']
for i in range(6):
    motors.append(robot.getDevice(motorsNames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)

lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

def set_speed(left, right):
    # Asume 3 motores por lado
    for i in range(3):
        motors[i].setVelocity(left)
        motors[i+3].setVelocity(right)

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.subscription = self.create_subscription(
            String,
            '/object_detected',
            self.object_callback,
            10)
        self.object_detected = False

    def object_callback(self, msg):
        self.get_logger().info(f"Objeto detectado: {msg.data}")
        self.object_detected = True
        # Aquí puedes cambiar el estado del rover para acercarse y detenerse

    def main_loop(self):
        range_image = lidar.getRangeImage()
        num_points = len(range_image)
        # Considera los 20% centrales como "frente"
        front_width = int(num_points * 0.2)
        start = (num_points - front_width) // 2
        end = start + front_width
        front_ranges = range_image[start:end]

        # Filtra solo los obstáculos en el rango deseado
        obstacles_in_range = [d for d in front_ranges if OBSTACLE_MIN <= d <= OBSTACLE_MAX]
        min_front = min(front_ranges) if front_ranges else float('inf')

        if obstacles_in_range:
            # Decide hacia dónde girar: compara izquierda y derecha
            left_ranges = range_image[:num_points//3]
            right_ranges = range_image[-num_points//3:]
            avg_left = sum(left_ranges) / len(left_ranges)
            avg_right = sum(right_ranges) / len(right_ranges)
            if avg_left > avg_right:
                set_speed(MAX_SPEED * 0.5, MAX_SPEED * 0.5)
                print("Obstáculo adelante (0.75-1.8m), girando a la izquierda")
            else:
                set_speed(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                print("Obstáculo adelante (0.75-1.8m), girando a la derecha")
        else:
            print("No hay obstáculos adelante, avanzando")
            set_speed(-MAX_SPEED * 0.5, MAX_SPEED * 0.5)

def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    # Llama a node.main_loop() dentro de un timer o ciclo adecuado
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


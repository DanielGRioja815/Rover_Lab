#Nodo para publicar velocidades en el teclado

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class Velocidad_Teclado(Node):

    def __init__(self):
        super().__init__('cmd_vel_publishe_publisher')
        self.publisher_ = self.create_publisher(Int32, 'cmd_vel', 10)
        self.get_logger().info('Velocidades de los motores')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        user_input = input("Ingresa velocidad (-100 a 100): ")
        vel = int(user_input)  
        msg = Int32()
        msg.data = int(vel)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Velocidad:  {vel}')


def main(args=None):
    rclpy.init(args=args)
    node = Velocidad_Teclado()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#Nodo para publicar velocidades en el teclado

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty


class comandos_teclado(Node):

    def __init__(self):
        super().__init__('comandos_publisher')
        self.publisher_ = self.create_publisher(String, 'direccion', 10)
        self.get_logger().info('Comandos de los motores')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.settings = termios.tcgetattr(sys.stdin)
        self.i = 0

    def timer_callback(self):
        key = self.get_key()

        if key:  # solo publicar si hay tecla
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)
            self.get_logger().info(f'Tecla publicada: "{key}"')

    def get_key(self):

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) 
        #Tiempo de lectura de teclado, el mismo que el timer_callback = 0.1 en este caso
        if rlist: key = sys.stdin.read(1)
        else: key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

        


def main(args=None):
    rclpy.init(args=args)
    node = comandos_teclado()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    
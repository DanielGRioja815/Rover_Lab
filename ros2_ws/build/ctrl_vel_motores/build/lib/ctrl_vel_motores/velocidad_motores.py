#Nodo para publicar velocidades en el teclado

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from Rosmaster_Lib import Rosmaster
import time
import threading

class velocidad_motores(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscription')

        # Suscriptor ROS2
        self.subscription = self.create_subscription(
            Int32,
            'cmd_vel',
            self.motores_callback,
            10
        )

        # Inicializar controladora Rosmaster con puerto explícito
        self.bot = Rosmaster('/dev/ttyUSB0')

        # Crear hilo de recepción (MUY importante)
        self.bot.create_receive_threading()

        # Velocidad inicial
        self.velocidad_actual = 0

        # Bandera para controlar el hilo del motor
        self.running = True

        # Lanzar un hilo dedicado a los motores
        self.motor_thread = threading.Thread(target=self.motor_loop)
        self.motor_thread.daemon = True
        self.motor_thread.start()

        self.get_logger().info("Nodo ROS2 + Rosmaster inicializado correctamente.")


    def motores_callback(self, msg):
        """Solo actualizar variable compartida, sin bloquear."""
        self.velocidad_actual = msg.data
        self.get_logger().info(f"Velocidad recibida: {self.velocidad_actual}")


    def motor_loop(self):
        
        while 1:
            
            self.bot.set_motor(
                self.velocidad_actual,
                self.velocidad_actual,
                self.velocidad_actual,
                self.velocidad_actual)
           
            self.get_logger().info(f"avanzando")

            time.sleep(0.05)  # 20 Hz → suficiente, no satura el serial


    def destroy_node(self):
        """Apagar hilo y cerrar serial limpiamente."""
        self.running = False
        time.sleep(0.1)

        try:
            del self.bot
        except:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = velocidad_motores()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

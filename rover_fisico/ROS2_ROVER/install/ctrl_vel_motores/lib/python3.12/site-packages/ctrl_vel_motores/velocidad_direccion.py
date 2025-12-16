#Nodo para publicar velocidades en el teclado

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,String
from Rosmaster_Lib import Rosmaster
import time
import threading

class velocidad_direccion(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscription')

        # Suscriptor ROS2
        self.subscription = self.create_subscription(
            Int32,
            'cmd_vel',
            self.motores_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            'direccion',
            self.direccion_callback,
            10
        )

        # Inicializar controladora Rosmaster con puerto explícito
        self.bot = Rosmaster('/dev/ttyUSB0')

        # Crear hilo de recepción (MUY importante)
        self.bot.create_receive_threading()

        # Velocidad inicial
        self.velocidad_actual = 0
        self.comando_actual = "q"

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

    def direccion_callback(self, msg):
        
        self.comando_actual = msg.data
        self.get_logger().info(f"Cambiando estado: {self.comando_actual}")


    def motor_loop(self):
        
        while 1:

            
            #Motor1 = izquierda Motor4 = derecha
            #Motor2 = izquierda Motor3 = derecha

            if self.comando_actual == "q" or self.comando_actual == "Q":
            
                self.bot.set_motor(0,0,0,0)
                self.get_logger().info(f"Stop")

            elif self.comando_actual == "w" or self.comando_actual == "W":

                self.bot.set_motor(self.velocidad_actual,self.velocidad_actual,self.velocidad_actual,self.velocidad_actual)
                self.get_logger().info(f"Avanzando")

            elif self.comando_actual == "s" or self.comando_actual == "S":
                self.bot.set_motor(-self.velocidad_actual,-self.velocidad_actual,-self.velocidad_actual,-self.velocidad_actual)
                self.get_logger().info(f"Retrocede")

            elif self.comando_actual == "a" or self.comando_actual == "A":
                self.bot.set_motor(-self.velocidad_actual,-self.velocidad_actual,self.velocidad_actual,self.velocidad_actual)
                self.get_logger().info(f"Gira_Izq")
                #Motor 1 y 2 = atras
                #Motor 3 y 4 = adelante

            elif self.comando_actual == "d" or self.comando_actual == "D":
                self.bot.set_motor(self.velocidad_actual,self.velocidad_actual,-self.velocidad_actual,-self.velocidad_actual)
                self.get_logger().info(f"Gira_Der")
                #Motor 1 y 2 = adelante
                #Motor 3 y 4 = atras

            else:
                self.get_logger().info(f"Comando no reconocido")

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
    node = velocidad_direccion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

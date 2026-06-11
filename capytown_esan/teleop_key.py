#!/usr/bin/env python3
"""teleop_key.py - Teleoperacion del Yahboom por teclado (WASD).

Controles:
  W / w  - Avanzar
  S / s  - Retroceder
  A / a  - Girar izquierda (antihorario)
  D / d  - Girar derecha (horario)
  ESPACIO - Frenar
  Q / q  - Salir
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.5


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        msg = Twist()
        try:
            self.get_logger().info('Teleop iniciado. Controles: WASD, ESPACIO=frenar, Q=salir')
            while True:
                key = self.get_key()
                msg.linear.x = 0.0
                msg.angular.z = 0.0

                if key == 'w':
                    msg.linear.x = LINEAR_SPEED
                elif key == 's':
                    msg.linear.x = -LINEAR_SPEED
                elif key == 'a':
                    msg.angular.z = ANGULAR_SPEED
                elif key == 'd':
                    msg.angular.z = -ANGULAR_SPEED
                elif key == ' ':
                    pass  # frenar
                elif key == 'q':
                    break

                self.pub.publish(msg)
        except KeyboardInterrupt:
            pass
        finally:
            self.pub.publish(Twist())  # frenar al salir
            self.get_logger().info('Teleop finalizado')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

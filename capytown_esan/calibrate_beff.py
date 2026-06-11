#!/usr/bin/env python3
"""calibrate_beff.py - Calibracion iterativa de b_eff (track efectivo)
mediante protocolo UMBmark simplificado.

El robot gira 360 grados en sitio, se mide el angulo real con un
transportador, y el script calcula el b_eff corregido.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class BeffCalibrator(Node):
    def __init__(self):
        super().__init__('calibrate_beff')

        # Parametros (se pueden pasar por --ros-args -p)
        self.declare_parameter('b_eff', 0.2574)
        self.declare_parameter('target_angle_deg', 358.0)
        self.declare_parameter('angular_vel', 0.5)

        self.b_eff = self.get_parameter('b_eff').value
        self.target_angle = self.get_parameter('target_angle_deg').value
        self.angular_vel = self.get_parameter('angular_vel').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)

        self.get_logger().info(
            f'Iniciando calibracion de b_eff (actual={self.b_eff:.4f} m)\n'
            f'Giro objetivo: {self.target_angle:.1f}° a {self.angular_vel:.2f} rad/s'
        )

    def rotate(self):
        """Gira el robot target_angle grados usando b_eff actual."""
        angle_rad = self.target_angle * 3.14159 / 180.0
        duration = angle_rad / self.angular_vel

        msg = Twist()
        msg.angular.z = self.angular_vel

        steps = int(duration * 10)
        self.get_logger().info(f'Girando durante {duration:.2f} s...')
        for _ in range(steps):
            self.cmd_pub.publish(msg)
            self.rate.sleep()

        # Frenar
        self.cmd_pub.publish(Twist())
        self.rate.sleep()
        self.get_logger().info('Giro completado. Mide el angulo con el transportador.')

    def correct_beff(self, measured_deg):
        """Calcula el b_eff corregido dado el angulo medido."""
        # Si el robot giro MENOS de lo esperado (measured < target),
        # b_eff actual es DEMASIADO GRANDE -> hay que reducirlo
        # Si giro MAS de lo esperado (measured > target),
        # b_eff actual es DEMASIADO PEQUENO -> hay que aumentarlo
        correction = self.target_angle / measured_deg
        new_beff = self.b_eff * correction
        error_pct = abs(measured_deg - self.target_angle) / self.target_angle * 100
        return new_beff, error_pct

    def run(self):
        try:
            while True:
                input('Presiona ENTER para iniciar el giro...')
                self.rotate()

                measured = input(
                    f'Angulo medido (transportador) en grados [{self.target_angle:.1f}]: '
                )
                if not measured.strip():
                    measured = self.target_angle
                else:
                    measured = float(measured)

                new_beff, error_pct = self.correct_beff(measured)
                self.get_logger().info(
                    f'b_eff actual: {self.b_eff:.4f} m\n'
                    f'Angulo medido: {measured:.1f}°\n'
                    f'b_eff corregido: {new_beff:.4f} m\n'
                    f'Error: {error_pct:.2f}%'
                )

                if error_pct < 1.0:
                    self.get_logger().info(
                        'CONVERGENCIA ALCANZADA! '
                        f'b_eff final = {new_beff:.4f} m. '
                        'Actualiza config/wheel_params.yaml y reinicia bringup.'
                    )
                    break

                aplicar = input('Aplicar correccion? (s/n): ').lower()
                if aplicar == 's':
                    self.b_eff = new_beff
                else:
                    manual = input('Ingresa b_eff manualmente (o ENTER para mantener): ')
                    if manual.strip():
                        self.b_eff = float(manual)
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info('Calibracion finalizada')


def main(args=None):
    rclpy.init(args=args)
    node = BeffCalibrator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
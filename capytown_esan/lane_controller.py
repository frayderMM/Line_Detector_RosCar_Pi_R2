#!/usr/bin/env python3
"""CapyTown lane_controller - Semana 11 (RC-2).

Controlador PID sobre /lane_error que publica /cmd_vel.

Convención de signo (igual que lane_detector):
  error > 0  →  centro del carril a la DERECHA  →  ω < 0 (girar derecha)
  error < 0  →  centro del carril a la IZQUIERDA →  ω > 0 (girar izquierda)

Modo recuperación: si se pierde la línea, rota hacia el último lado conocido.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.declare_parameters('', [
            ('kp',             2.5),
            ('ki',             0.0),
            ('kd',             0.3),
            ('linear_speed',   0.20),
            ('max_angular',    2.0),
            ('integral_limit', 0.5),
            ('error_timeout',  0.5),
            ('control_rate',   30.0),
            ('recovery_w',     0.6),   # rad/s al girar buscando la línea
            ('recovery_v',     0.0),   # m/s avance durante recuperación
        ])

        gp             = self.get_parameter
        self.kp        = float(gp('kp').value)
        self.ki        = float(gp('ki').value)
        self.kd        = float(gp('kd').value)
        self.v         = float(gp('linear_speed').value)
        self.max_w     = float(gp('max_angular').value)
        self.i_limit   = float(gp('integral_limit').value)
        self.timeout   = float(gp('error_timeout').value)
        self.recovery_w = float(gp('recovery_w').value)
        self.recovery_v = float(gp('recovery_v').value)
        rate           = float(gp('control_rate').value)

        self.error      = None
        self.last_error = 0.0
        self.integral   = 0.0
        self.last_stamp = self.get_clock().now()
        self.last_rx    = self.get_clock().now()

        self.sub   = self.create_subscription(
            Float32, '/lane_error', self.on_error, 10)
        self.pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('lane_controller listo.')
        self.get_logger().info(
            f'PID kp={self.kp} ki={self.ki} kd={self.kd}  '
            f'v={self.v} m/s  max_w={self.max_w} rad/s  '
            f'recovery_w={self.recovery_w} rad/s')

    def on_error(self, msg):
        if not math.isnan(msg.data):
            self.error   = msg.data
            self.last_rx = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt  = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now

        if dt <= 0.0:
            return

        age = (now - self.last_rx).nanoseconds * 1e-9

        # Línea perdida → modo recuperación: girar hacia el último lado conocido
        if self.error is None or age > self.timeout:
            self.integral = 0.0
            if self.last_error != 0.0:
                # -sign(last_error) porque error>0 → línea a la derecha → girar derecha → ω<0
                direction = -math.copysign(1.0, self.last_error)
                cmd = Twist()
                cmd.linear.x  = self.recovery_v
                cmd.angular.z = direction * self.recovery_w
                self.pub.publish(cmd)
                self.get_logger().info(
                    f'[RECOVERY] línea perdida, girando {"derecha" if direction < 0 else "izquierda"}')
            else:
                self.pub.publish(Twist())
            return

        e = self.error

        # P
        P = self.kp * e

        # I con anti-windup
        self.integral += e * dt
        self.integral  = max(-self.i_limit, min(self.i_limit, self.integral))
        I = self.ki * self.integral

        # D
        derivative = (e - self.last_error) / dt
        D = self.kd * derivative

        # ω negado: error>0 → girar derecha → ω<0
        w = -(P + I + D)
        w = max(-self.max_w, min(self.max_w, w))

        cmd           = Twist()
        cmd.linear.x  = self.v
        cmd.angular.z = w
        self.pub.publish(cmd)

        self.last_error = e


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # frena al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

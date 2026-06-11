#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('integral_limit', 0.4)
        self.declare_parameter('error_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('turn_right_negative', True)

        self.error = float('nan')
        self.last_error = 0.0
        self.integral = 0.0
        self.last_error_time = None
        self.last_control_time = self.get_clock().now()

        self.sub = self.create_subscription(Float32, '/lane_error', self.error_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('LaneController escuchando /lane_error y publicando /cmd_vel')
        self.get_logger().warn('SEGURIDAD: linear_speed inicia en 0.0. Cambiar en config/pid_params.yaml al probar movimiento.')

    def error_callback(self, msg):
        self.error = float(msg.data)
        self.last_error_time = self.get_clock().now()

    def stop_robot(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now

        if dt <= 0.0:
            return

        timeout = float(self.get_parameter('error_timeout').value)

        if self.last_error_time is None:
            self.stop_robot()
            return

        age = (now - self.last_error_time).nanoseconds / 1e9

        if age > timeout or math.isnan(self.error):
            self.integral = 0.0
            self.stop_robot()
            return

        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        linear_speed = float(self.get_parameter('linear_speed').value)
        max_angular = float(self.get_parameter('max_angular').value)
        integral_limit = float(self.get_parameter('integral_limit').value)
        turn_right_negative = bool(self.get_parameter('turn_right_negative').value)

        p = kp * self.error

        self.integral += self.error * dt
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        i = ki * self.integral

        derivative = (self.error - self.last_error) / dt
        d = kd * derivative

        w = p + i + d

        # lane_error positivo = centro del carril a la derecha.
        # En ROS, giro derecha suele ser angular.z negativo.
        if turn_right_negative:
            w = -w

        w = max(-max_angular, min(max_angular, w))

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = w

        self.pub_cmd.publish(cmd)
        self.last_error = self.error


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

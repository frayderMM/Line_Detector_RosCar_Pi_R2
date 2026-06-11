#!/usr/bin/env python3
"""error_odom.py - Analiza un rosbag de RC-1 y grafica la trayectoria.

Uso:
    python3 error_odom.py <ruta_al_bag>
    python3 error_odom.py tambo_G1_run1
"""

import sys
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def read_odom(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions('', '')
    )

    xs, ys = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/odom':
            msg = deserialize_message(data, Odometry)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
    return xs, ys


def plot_trajectory(xs, ys, out_png):
    fig, ax = plt.subplots(figsize=(6, 6))

    # Trayectoria estimada por odometria
    ax.plot(xs, ys, label='trayectoria /odom', color='#B85042', linewidth=2)

    # Cuadrado ideal de 1x1 m
    ideal = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
    ax.plot(
        [p[0] for p in ideal],
        [p[1] for p in ideal],
        '--', label='ideal', color='#5C6D3A', linewidth=2
    )

    ax.set_aspect('equal')
    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Trayectoria RC-1 - La Manzana del Tambo')
    ax.grid(True, alpha=0.3)
    fig.savefig(out_png, dpi=150)
    print(f'Grafico guardado: {out_png}')

    # Calcular error de cierre
    dx = xs[-1] - xs[0]
    dy = ys[-1] - ys[0]
    error_total = (dx**2 + dy**2) ** 0.5
    print(f'Error de cierre:')
    print(f'  dx = {dx*100:+.2f} cm')
    print(f'  dy = {dy*100:+.2f} cm')
    print(f'  error total = {error_total*100:.2f} cm')
    return dx, dy, error_total


def main():
    if len(sys.argv) < 2:
        print('Uso: python3 error_odom.py <ruta_al_bag>')
        sys.exit(1)

    bag_path = sys.argv[1]
    xs, ys = read_odom(bag_path)
    out_png = bag_path.rstrip('/') + '_trayectoria.png'
    plot_trajectory(xs, ys, out_png)


if __name__ == '__main__':
    main()

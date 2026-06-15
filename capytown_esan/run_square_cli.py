#!/usr/bin/env python3
"""run_square_cli.py - Alternativa a move_square.py usando subprocess.
Ejecuta los comandos ROS2 topic pub para avanzar y girar.

CONFIGURACION:
  LADO        = 1.8  m
  VEL_LINEAL  = 0.15 m/s
  VEL_ANGULAR = 0.6  rad/s

  Avanzar: --times = LADO / VEL_LINEAL * 10 = 1.8 / 0.15 * 10 = 120
  Girar:   --times = (pi/2) / VEL_ANGULAR * 10 = 1.57 / 0.6 * 10 = 26

Uso:
    python3 run_square_cli.py
"""

import subprocess
import time


# ===== CONFIGURACION =====
LADO = 1.8
VEL_LINEAL = 0.15
VEL_ANGULAR = 0.6
TIMES_AVANZAR = int(LADO / VEL_LINEAL * 10)  # 120
TIMES_GIRAR = int(3.14159 / 2 / VEL_ANGULAR * 10)  # 26
# ========


def run_cmd(cmd):
    print(f"Ejecutando: {cmd}")
    subprocess.run(cmd, shell=True)


def forward():
    run_cmd(
        f'ros2 topic pub --rate 10 --times {TIMES_AVANZAR} /cmd_vel '
        f'geometry_msgs/msg/Twist "{{linear: {{x: {VEL_LINEAL}}}, angular: {{z: 0.0}}}}"'
    )
    time.sleep(0.2)
    run_cmd(
        'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"'
    )


def turn():
    run_cmd(
        f'ros2 topic pub --rate 10 --times {TIMES_GIRAR} /cmd_vel '
        f'geometry_msgs/msg/Twist "{{linear: {{x: 0.0}}, angular: {{z: {VEL_ANGULAR}}}}}"'
    )
    time.sleep(0.2)
    run_cmd(
        'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"'
    )


def main():
    print(f"Cuadrado de {LADO}x{LADO}m | vel={VEL_LINEAL} m/s | giro={VEL_ANGULAR} rad/s")
    print("Iniciando 3 vueltas...")
    for vuelta in range(1, 4):
        print(f"\n--- Vuelta {vuelta}/3 ---")
        for lado in range(4):
            print(f"  Lado {lado + 1}")
            forward()
            turn()
        print(f"Vuelta {vuelta} completada")
    print("\n3 vueltas finalizadas")


if __name__ == "__main__":
    main()

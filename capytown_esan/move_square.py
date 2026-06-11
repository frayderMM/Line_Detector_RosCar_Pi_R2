#nuevo move square

#!/usr/bin/env python3
"""
move_square.py - RC-1 "La Manzana del Tambo"
Robot recorre el carril AZUL central (21 cm desde el borde del carril)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

# ===============================================================
#   AJUSTA SOLO ESTA SECCION (Configuración para 2x2m y 3 Vueltas)
# ===============================================================

LADO = 1.425          # Distancia ideal para el carril azul en la baldosa de 2x2m
VEL_LINEAL = 0.15     # Velocidad en línea recta (m/s)
VEL_ANGULAR = 0.5   # Velocidad de giro (rad/s)
CORRECCION_GIRO = 1.19 # Multiplicador para compensar el patinaje (1.32 = gira un 32% más)
NUM_VUELTAS = 3       # OBLIGATORIO: El RC-1 requiere 3 vueltas
PAUSA_STOP = 0.5      # Medio segundo de pausa para estabilizar la inercia

# ===============================================================
#   CALCULOS AUTOMATICOS - NO TOCAR
# ===============================================================

TIEMPO_RECTO = LADO / VEL_LINEAL
TIEMPO_GIRO  = (math.pi / 2.0 / VEL_ANGULAR) * CORRECCION_GIRO
FRECUENCIA   = 10


class ManejadorCuadrado(Node):

    def __init__(self):
        super().__init__('manejador_cuadrado')
        self.publicador = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('='*52)
        self.get_logger().info('RC-1 - Carril AZUL central (3 Vueltas)')
        self.get_logger().info('='*52)
        self.get_logger().info(f'  Lado         : {LADO} m')
        self.get_logger().info(f'  Vel lineal   : {VEL_LINEAL} m/s')
        self.get_logger().info(f'  Vel angular  : {VEL_ANGULAR} rad/s')
        self.get_logger().info(f'  Correccion   : x{CORRECCION_GIRO}')
        self.get_logger().info(f'  Tiempo recto : {TIEMPO_RECTO:.2f} s')
        self.get_logger().info(f'  Tiempo giro  : {TIEMPO_GIRO:.2f} s')
        self.get_logger().info(f'  Pausa stop   : {PAUSA_STOP} s')
        self.get_logger().info(f'  Vueltas      : {NUM_VUELTAS}')
        self.get_logger().info('='*52)

    def publicar_comando(self, vel_x: float, vel_z: float, duracion: float):
        """Publica Twist constante durante N segundos."""
        mensaje = Twist()
        mensaje.linear.x  = vel_x
        mensaje.angular.z = vel_z
        tiempo_fin = time.time() + duracion
        intervalo  = 1.0 / FRECUENCIA
        while time.time() < tiempo_fin:
            self.publicador.publish(mensaje)
            time.sleep(intervalo)

    def frenar(self, pausa: float = PAUSA_STOP):
        """Publica Twist=0 para detener el robot."""
        stop = Twist()
        self.publicador.publish(stop)
        time.sleep(pausa)

    def avanzar_recto(self, vuelta: int, lado: int):
        """Avanza LADO metros."""
        self.get_logger().info(
            f'  [Vuelta {vuelta+1}/lado {lado+1}] '
            f'Avanzando {LADO} m ({TIEMPO_RECTO:.1f} s) ...'
        )
        self.publicar_comando(VEL_LINEAL, 0.0, TIEMPO_RECTO)
        self.frenar()

    def girar_izquierda_90(self, vuelta: int, lado: int):
        """
        Gira 90 grados antihorario.
        angular.z > 0 = izquierda (REP-105).
        """
        self.get_logger().info(
            f'  [Vuelta {vuelta+1}/lado {lado+1}] '
            f'Girando 90 izq ({TIEMPO_GIRO:.2f} s, x{CORRECCION_GIRO}) ...'
        )
        self.publicar_comando(0.0, VEL_ANGULAR, TIEMPO_GIRO)
        self.frenar()

    def ejecutar(self):
        """
        Recorre el cuadrado NUM_VUELTAS veces.
        Sentido: antihorario.
        Secuencia por vuelta: recto -> giro x 4 lados.
        """
        self.get_logger().info('Iniciando en 2 segundos - alejate del robot...')
        time.sleep(2.0)

        for vuelta in range(NUM_VUELTAS):
            self.get_logger().info(f'>>> VUELTA {vuelta+1} de {NUM_VUELTAS} <<<')
            for lado in range(4):
                self.avanzar_recto(vuelta, lado)
                self.girar_izquierda_90(vuelta, lado)

        self.frenar(pausa=0.5)
        self.get_logger().info('='*52)
        self.get_logger().info('Recorrido completado.')
        self.get_logger().info('Mide el error con cinta metrica.')
        self.get_logger().info('='*52)


def main(args=None):
    rclpy.init(args=args)
    nodo = ManejadorCuadrado()
    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        nodo.get_logger().warn('Interrumpido - frenando robot.')
        nodo.frenar(pausa=0.5)
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

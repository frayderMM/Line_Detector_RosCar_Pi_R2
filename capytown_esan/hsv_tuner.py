#!/usr/bin/env python3
"""
Calibrador interactivo de HSV con trackbars en tiempo real.

Muestra ventanas con la imagen original, la máscara amarilla y la máscara blanca.
Al presionar 's' guarda los valores actuales en hsv_tuner_output.yaml.
Al presionar 'q' sale.

Uso (dentro del Docker con DISPLAY configurado):
    source /opt/ros/humble/setup.bash
    export DISPLAY=:0
    ros2 run capytown_esan hsv_tuner        # si está instalado como entry_point
    python3 scripts/hsv_tuner.py            # directo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


WIN_ORIG  = "Original (q=salir  s=guardar)"
WIN_YEL   = "Mascara AMARILLO"
WIN_WHT   = "Mascara BLANCO"
WIN_COMP  = "Comparacion (Orig | Amarillo | Blanco)"
OUT_YAML  = "hsv_tuner_output.yaml"


def make_windows():
    for win in [WIN_ORIG, WIN_YEL, WIN_WHT, WIN_COMP]:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 640, 480)

    cv2.createTrackbar('Y H min', WIN_YEL,  15,  179, lambda x: None)
    cv2.createTrackbar('Y H max', WIN_YEL,  45,  179, lambda x: None)
    cv2.createTrackbar('Y S min', WIN_YEL,  45,  255, lambda x: None)
    cv2.createTrackbar('Y S max', WIN_YEL, 255,  255, lambda x: None)
    cv2.createTrackbar('Y V min', WIN_YEL,  80,  255, lambda x: None)
    cv2.createTrackbar('Y V max', WIN_YEL, 255,  255, lambda x: None)

    cv2.createTrackbar('W S min', WIN_WHT,   0,  255, lambda x: None)
    cv2.createTrackbar('W S max', WIN_WHT,  65,  255, lambda x: None)
    cv2.createTrackbar('W V min', WIN_WHT, 170,  255, lambda x: None)
    cv2.createTrackbar('W V max', WIN_WHT, 255,  255, lambda x: None)


def get_params():
    g = cv2.getTrackbarPos
    ylo = np.array([g('Y H min', WIN_YEL), g('Y S min', WIN_YEL), g('Y V min', WIN_YEL)], np.uint8)
    yhi = np.array([g('Y H max', WIN_YEL), g('Y S max', WIN_YEL), g('Y V max', WIN_YEL)], np.uint8)
    wlo = np.array([0,                     g('W S min', WIN_WHT), g('W V min', WIN_WHT)], np.uint8)
    whi = np.array([179,                   g('W S max', WIN_WHT), g('W V max', WIN_WHT)], np.uint8)
    return ylo, yhi, wlo, whi


def save_yaml(ylo, yhi, wlo, whi):
    data = {
        'lane_detector': {
            'ros__parameters': {
                'yellow_h_min': int(ylo[0]), 'yellow_h_max': int(yhi[0]),
                'yellow_s_min': int(ylo[1]), 'yellow_s_max': int(yhi[1]),
                'yellow_v_min': int(ylo[2]), 'yellow_v_max': int(yhi[2]),
                'white_s_min':  int(wlo[1]), 'white_s_max':  int(whi[1]),
                'white_v_min':  int(wlo[2]), 'white_v_max':  int(whi[2]),
            }
        }
    }
    with open(OUT_YAML, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"\n=== Guardado en {os.path.abspath(OUT_YAML)} ===")
    print(f"  Amarillo  H:[{ylo[0]},{yhi[0]}]  S:[{ylo[1]},{yhi[1]}]  V:[{ylo[2]},{yhi[2]}]")
    print(f"  Blanco    S:[{wlo[1]},{whi[1]}]  V:[{wlo[2]},{whi[2]}]")


class HsvTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.bridge  = CvBridge()
        self.frame   = None
        make_windows()
        self.sub = self.create_subscription(Image, '/image_raw', self._cb, 5)
        self.timer = self.create_timer(0.03, self._update)
        self.get_logger().info("HSV Tuner listo. Ventanas abiertas.")
        self.get_logger().info("  's' → guardar YAML    'q' → salir")

    def _cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))

    def _update(self):
        if self.frame is None:
            return

        frame = self.frame.copy()
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ylo, yhi, wlo, whi = get_params()

        mask_y = cv2.inRange(hsv, ylo, yhi)
        mask_w = cv2.inRange(hsv, wlo, whi)

        # Morfología básica
        k = np.ones((3, 3), np.uint8)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN,  k)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN,  k)

        # Overlay de colores sobre la imagen
        overlay = frame.copy()
        overlay[mask_y > 0] = (0, 220, 0)    # verde = amarillo detectado
        overlay[mask_w > 0] = (255, 200, 0)  # azul  = blanco detectado
        composite = cv2.addWeighted(frame, 0.5, overlay, 0.5, 0)

        # Texto informativo
        info = (f"Y H:[{ylo[0]},{yhi[0]}] S:[{ylo[1]},{yhi[1]}] V:[{ylo[2]},{yhi[2]}]  |  "
                f"W S:[{wlo[1]},{whi[1]}] V:[{wlo[2]},{whi[2]}]  |  "
                f"s=guardar  q=salir")
        cv2.putText(composite, info, (5, frame.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)

        # Mostrar máscaras como BGR
        mask_y_bgr = cv2.cvtColor(mask_y, cv2.COLOR_GRAY2BGR)
        mask_w_bgr = cv2.cvtColor(mask_w, cv2.COLOR_GRAY2BGR)

        cv2.imshow(WIN_ORIG, frame)
        cv2.imshow(WIN_YEL,  mask_y_bgr)
        cv2.imshow(WIN_WHT,  mask_w_bgr)
        cv2.imshow(WIN_COMP, composite)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            save_yaml(ylo, yhi, wlo, whi)
        elif key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HsvTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

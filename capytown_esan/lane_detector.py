#!/usr/bin/env python3
"""CapyTown lane_detector - Semana 11 (RC-2).

Amarillo (izquierda): HSV  — línea interior del carril
Blanco   (derecha):   LAB  — línea exterior del carril

El robot mantiene la línea amarilla a su izquierda y la blanca a su derecha,
calculando el error como la distancia del centro entre ambas líneas al centro
de la imagen.

Convención de signo:
  error > 0  →  centro a la DERECHA del robot  →  girar derecha (ω < 0)
  error < 0  →  centro a la IZQUIERDA del robot →  girar izquierda (ω > 0)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        self.declare_parameters('', [
            # Blanco - HSV (baja saturación + valor medio-alto = gris/blanco)
            ('white_s_min',        0),
            ('white_s_max',        80),    # saturación baja = sin color = blanco/gris
            ('white_v_min',        100),   # valor mínimo para separar del piso oscuro
            ('white_v_max',        255),
            ('white_max_area',     25000), # rechaza reflejos muy grandes
            # Amarillo - HSV
            ('yellow_h_min', 15),
            ('yellow_h_max', 40),
            ('yellow_s_min', 60),
            ('yellow_s_max', 255),
            ('yellow_v_min', 80),
            ('yellow_v_max', 255),
            # Geometría
            ('min_area',           150),
            ('px_per_meter',       600.0),
            ('look_ahead_row',     0.88),
            ('band_half_height',   30),
            # Navegación: amarillo como referencia principal
            ('yellow_setpoint',    0.28), # fracción del ancho donde debe estar el amarillo (izq)
            ('white_weight',       0.25), # peso del blanco cuando ambas líneas visibles (0=ignorar blanco)
            # Comportamiento
            ('require_both_lines', False),
            ('publish_debug',      True),
        ])

        gp = self.get_parameter

        # Blanco en HSV: H cualquiera, S baja, V alta
        self.white_lo_hsv = np.array([0,   gp('white_s_min').value, gp('white_v_min').value], dtype=np.uint8)
        self.white_hi_hsv = np.array([179, gp('white_s_max').value, gp('white_v_max').value], dtype=np.uint8)
        self.white_max_area = float(gp('white_max_area').value)

        self.yellow_lo = np.array([gp('yellow_h_min').value,
                                    gp('yellow_s_min').value,
                                    gp('yellow_v_min').value], dtype=np.uint8)
        self.yellow_hi = np.array([gp('yellow_h_max').value,
                                    gp('yellow_s_max').value,
                                    gp('yellow_v_max').value], dtype=np.uint8)

        self.min_area        = float(gp('min_area').value)
        self.px_per_meter    = float(gp('px_per_meter').value)
        self.look_ahead_row  = float(gp('look_ahead_row').value)
        self.band_half_h     = int(gp('band_half_height').value)
        self.yellow_setpoint = float(gp('yellow_setpoint').value)
        self.white_weight    = float(gp('white_weight').value)
        self.require_both    = bool(gp('require_both_lines').value)
        self.publish_debug   = bool(gp('publish_debug').value)

        self.M         = None
        self.warp_size = None

        self.sub     = self.create_subscription(
            Image, '/image_raw', self.on_image, 10)
        self.pub_err = self.create_publisher(Float32, '/lane_error', 10)
        self.pub_dbg = self.create_publisher(Image, '/lane/debug_image', 10)

        self.get_logger().info('lane_detector listo.')
        self.get_logger().info(
            f'yellow HSV [{self.yellow_lo}] - [{self.yellow_hi}]  '
            f'white HSV [{self.white_lo_hsv}] - [{self.white_hi_hsv}]')

    # ------------------------------------------------------------------
    def build_ipm(self, w, h):
        src = np.float32([
            [0.20 * w, 0.55 * h],
            [0.80 * w, 0.55 * h],
            [1.00 * w, 0.97 * h],
            [0.00 * w, 0.97 * h],
        ])
        dst = np.float32([
            [0.25 * w, 0.0],
            [0.75 * w, 0.0],
            [0.75 * w,  h],
            [0.25 * w,  h],
        ])
        self.M         = cv2.getPerspectiveTransform(src, dst)
        self.warp_size = (w, h)

    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]
        if self.M is None:
            self.build_ipm(w, h)

        warp = cv2.warpPerspective(frame, self.M, self.warp_size)

        # ── Detección de colores ──────────────────────────────────────
        hsv         = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
        mask_yellow    = cv2.inRange(hsv, self.yellow_lo,    self.yellow_hi)
        mask_white_raw = cv2.inRange(hsv, self.white_lo_hsv, self.white_hi_hsv)

        # Morfología para eliminar ruido
        kernel = np.ones((3, 3), np.uint8)
        mask_yellow    = cv2.morphologyEx(mask_yellow,    cv2.MORPH_OPEN,  kernel)
        mask_yellow    = cv2.morphologyEx(mask_yellow,    cv2.MORPH_CLOSE, kernel)
        mask_white_raw = cv2.morphologyEx(mask_white_raw, cv2.MORPH_OPEN,  kernel)
        mask_white_raw = cv2.morphologyEx(mask_white_raw, cv2.MORPH_CLOSE, kernel)

        # Amarillo: sin restricción de zona — su hue es específico, funciona en curvas
        # Blanco: restringido a mitad derecha — evita falsos positivos de reflejos
        right_zone = np.zeros((h, w), dtype=np.uint8)
        right_zone[:, w // 2:] = 255
        mask_white_raw = cv2.bitwise_and(mask_white_raw, right_zone)

        # Evitar que el amarillo contamine el blanco
        mask_white_raw = cv2.bitwise_and(mask_white_raw,
                                          cv2.bitwise_not(mask_yellow))

        # Filtrar blanco: solo blobs de tamaño razonable (descarta reflejos y ruido)
        mask_white = self._filter_by_area(mask_white_raw)

        # ── Centroides en la banda de look-ahead ──────────────────────
        row  = int(self.look_ahead_row * h)
        band = slice(max(0, row - self.band_half_h),
                     min(h, row + self.band_half_h))

        x_yellow = self._centroid_x(mask_yellow[band, :])
        x_white  = self._centroid_x(mask_white[band, :])

        # ── Error de posición (amarillo = referencia principal) ───────
        # El robot se posiciona para que el amarillo aparezca en yellow_setpoint
        # del ancho de imagen. El blanco aporta solo white_weight de corrección.
        target_y_px = self.yellow_setpoint * w
        target_w_px = (1.0 - self.yellow_setpoint) * w

        error_px = None

        if x_yellow is not None and x_white is not None:
            err_y    = x_yellow - target_y_px
            err_w    = x_white  - target_w_px
            error_px = (1.0 - self.white_weight) * err_y + self.white_weight * err_w
        elif x_yellow is not None:
            error_px = x_yellow - target_y_px
        elif x_white is not None and not self.require_both:
            error_px = x_white - target_w_px

        error_m = error_px / self.px_per_meter if error_px is not None else float('nan')

        out      = Float32()
        out.data = float(error_m)
        self.pub_err.publish(out)

        # Para debug: posición del centro estimado en píxeles
        center_px = (w * self.yellow_setpoint + error_px) if error_px is not None else None

        if self.publish_debug:
            self._publish_debug(warp, mask_white, mask_yellow, row,
                                x_white, x_yellow, center_px, msg)

    # ------------------------------------------------------------------
    def _filter_by_area(self, mask):
        """Conserva contornos cuya área esté entre min_area y white_max_area."""
        result = np.zeros_like(mask)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area <= area <= self.white_max_area:
                cv2.drawContours(result, [cnt], -1, 255, -1)
        return result

    @staticmethod
    def _centroid_x(mask):
        m = cv2.moments(mask, binaryImage=True)
        if m['m00'] < 1e-3:
            return None
        return m['m10'] / m['m00']

    # ------------------------------------------------------------------
    def _publish_debug(self, warp, mask_white, mask_yellow, row,
                       xw, xy, xc, header_msg):
        h, w = warp.shape[:2]
        dbg = warp.copy()

        # Overlay semitransparente de máscaras sobre la imagen warpeada
        overlay = dbg.copy()
        overlay[mask_yellow > 0] = (0, 220, 220)   # cyan = amarillo detectado
        overlay[mask_white  > 0] = (200, 200, 255)  # azul claro = blanco detectado
        cv2.addWeighted(overlay, 0.5, dbg, 0.5, 0, dbg)

        # Línea de look-ahead y centro de imagen
        cv2.line(dbg, (0, row), (w, row), (0, 255, 0), 1)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

        # Puntos de centroide
        for x, color, label in (
            (xw, (255, 255, 255), 'W'),   # blanco
            (xy, (0, 200, 255),   'Y'),   # amarillo
            (xc, (0, 0, 255),     'C'),   # centro calculado
        ):
            if x is not None:
                cv2.circle(dbg, (int(x), row), 7, color, -1)
                cv2.putText(dbg, label, (int(x) + 9, row - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Estado de detección en texto
        detected = []
        if xy is not None:
            detected.append('Y')
        if xw is not None:
            detected.append('W')
        status = '+'.join(detected) if detected else 'NONE'
        cv2.putText(dbg, f'Lines: {status}', (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        out        = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = header_msg.header
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

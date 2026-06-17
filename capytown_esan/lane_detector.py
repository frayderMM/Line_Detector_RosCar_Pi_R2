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
            # Blanco - HSV
            ('white_s_min',           0),
            ('white_s_max',           65),
            ('white_v_min',           170),
            ('white_v_max',           255),
            ('white_max_area',        25000),
            ('white_min_area',        1000),
            ('white_min_elongation',  5.0),
            # Amarillo - HSV
            ('yellow_h_min',          15),
            ('yellow_h_max',          45),
            ('yellow_s_min',          45),
            ('yellow_s_max',          255),
            ('yellow_v_min',          80),
            ('yellow_v_max',          255),
            ('yellow_min_area',       500),
            ('yellow_min_elongation', 8.0),
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
        self.white_lo_hsv         = np.array([0,   gp('white_s_min').value, gp('white_v_min').value], dtype=np.uint8)
        self.white_hi_hsv         = np.array([179, gp('white_s_max').value, gp('white_v_max').value], dtype=np.uint8)
        self.white_max_area       = float(gp('white_max_area').value)
        self.white_min_area       = float(gp('white_min_area').value)
        self.white_min_elongation = float(gp('white_min_elongation').value)
        self.yellow_min_area      = float(gp('yellow_min_area').value)
        self.yellow_min_elongation= float(gp('yellow_min_elongation').value)

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

        # Filtrar por forma: solo cintas alargadas, no reflejos redondos
        mask_yellow = self._filter_by_shape(
            mask_yellow,
            min_area=self.yellow_min_area, max_area=self.white_max_area,
            min_elongation=self.yellow_min_elongation)
        mask_white = self._filter_by_shape(
            mask_white_raw,
            min_area=self.white_min_area,  max_area=self.white_max_area,
            min_elongation=self.white_min_elongation,
            min_cx_ratio=0.45)

        # ── Centroides en la banda de look-ahead ──────────────────────
        row  = int(self.look_ahead_row * h)
        band = slice(max(0, row - self.band_half_h),
                     min(h, row + self.band_half_h))

        x_yellow = self._centroid_x(mask_yellow[band, :])
        x_white  = self._centroid_x(mask_white[band, :])

        # ── Error de posición ─────────────────────────────────────────
        # Modo RECTA  (Y+W visibles): centro exacto entre las dos líneas
        # Modo CURVA  (solo Y):       amarillo como referencia de setpoint
        # Modo NINGUNA:               NaN → controller para el robot
        error_px = None

        if x_yellow is not None and x_white is not None:
            # Recta: ancla al centro real del carril
            error_px = (x_yellow + x_white) / 2.0 - w / 2.0
        elif x_yellow is not None:
            # Curva: mantener amarillo en su posición objetivo
            error_px = x_yellow - self.yellow_setpoint * w
        elif x_white is not None and not self.require_both:
            # Solo blanco (raro): usar blanco como referencia
            error_px = x_white - (1.0 - self.yellow_setpoint) * w

        error_m = error_px / self.px_per_meter if error_px is not None else float('nan')

        out      = Float32()
        out.data = float(error_m)
        self.pub_err.publish(out)

        # Para debug: posición del centro estimado en píxeles
        center_px = (w / 2.0 + error_px) if error_px is not None else None

        if self.publish_debug:
            self._publish_debug(warp, mask_white, mask_yellow, row,
                                x_white, x_yellow, center_px, msg)

    # ------------------------------------------------------------------
    def _filter_by_shape(self, mask, min_area, max_area, min_elongation,
                         min_cx_ratio=0.0, h_total=0, min_cy=0):
        """Conserva blobs alargados (forma de cinta) usando PCA.
        Rechaza reflejos puntuales (circulares) y ruido pequeño."""
        result = np.zeros_like(mask)
        h, w = mask.shape[:2]
        num, labels, stats, cents = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        for i in range(1, num):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area or area > max_area:
                continue
            cx, cy = cents[i]
            if cx < min_cx_ratio * w:
                continue
            if cy < min_cy:
                continue
            # Elongación por PCA
            pts = np.column_stack(np.where(labels == i))
            if len(pts) > 10:
                xy = pts[:, ::-1].astype(np.float32)
                _, _, eigval = cv2.PCACompute2(xy, mean=None)
                elongation = float(eigval[0, 0] / (eigval[1, 0] + 1e-6))
                if elongation < min_elongation:
                    continue
            result[labels == i] = 255
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

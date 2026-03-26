"""
vision_node.py
--------------
ROS2 node: képfeldolgozás és hibavektor számítás.

Feladatok:
  1. Feliratkozik a kamera képfolyamára
  2. Detektálja az aljzatot és a dugaszt (feature_detector)
  3. Kiszámítja a hibavektort (ibvs_controller HomographyManager)
  4. Publikálja az eredményt a controller_node számára
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from cv_bridge import CvBridge

from .feature_detector import ConnectorDetector, PlugDetector
from .ibvs_controller import HomographyManager, DepthEstimator


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        # ── Paraméterek ────────────────────────────────────────────────
        self.declare_parameter("plug_detection_method", "color")
        self.declare_parameter("convergence_threshold_px", 5.0)
        self.declare_parameter("connector_real_size_mm", 80.0)
        self.declare_parameter("debug_image", True)

        plug_method  = self.get_parameter("plug_detection_method").value
        self.conv_th = self.get_parameter("convergence_threshold_px").value
        real_size    = self.get_parameter("connector_real_size_mm").value
        self.debug   = self.get_parameter("debug_image").value

        # ── Detektorok ─────────────────────────────────────────────────
        self.socket_detector = ConnectorDetector()
        self.plug_detector   = PlugDetector(method=plug_method)
        self.homography_mgr  = HomographyManager()
        self.bridge          = CvBridge()

        # Kamera paraméterek (kalibrációs üzenetből töltjük be)
        self.fx = self.fy = 600.0
        self.cx = 640.0
        self.cy = 360.0
        self.camera_info_received = False

        # Mélységbecslő (ha nincs RealSense)
        self.depth_estimator = DepthEstimator(
            focal_length_px=self.fx,
            real_size_mm=real_size
        )

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10
        )

        # ── Publishers ─────────────────────────────────────────────────
        self.error_pub = self.create_publisher(
            Twist, "/vision/visual_error", 10
        )
        self.detected_pub = self.create_publisher(
            Bool, "/vision/target_detected", 10
        )
        if self.debug:
            self.debug_pub = self.create_publisher(
                Image, "/vision/debug_image", 10
            )

        # ── Állapot ────────────────────────────────────────────────────
        # Referencia jellemzők (kalibrációs fázisból)
        # Ha None: még nincs kalibrálva → csak detektálás fut
        self.ref_socket_corners = None
        self.ref_plug_corners   = None
        self.calibrated         = False

        self.get_logger().info("VisionNode elindult.")
        self.get_logger().warn(
            "Kalibrálás szükséges! Hívd meg a /vision/calibrate service-t."
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        """Kamera kalibrációs adatok fogadása (egyszer fut le)."""
        if self.camera_info_received:
            return

        K = msg.k           # 3×3 intrinsic mátrix (sorfolytonosan)
        self.fx = K[0]
        self.fy = K[4]
        self.cx = K[2]
        self.cy = K[5]

        # Frissítjük a mélységbecslőt a pontos fókusztávolsággal
        self.depth_estimator.f = self.fx

        self.camera_info_received = True
        self.get_logger().info(
            f"Kamera kalibrálás betöltve: fx={self.fx:.1f}, fy={self.fy:.1f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

    def image_callback(self, msg: Image):
        """
        Fő képfeldolgozó callback – minden képkockán lefut (~30 Hz).
        """
        # ROS2 Image → OpenCV BGR kép
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # ── 1. Detektálás ──────────────────────────────────────────────
        socket_result = self.socket_detector.detect(image)
        plug_result   = self.plug_detector.detect(image)

        detected = socket_result.success and plug_result.success
        self.detected_pub.publish(Bool(data=detected))

        if not detected:
            self._publish_zero_error()
            if self.debug:
                self._publish_debug(image, socket_result, plug_result)
            return

        # ── 2. Mélység becslés ─────────────────────────────────────────
        # Az aljzat területéből becsüljük a mélységet
        Z = self.depth_estimator.estimate_from_area(socket_result.area)

        # ── 3. Hibavektor számítás ─────────────────────────────────────
        if self.calibrated:
            # Homográfia alapú hiba a virtuális képsíkon
            error_vec = self.homography_mgr.compute_error(
                socket_result.corners,
                plug_result.corners
            )
            # Átlagos x és y hiba a 4 sarokpontból
            error_xy = error_vec.reshape(4, 2)
            error_x = float(error_xy[:, 0].mean())
            error_y = float(error_xy[:, 1].mean())
        else:
            # Kalibráció nélküli egyszerű módszer:
            # csak a középpontok különbségét mérjük
            error_x = float(socket_result.center[0] - plug_result.center[0])
            error_y = float(socket_result.center[1] - plug_result.center[1])
            self.get_logger().warn(
                "Nincs homográfia kalibráció – egyszerű középpont hibát használok!",
                throttle_duration_sec=5.0
            )

        # Z irányú hiba (mélység) – a kívánt dokkolási mélységtől való eltérés
        target_depth = 0.15    # 15 cm – ezt kalibráláskor kell beállítani
        error_z = Z - target_depth

        # ── 4. Hibavektor publikálás ────────────────────────────────────
        error_msg = Twist()
        error_msg.linear.x  = error_x
        error_msg.linear.y  = error_y
        error_msg.linear.z  = error_z
        # Az angular részt a vezérlőhöz hagyjuk (orientáció korrekció)
        self.error_pub.publish(error_msg)

        # ── 5. Debug kép ───────────────────────────────────────────────
        if self.debug:
            self._publish_debug(image, socket_result, plug_result,
                                 error_x, error_y, error_z, Z)

    # ──────────────────────────────────────────────────────────────────
    # Segédfüggvények
    # ──────────────────────────────────────────────────────────────────

    def _publish_zero_error(self):
        """Nulla hibavektor publikálása (ha nincs detektálás)."""
        self.error_pub.publish(Twist())

    def _publish_debug(self, image, socket_res, plug_res,
                        ex=0.0, ey=0.0, ez=0.0, Z=0.0):
        """Annotált debug kép publikálása RViz2-be."""
        debug = image.copy()

        # Aljzat rajzolása (zöld)
        debug = self.socket_detector.draw_debug(debug, socket_res)

        # Dugasz rajzolása (kék)
        if plug_res.success:
            cx, cy = plug_res.center.astype(int)
            cv2.circle(debug, (cx, cy), 10, (255, 100, 0), -1)
            cv2.putText(debug, "PLUG", (cx + 12, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)

        # Hibavektor felirat
        h, w = debug.shape[:2]
        cv2.putText(debug,
                    f"err: x={ex:.1f}px  y={ey:.1f}px  Z={Z:.3f}m",
                    (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        if not socket_res.success or not plug_res.success:
            cv2.putText(debug, "DETECTION FAILED", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        ros_img = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        self.debug_pub.publish(ros_img)

    def set_calibration(self,
                         ref_socket_corners: np.ndarray,
                         ref_plug_corners: np.ndarray):
        """
        Homográfia kalibráció végrehajtása referencia pontokkal.

        Ezt a metódust akkor hívjuk, amikor a robot manuálisan
        be van illesztve (tanítási fázis).

        Args:
            ref_socket_corners: aljzat sarokpontjai a referencia pozícióban (4×2)
            ref_plug_corners:   dugasz sarokpontjai a referencia pozícióban (4×2)
        """
        self.homography_mgr.calibrate_from_reference(
            ref_socket_corners,
            ref_plug_corners
        )
        self.ref_socket_corners = ref_socket_corners
        self.ref_plug_corners   = ref_plug_corners
        self.calibrated = True
        self.get_logger().info("Homográfia kalibráció kész.")


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
camera_node.py
--------------
ROS2 node: valódi USB kamera kezelése (nem szimulációhoz).

Szimulációban ezt a node-ot NEM kell futtatni –
a Gazebo kamera plugin közvetlenül publikál a /camera/image_raw topic-ra.

Valódi hardvernél (laborban) ezt indítjuk a kamera helyett,
vagy használhatjuk a gyártó saját ROS2 driverét:
  - Intel RealSense: realsense2_camera csomag
  - Általános USB:   v4l2_camera csomag

Ez a node egy egyszerű OpenCV alapú alternatíva,
ha a fentiek valami miatt nem elérhetők.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        # ── Paraméterek ────────────────────────────────────────────────
        self.declare_parameter("device_id",         0)       # /dev/video0
        self.declare_parameter("image_width",        1280)
        self.declare_parameter("image_height",       720)
        self.declare_parameter("fps",                30)
        self.declare_parameter("calibration_file",
                               "config/camera_calibration.yaml")

        device    = self.get_parameter("device_id").value
        width     = self.get_parameter("image_width").value
        height    = self.get_parameter("image_height").value
        fps       = self.get_parameter("fps").value
        calib_f   = self.get_parameter("calibration_file").value

        # ── Kamera megnyitása ──────────────────────────────────────────
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Nem sikerült megnyitni a kamerát: /dev/video{device}")
            raise RuntimeError("Kamera nem elérhető")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        self.bridge = CvBridge()

        # ── Kalibrációs adatok betöltése ───────────────────────────────
        self.camera_info_msg = self._load_calibration(calib_f, width, height)

        # ── Publishers ─────────────────────────────────────────────────
        self.image_pub = self.create_publisher(
            Image, "/camera/image_raw", 10
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", 10
        )

        # ── Timer: képkocka olvasás ────────────────────────────────────
        dt = 1.0 / fps
        self.timer = self.create_timer(dt, self.capture_and_publish)

        self.get_logger().info(
            f"CameraNode elindult: {width}×{height} @ {fps}fps, "
            f"device=/dev/video{device}"
        )

    def capture_and_publish(self):
        """Képkocka olvasása és publikálása."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Képkocka olvasása sikertelen.", throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        # Image üzenet
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp    = now
        img_msg.header.frame_id = "camera_optical_frame"
        self.image_pub.publish(img_msg)

        # CameraInfo üzenet (ugyanolyan timestamp-pel)
        self.camera_info_msg.header.stamp    = now
        self.camera_info_msg.header.frame_id = "camera_optical_frame"
        self.info_pub.publish(self.camera_info_msg)

    def _load_calibration(self, filepath: str,
                           width: int, height: int) -> CameraInfo:
        """
        Kamera kalibrációs adatok betöltése YAML fájlból.
        Ha a fájl nem létezik, alapértelmezett (becsült) értékeket használ.
        """
        msg = CameraInfo()
        msg.width  = width
        msg.height = height

        try:
            import yaml
            with open(filepath, "r") as f:
                data = yaml.safe_load(f)

            k = data["camera_matrix"]["data"]
            d = data["distortion_coefficients"]["data"]

            msg.k = k          # 3×3 intrinsic mátrix (sorfolytonos, 9 elem)
            msg.d = d          # torzítási együtthatók
            msg.distortion_model = "plumb_bob"

            self.get_logger().info(f"Kalibráció betöltve: {filepath}")

        except (FileNotFoundError, KeyError):
            # Becsült értékek (Logitech C920 alapján, 1280×720)
            fx = fy = 900.0
            cx = width  / 2.0
            cy = height / 2.0

            msg.k = [fx, 0.0, cx,
                     0.0, fy, cy,
                     0.0, 0.0, 1.0]
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.distortion_model = "plumb_bob"

            self.get_logger().warn(
                f"Kalibrációs fájl nem található: {filepath}\n"
                "Becsült értékeket használok – kalibráld a kamerát!"
            )

        return msg

    def destroy_node(self):
        """Kamera felszabadítása node leálláskor."""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

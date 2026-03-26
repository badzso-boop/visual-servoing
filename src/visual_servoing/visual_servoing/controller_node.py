"""
controller_node.py
------------------
ROS2 node: IBVS + ADRC vezérlő és állapotgép.

Ez a node fogadja a vision_node hibavektorát,
és az ADRC segítségével sebesség parancsot küld a robotkarnak.
"""

import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .adrc_controller import MultiAxisADRC
from .ibvs_controller import IBVSController, IBVSResult


class AssemblyState(Enum):
    """Az illesztési folyamat állapotai."""
    WAITING         = auto()   # Várakozás – nincs detektált célpont
    XY_ALIGNMENT    = auto()   # X/Y igazítás a képtérben
    Z_APPROACH      = auto()   # Z irányú közelítés
    DOCKING         = auto()   # Végső beillesztés
    DONE            = auto()   # Sikeres illesztés
    ERROR           = auto()   # Hiba állapot


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # ── Paraméterek ────────────────────────────────────────────────
        self.declare_parameter("xy_threshold_px",   8.0)   # pixel
        self.declare_parameter("z_threshold_m",     0.02)  # méter
        self.declare_parameter("approach_speed",    0.02)  # m/s
        self.declare_parameter("docking_speed",     0.005) # m/s
        self.declare_parameter("control_rate_hz",   30.0)

        self.xy_th     = self.get_parameter("xy_threshold_px").value
        self.z_th      = self.get_parameter("z_threshold_m").value
        self.app_spd   = self.get_parameter("approach_speed").value
        self.dock_spd  = self.get_parameter("docking_speed").value
        rate_hz        = self.get_parameter("control_rate_hz").value

        dt = 1.0 / rate_hz

        # ── Vezérlők ───────────────────────────────────────────────────
        self.adrc = MultiAxisADRC({
            "dt":                  dt,
            "observer_bandwidth":  50.0,
            "control_bandwidth":   8.0,
            "b0":                  1.0,
            "max_vx":              0.05,
            "max_vy":              0.05,
            "max_vz":              0.03,
        })

        self.ibvs = IBVSController({
            "gain":                 0.3,
            "convergence_threshold": self.xy_th,
            "max_linear_vel":       0.05,
            "max_angular_vel":      0.2,
        })

        # ── Állapotgép ─────────────────────────────────────────────────
        self.state = AssemblyState.WAITING
        self.consecutive_done = 0    # hány egymást követő képkockán volt kész

        # Legutóbbi adatok
        self.last_error     = Twist()
        self.target_visible = False

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(
            Twist, "/vision/visual_error",
            self.error_callback, 10
        )
        self.create_subscription(
            Bool, "/vision/target_detected",
            self.detected_callback, 10
        )

        # ── Publishers ─────────────────────────────────────────────────
        # Cartesian sebesség a robot_node felé
        self.vel_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        # Státusz kijelzőhöz
        self.status_pub = self.create_publisher(
            String, "/assembly/status", 10
        )

        # ── Vezérlő timer ──────────────────────────────────────────────
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(
            f"ControllerNode elindult. dt={dt:.3f}s, "
            f"XY küszöb={self.xy_th}px, Z küszöb={self.z_th}m"
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def error_callback(self, msg: Twist):
        """Hibavektor fogadása a vision_node-tól."""
        self.last_error = msg
        # ADRC mélység frissítése
        self.ibvs.update_depth_estimate(
            0.15 + msg.linear.z   # kívánt mélység + eltérés = aktuális
        )

    def detected_callback(self, msg: Bool):
        """Célpont láthatóság frissítése."""
        self.target_visible = msg.data

        if not msg.data and self.state not in (
            AssemblyState.WAITING, AssemblyState.DONE, AssemblyState.ERROR
        ):
            self.get_logger().warn(
                "Célpont elveszett! Visszatérés WAITING állapotba.",
                throttle_duration_sec=2.0
            )
            self._transition(AssemblyState.WAITING)

    # ──────────────────────────────────────────────────────────────────
    # Vezérlő főhurok
    # ──────────────────────────────────────────────────────────────────

    def control_loop(self):
        """
        Vezérlő hurok – a timer hívja meg rate_hz-es frissítéssel.
        Az aktuális állapot alapján számít sebességparancsot.
        """
        ex = self.last_error.linear.x
        ey = self.last_error.linear.y
        ez = self.last_error.linear.z

        velocity = np.zeros(6)   # [vx, vy, vz, wx, wy, wz]

        # ── Állapotgép ─────────────────────────────────────────────────

        if self.state == AssemblyState.WAITING:
            if self.target_visible:
                self.get_logger().info("Célpont detektálva – XY igazítás kezd.")
                self.adrc.reset()
                self._transition(AssemblyState.XY_ALIGNMENT)

        elif self.state == AssemblyState.XY_ALIGNMENT:
            # ADRC vezérlő az X és Y tengelyekre
            adrc_out = self.adrc.compute(ex, ey, 0.0)
            velocity[0] = adrc_out[0]   # vx
            velocity[1] = adrc_out[1]   # vy
            # vz = 0 (még nem közelítünk)

            # Átmenet Z közelítésbe ha XY hiba kis
            if abs(ex) < self.xy_th and abs(ey) < self.xy_th:
                self.get_logger().info(
                    f"XY kész (ex={ex:.1f}px, ey={ey:.1f}px). Z közelítés."
                )
                self._transition(AssemblyState.Z_APPROACH)

        elif self.state == AssemblyState.Z_APPROACH:
            # Finomkorrekcióval közelítünk
            adrc_out = self.adrc.compute(ex, ey, 0.0)
            velocity[0] = adrc_out[0] * 0.5    # lassabb XY korrekció
            velocity[1] = adrc_out[1] * 0.5
            velocity[2] = self.app_spd          # konstans előre mozgás

            # Átmenet dokkolásba ha Z hiba elég kis
            if ez < self.z_th:
                self.get_logger().info(
                    f"Z elég közel (ez={ez:.3f}m). Dokkolás kezd."
                )
                self._transition(AssemblyState.DOCKING)

        elif self.state == AssemblyState.DOCKING:
            # Nagyon lassú lineáris beillesztés, orientáció tartásával
            velocity[2] = self.dock_spd     # csak Z irányban

            # Sikerkritérium: a Z hiba nagyon kicsi (teljesen bement)
            if ez < self.z_th * 0.3:
                self.consecutive_done += 1
                if self.consecutive_done >= 5:  # 5 egymást követő képkocka
                    self.get_logger().info("Sikeres illesztés!")
                    self._transition(AssemblyState.DONE)
            else:
                self.consecutive_done = 0

        elif self.state in (AssemblyState.DONE, AssemblyState.ERROR):
            velocity = np.zeros(6)   # nem mozog

        # ── Sebességparancs publikálás ──────────────────────────────────
        self._publish_velocity(velocity)
        self._publish_status()

    # ──────────────────────────────────────────────────────────────────
    # Segédfüggvények
    # ──────────────────────────────────────────────────────────────────

    def _transition(self, new_state: AssemblyState):
        """Állapotváltás logolással."""
        old = self.state.name
        self.state = new_state
        self.consecutive_done = 0
        self.get_logger().info(f"Állapot: {old} → {new_state.name}")

    def _publish_velocity(self, velocity: np.ndarray):
        """
        Sebességparancs küldése a robot MoveIt2 servo interfészének.

        A /servo_node/delta_twist_cmds topic-ot a MoveIt2 Servo
        komponens figyeli és valós idejű ízületi parancsokra fordítja.
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"   # end-effector frame

        msg.twist.linear.x  = velocity[0]
        msg.twist.linear.y  = velocity[1]
        msg.twist.linear.z  = velocity[2]
        msg.twist.angular.x = velocity[3]
        msg.twist.angular.y = velocity[4]
        msg.twist.angular.z = velocity[5]

        self.vel_pub.publish(msg)

    def _publish_status(self):
        """Állapot szöveg publikálása (UI / monitoring célokra)."""
        self.status_pub.publish(String(data=self.state.name))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

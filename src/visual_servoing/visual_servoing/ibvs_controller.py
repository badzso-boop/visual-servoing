"""
ibvs_controller.py
------------------
Image-Based Visual Servoing (IBVS) vezérlő implementációja
perspektív transzformáció alapján (Cao 2022).

Nincs ROS2 függőség – tisztán NumPy matematika.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class IBVSResult:
    """IBVS számítás eredménye."""
    velocity: np.ndarray    # [vx, vy, vz, wx, wy, wz] – m/s és rad/s
    error: np.ndarray       # képtérbeli hibavektor
    error_norm: float       # hiba nagysága (konvergencia figyeléséhez)
    converged: bool         # elérte-e a célpozíciót


class HomographyManager:
    """
    Homográfia mátrixok kezelése és a virtuális képsíkra vetítés.

    A kalibrációs fázisban meghatározzuk a H_a és H_b mátrixokat,
    amelyek az aljzat és a dugasz képi pontjait a virtuális síkra vetítik.
    """

    def __init__(self):
        self.H_a = None   # aljzat → virtuális sík
        self.H_b = None   # dugasz → virtuális sík
        self.calibrated = False

    def calibrate(self,
                  socket_image_points: np.ndarray,
                  socket_world_points: np.ndarray,
                  plug_image_points: np.ndarray,
                  plug_world_points: np.ndarray):
        """
        Homográfia mátrixok kalibrálása ismert pontpárok alapján.

        A kalibráláshoz a robot egy ismert pozícióba mozog,
        ahol a kamera látja mind az aljzatot, mind a dugaszt,
        és ezek pontos koordinátái ismertek.

        Args:
            socket_image_points: aljzat sarokpontjai a képen (N×2)
            socket_world_points: aljzat sarokpontjai a valóságban (N×2)
            plug_image_points:   dugasz sarokpontjai a képen (N×2)
            plug_world_points:   dugasz sarokpontjai a valóságban (N×2)
        """
        # findHomography RANSAC-ot használ az outlier-ek kiszűrésére
        self.H_a, _ = cv_findHomography(
            socket_image_points, socket_world_points
        )
        self.H_b, _ = cv_findHomography(
            plug_image_points, plug_world_points
        )
        self.calibrated = True

    def calibrate_from_reference(self,
                                  socket_ref_points: np.ndarray,
                                  plug_ref_points: np.ndarray):
        """
        Egyszerűsített kalibráció: a referencia (illesztett) pozícióból
        számítja a mátrixokat, ahol f̃_α = f̃_β feltétel teljesül.

        Ezt akkor hívjuk, amikor a robot manuálisan be van illesztve
        (tanítási fázis), és elmentjük a referencia képi jellemzőket.

        Args:
            socket_ref_points: aljzat sarokpontjai az illesztett pozícióban (4×2)
            plug_ref_points:   dugasz sarokpontjai az illesztett pozícióban (4×2)
        """
        import cv2

        # Referencia pontok: az aljzat és dugasz pontjainak egyezniük kell
        # a virtuális síkon → H mátrixok ezt biztosítják
        # Egyszerűsítés: az egységmátrixhoz közeli H mátrixokat használunk,
        # ahol a virtuális sík megegyezik a referencia kép síkjával
        self.H_a, _ = cv2.findHomography(
            socket_ref_points.astype(np.float32),
            socket_ref_points.astype(np.float32)  # referenciában önmaga
        )
        self.H_b, _ = cv2.findHomography(
            plug_ref_points.astype(np.float32),
            socket_ref_points.astype(np.float32)  # dugasz → aljzat referencia
        )
        self.calibrated = True

    def project_to_virtual(self, points: np.ndarray, H: np.ndarray) -> np.ndarray:
        """
        Képi pontok vetítése a virtuális képsíkra a H homográfia mátrixszal.

        Args:
            points: képi pontok (N×2)
            H:      3×3 homográfia mátrix
        Returns:
            Vetített pontok a virtuális síkon (N×2)
        """
        import cv2

        pts = points.reshape(-1, 1, 2).astype(np.float32)
        projected = cv2.perspectiveTransform(pts, H)
        return projected.reshape(-1, 2)

    def compute_error(self,
                      socket_points: np.ndarray,
                      plug_points: np.ndarray) -> np.ndarray:
        """
        Hibavektor számítása a virtuális képsíkon.

        e = f̃_α - f̃_β
        Ha e = 0, akkor a dugasz pontosan az aljzat előtt van.

        Args:
            socket_points: aljzat képi pontjai (4×2)
            plug_points:   dugasz képi pontjai (4×2)
        Returns:
            Hibavektor (8,) – mind a 4 sarokpont x,y eltérése
        """
        socket_virtual = self.project_to_virtual(socket_points, self.H_a)
        plug_virtual   = self.project_to_virtual(plug_points,   self.H_b)

        # Hibavektor: célpozíció - aktuális pozíció
        error = socket_virtual - plug_virtual
        return error.flatten()  # (8,) vektor


def cv_findHomography(src_points: np.ndarray,
                      dst_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """OpenCV findHomography wrapper – könnyebb mockoláshoz tesztekben."""
    import cv2
    H, mask = cv2.findHomography(
        src_points.astype(np.float32),
        dst_points.astype(np.float32),
        cv2.RANSAC,
        5.0
    )
    return H, mask


class IBVSController:
    """
    Image-Based Visual Servoing vezérlő.

    A vezérlési törvény: v_c = -λ * L⁺ * e
    ahol:
        v_c – a kamera (end-effector) sebességparancs
        λ   – vezérlési erősítés
        L⁺  – interakciós mátrix pszeudo-inverze
        e   – képtérbeli hibavektor
    """

    def __init__(self, config: dict = None):
        cfg = config or {}

        # Vezérlési erősítés (gain) – ha nagy: gyors de instabil, ha kicsi: lassú
        self.lam = cfg.get("gain", 0.3)

        # Kamera intrinsic paraméterek (kalibrációból)
        self.fx = cfg.get("fx", 600.0)   # fókusztávolság x irányban
        self.fy = cfg.get("fy", 600.0)   # fókusztávolság y irányban
        self.cx = cfg.get("cx", 640.0)   # főpont x
        self.cy = cfg.get("cy", 360.0)   # főpont y

        # Konvergencia küszöb (pixel)
        self.convergence_threshold = cfg.get("convergence_threshold", 5.0)

        # Max sebesség korlátok (biztonság!)
        self.max_linear_vel  = cfg.get("max_linear_vel",  0.05)   # m/s
        self.max_angular_vel = cfg.get("max_angular_vel", 0.3)    # rad/s

        # Becsült mélység (Z) – ha nincs mélységi kamera, fix értéket használunk
        # és a depth_estimator frissíti
        self.Z_estimate = cfg.get("initial_depth", 0.5)  # méter

    def update_depth_estimate(self, z: float):
        """
        Mélység becslés frissítése (pl. RealSense mélységi csatornából
        vagy jellemzőméret alapú becslésből).
        """
        self.Z_estimate = max(0.05, z)  # biztonsági minimum

    def compute_interaction_matrix(self,
                                    points: np.ndarray,
                                    Z: float = None) -> np.ndarray:
        """
        Interakciós mátrix (Image Jacobian) számítása.

        Minden képi ponthoz egy 2×6-os részleges interakciós mátrix tartozik.
        N pont esetén az eredmény (2N)×6 méretű.

        A formula egy (u, v) képi pontra:
            L = [-fx/Z,    0,  u/Z,  u*v/fx,    -(fx²+u²)/fx,  v  ]
                [   0,  -fy/Z, v/Z,  (fy²+v²)/fy,  -u*v/fy,   -u  ]

        Args:
            points: képi pontok NORMALIZÁLT koordinátákban (N×2)
                    (normalizált = főponttól mérve, osztva a fókusztávolsággal)
            Z:      mélység méterben
        Returns:
            Interakciós mátrix (2N × 6)
        """
        Z = Z or self.Z_estimate
        N = len(points)
        L = np.zeros((2 * N, 6))

        for i, (u, v) in enumerate(points):
            # Normalizált koordináták → a főponttól mérve, osztva fx,fy-val
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy

            row = 2 * i
            # x irányú sor
            L[row, 0] = -1.0 / Z
            L[row, 1] = 0.0
            L[row, 2] = x / Z
            L[row, 3] = x * y
            L[row, 4] = -(1.0 + x ** 2)
            L[row, 5] = y

            # y irányú sor
            L[row+1, 0] = 0.0
            L[row+1, 1] = -1.0 / Z
            L[row+1, 2] = y / Z
            L[row+1, 3] = 1.0 + y ** 2
            L[row+1, 4] = -x * y
            L[row+1, 5] = -x

        return L

    def compute_velocity(self,
                          error: np.ndarray,
                          feature_points: np.ndarray) -> IBVSResult:
        """
        Sebességparancs számítása IBVS törvény alapján.

        v_c = -λ * L⁺ * e

        Args:
            error:          hibavektor (2N,) – virtuális képsíkon számított
            feature_points: aktuális képi pontok (N×2) – interakciós mátrixhoz
        Returns:
            IBVSResult – sebességparancs és metrikák
        """
        error_norm = np.linalg.norm(error)
        converged = error_norm < self.convergence_threshold

        if converged:
            return IBVSResult(
                velocity=np.zeros(6),
                error=error,
                error_norm=error_norm,
                converged=True
            )

        # Interakciós mátrix és pszeudo-inverze
        L = self.compute_interaction_matrix(feature_points, self.Z_estimate)

        # SVD alapú pszeudo-inverz (numerikusan stabil)
        # L⁺ = (L^T L)^{-1} L^T
        L_pinv = np.linalg.pinv(L)

        # Sebességparancs: v = -λ * L⁺ * e
        velocity = -self.lam * L_pinv @ error

        # Biztonsági sebesség korlátozás
        velocity = self._clamp_velocity(velocity)

        return IBVSResult(
            velocity=velocity,
            error=error,
            error_norm=error_norm,
            converged=False
        )

    def _clamp_velocity(self, velocity: np.ndarray) -> np.ndarray:
        """
        Sebességparancs korlátozása a biztonsági határokhoz.
        """
        clamped = velocity.copy()

        # Lineáris sebességek (vx, vy, vz)
        linear = clamped[:3]
        linear_norm = np.linalg.norm(linear)
        if linear_norm > self.max_linear_vel:
            clamped[:3] = linear * (self.max_linear_vel / linear_norm)

        # Szögsebességek (wx, wy, wz)
        angular = clamped[3:]
        angular_norm = np.linalg.norm(angular)
        if angular_norm > self.max_angular_vel:
            clamped[3:] = angular * (self.max_angular_vel / angular_norm)

        return clamped


class DepthEstimator:
    """
    Mélység (Z irányú távolság) becslése a célpont látszó méretéből.

    Ha nincs mélységi kamera, a mélység a csatlakozóaljzat
    képen látszó területéből becsülhető:

        Z ≈ f * D_valódi / D_képen

    ahol D_valódi az aljzat ismert fizikai mérete (mm),
    D_képen pedig a képen mért mérete (pixel).
    """

    def __init__(self, focal_length_px: float, real_size_mm: float):
        """
        Args:
            focal_length_px: fókusztávolság pixelben (kalibrációból)
            real_size_mm:    az aljzat valódi átmérője/oldalhossza mm-ben
        """
        self.f = focal_length_px
        self.D_real = real_size_mm / 1000.0  # méterbe konvertálva

    def estimate(self, apparent_size_px: float) -> float:
        """
        Mélység becslése a látszó méretből.

        Args:
            apparent_size_px: az aljzat látszó átmérője/oldalhossza pixelben
        Returns:
            Becsült mélység méterben
        """
        if apparent_size_px < 1.0:
            return 1.0  # alapértelmezett ha nem detektálható

        Z = (self.f * self.D_real) / apparent_size_px
        return float(np.clip(Z, 0.05, 2.0))   # 5cm – 2m tartomány

    def estimate_from_area(self, area_px: float) -> float:
        """
        Alternatív: területből becsül (kevésbé pontosan, de zajra robusztusabb).
        """
        if area_px < 1.0:
            return 1.0

        # A terület négyzetgyöke arányos az oldalhosszal
        apparent_size = np.sqrt(area_px)
        return self.estimate(apparent_size)

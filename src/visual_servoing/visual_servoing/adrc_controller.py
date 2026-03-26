"""
adrc_controller.py
------------------
Active Disturbance Rejection Control (ADRC) implementáció.

Lineáris ESO (Extended State Observer) + PD szabályozó.
Minden vezérlési tengelyhez (x, y, z) külön ADRC példány fut.

Referencia: Han J. "From PID to Active Disturbance Rejection Control"
            IEEE Transactions on Industrial Electronics, 2009.
"""

import numpy as np


class LinearESO:
    """
    Lineáris Kiterjesztett Állapotmegfigyelő (Linear Extended State Observer).

    A rendszert másodfokú integrátorként modellezi:
        ẋ₁ = x₂
        ẋ₂ = f(t) + b₀ * u

    ahol f(t) az összes zavar és modellezési hiba összessége,
    amelyet az ESO becsül (z₃ ≈ f(t)).

    Állapotok:
        z₁ ≈ y   (kimenet becslés)
        z₂ ≈ ẏ   (sebesség becslés)
        z₃ ≈ f   (teljes zavar becslés)
    """

    def __init__(self, bandwidth: float, b0: float, dt: float):
        """
        Args:
            bandwidth: observer sávszélesség (ω_o) rad/s-ban
                       Általános szabály: ω_o = 3-5 × zárt hurok sávszélesség
            b0:        bemenet erősítés becslés
                       (a rendszer b0 * u tagjához)
            dt:        mintavételi idő másodpercben
        """
        self.b0 = b0
        self.dt = dt

        # ESO erősítések a sávszélességből számítva
        # (karakterisztikus polinom: (s + ω_o)³)
        w = bandwidth
        self.beta1 = 3 * w          # 3ω_o
        self.beta2 = 3 * w ** 2     # 3ω_o²
        self.beta3 = w ** 3         # ω_o³

        # Állapotok inicializálása
        self.z1 = 0.0   # becsült kimenet
        self.z2 = 0.0   # becsült sebesség
        self.z3 = 0.0   # becsült zavar

    def reset(self):
        """Állapotok visszaállítása (pl. újraindításkor)."""
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0

    def update(self, y: float, u: float) -> tuple:
        """
        ESO frissítése egy mintavételi lépéssel (Euler integrálás).

        Args:
            y: mért kimenet (pl. képi koordináta hibája)
            u: alkalmazott vezérlőjel
        Returns:
            (z1, z2, z3) – becsült állapotok
        """
        # Megfigyelési hiba
        e_obs = self.z1 - y

        # ESO differenciálegyenletek (diszkrét Euler lépés)
        dz1 = self.z2 - self.beta1 * e_obs
        dz2 = self.z3 - self.beta2 * e_obs + self.b0 * u
        dz3 =         - self.beta3 * e_obs

        # Euler integrálás
        self.z1 += self.dt * dz1
        self.z2 += self.dt * dz2
        self.z3 += self.dt * dz3

        return self.z1, self.z2, self.z3


class ADRCController:
    """
    ADRC vezérlő egy tengelyre.

    Felépítés:
        1. ESO becsüli a kimenet, sebesség és zavar értékét
        2. PD típusú szabályozó számítja a nominális vezérlőjelet
        3. A zavarkompenzsáció kivonja a becsült zavarokat

    A vezérlőjel: u = (u₀ - z₃) / b₀
    ahol: u₀ = kp * (r - z₁) + kd * (ṙ - z₂)
    """

    def __init__(self,
                 observer_bandwidth: float,
                 control_bandwidth: float,
                 b0: float,
                 dt: float):
        """
        Args:
            observer_bandwidth: ESO sávszélesség (ω_o) – általában 5-10× ω_c
            control_bandwidth:  zárt hurok sávszélesség (ω_c)
            b0:                 bemenet erősítés becslés
            dt:                 mintavételi idő (s)
        """
        self.dt = dt
        self.b0 = b0

        # ESO inicializálása
        self.eso = LinearESO(
            bandwidth=observer_bandwidth,
            b0=b0,
            dt=dt
        )

        # PD erősítések a zárt hurok sávszélességből (kritikusan csillapított)
        wc = control_bandwidth
        self.kp = wc ** 2      # ω_c²
        self.kd = 2 * wc       # 2ω_c

        # Kimeneti korlátozás
        self.output_limit = None

        # Előző vezérlőjel (ESO feedback-hez)
        self._last_u = 0.0

    def set_output_limit(self, limit: float):
        """Vezérlőjel amplitúdó korlátozása."""
        self.output_limit = abs(limit)

    def reset(self):
        """Vezérlő visszaállítása."""
        self.eso.reset()
        self._last_u = 0.0

    def compute(self, reference: float, measurement: float,
                 reference_dot: float = 0.0) -> float:
        """
        Vezérlőjel számítása egy lépésre.

        Args:
            reference:     kívánt érték (r)
            measurement:   mért kimenet (y)
            reference_dot: referencia deriváltja (ṙ), általában 0
        Returns:
            Vezérlőjel (u)
        """
        # ESO frissítése az előző vezérlőjellel
        z1, z2, z3 = self.eso.update(measurement, self._last_u)

        # Hibák az ESO becsléseivel
        e1 = reference - z1       # pozíció hiba
        e2 = reference_dot - z2   # sebesség hiba

        # Nominális vezérlőjel (PD rész)
        u0 = self.kp * e1 + self.kd * e2

        # Zavar kompenzáció
        u = (u0 - z3) / self.b0

        # Korlátozás (ha be van állítva)
        if self.output_limit is not None:
            u = np.clip(u, -self.output_limit, self.output_limit)

        self._last_u = u
        return float(u)

    @property
    def disturbance_estimate(self) -> float:
        """Az aktuálisan becsült zavar értéke (diagnosztikához)."""
        return self.eso.z3


class MultiAxisADRC:
    """
    Több tengelyes ADRC vezérlő.

    A rendszerünkben 3 tengelyre kell: x, y, z képi eltérés.
    Minden tengelynek saját ADRC példánya van (ezek egymástól
    függetlenek, mivel az IBVS-ben a tengelyek jól szeparáltak).
    """

    def __init__(self, config: dict = None):
        """
        Args:
            config: YAML-ből betölthető paraméterek szótára
        """
        cfg = config or {}

        dt   = cfg.get("dt",                   0.033)   # ~30 Hz
        w_o  = cfg.get("observer_bandwidth",   50.0)    # rad/s
        w_c  = cfg.get("control_bandwidth",    10.0)    # rad/s
        b0   = cfg.get("b0",                    1.0)

        self.controllers = {
            "x": ADRCController(w_o, w_c, b0, dt),
            "y": ADRCController(w_o, w_c, b0, dt),
            "z": ADRCController(w_o, w_c, b0, dt),
        }

        # Sebesség korlátok tengelyenként [m/s]
        self.controllers["x"].set_output_limit(cfg.get("max_vx", 0.05))
        self.controllers["y"].set_output_limit(cfg.get("max_vy", 0.05))
        self.controllers["z"].set_output_limit(cfg.get("max_vz", 0.02))

    def compute(self,
                error_x: float,
                error_y: float,
                error_z: float) -> np.ndarray:
        """
        Sebességparancs számítása mind a 3 tengelyre.

        Args:
            error_x: képtérbeli x eltérés (pixel → átskálázandó m/s-ra)
            error_y: képtérbeli y eltérés
            error_z: mélységi eltérés
        Returns:
            [vx, vy, vz] sebességparancs (m/s)
        """
        # Az ADRC bemenete a mért hiba (0 a referencia, error a mérés)
        vx = self.controllers["x"].compute(reference=0.0, measurement=-error_x)
        vy = self.controllers["y"].compute(reference=0.0, measurement=-error_y)
        vz = self.controllers["z"].compute(reference=0.0, measurement=-error_z)

        return np.array([vx, vy, vz])

    def reset(self):
        """Minden vezérlő visszaállítása."""
        for ctrl in self.controllers.values():
            ctrl.reset()

    @property
    def disturbances(self) -> dict:
        """Becsült zavarok tengelyenként (diagnosztikához)."""
        return {
            axis: ctrl.disturbance_estimate
            for axis, ctrl in self.controllers.items()
        }

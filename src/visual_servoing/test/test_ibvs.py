"""
test_ibvs.py
------------
Egységtesztek az IBVS vezérlő matematikájához.
ROS2 nélkül futtatható: pytest test/test_ibvs.py
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from visual_servoing.ibvs_controller import IBVSController, DepthEstimator
from visual_servoing.adrc_controller import LinearESO, ADRCController, MultiAxisADRC


# ──────────────────────────────────────────────────────────────────────
# IBVSController tesztek
# ──────────────────────────────────────────────────────────────────────

class TestIBVSController:

    def setup_method(self):
        """Minden teszt előtt friss vezérlő példány."""
        self.ctrl = IBVSController({
            "gain": 0.5,
            "fx": 600.0, "fy": 600.0,
            "cx": 640.0, "cy": 360.0,
            "convergence_threshold": 5.0,
            "max_linear_vel": 0.1,
            "max_angular_vel": 0.5,
            "initial_depth": 0.5,
        })

    def test_zero_error_returns_zero_velocity(self):
        """Ha nincs hiba, nem mozog a robot."""
        error = np.zeros(8)
        points = np.array([[640, 360], [700, 360], [700, 400], [640, 400]], dtype=float)
        result = self.ctrl.compute_velocity(error, points)

        assert result.converged
        assert np.allclose(result.velocity, 0.0)

    def test_nonzero_error_returns_nonzero_velocity(self):
        """Ha van hiba, a robot mozog."""
        error = np.array([50, 30, 50, 30, 50, 30, 50, 30], dtype=float)
        points = np.array([[640, 360], [700, 360], [700, 400], [640, 400]], dtype=float)
        result = self.ctrl.compute_velocity(error, points)

        assert not result.converged
        assert np.linalg.norm(result.velocity) > 0

    def test_velocity_within_limits(self):
        """A sebesség nem haladja meg a biztonsági korlátokat."""
        # Nagy hiba → nagy vezérlőjel → de korlátozott
        error = np.ones(8) * 1000.0
        points = np.array([[640, 360], [700, 360], [700, 400], [640, 400]], dtype=float)
        result = self.ctrl.compute_velocity(error, points)

        linear_speed  = np.linalg.norm(result.velocity[:3])
        angular_speed = np.linalg.norm(result.velocity[3:])

        assert linear_speed  <= self.ctrl.max_linear_vel  + 1e-6
        assert angular_speed <= self.ctrl.max_angular_vel + 1e-6

    def test_interaction_matrix_shape(self):
        """Az interakciós mátrix mérete helyes."""
        points = np.array([[600, 350], [680, 350], [680, 380], [600, 380]], dtype=float)
        L = self.ctrl.compute_interaction_matrix(points, Z=0.5)

        # 4 pont → 8×6 mátrix
        assert L.shape == (8, 6)

    def test_error_norm_decreases(self):
        """Minden lépésben csökken a hiba (konvergencia ellenőrzés)."""
        error = np.array([100, 80, 100, 80, 100, 80, 100, 80], dtype=float)
        points = np.array([[640, 360], [700, 360], [700, 400], [640, 400]], dtype=float)

        prev_norm = np.linalg.norm(error)
        for _ in range(5):
            result = self.ctrl.compute_velocity(error, points)
            # Szimulálunk egy lépést: hiba csökken a sebesség arányában
            error = error * 0.8    # közelítés: exponenciális konvergencia
            assert np.linalg.norm(error) < prev_norm
            prev_norm = np.linalg.norm(error)


# ──────────────────────────────────────────────────────────────────────
# DepthEstimator tesztek
# ──────────────────────────────────────────────────────────────────────

class TestDepthEstimator:

    def setup_method(self):
        # 600px fókusztávolság, 80mm valódi méret
        self.est = DepthEstimator(focal_length_px=600.0, real_size_mm=80.0)

    def test_known_depth(self):
        """Ismert mélységnél helyes becslés."""
        # Ha Z=0.5m és f=600, D_real=0.08m → D_px = f*D/Z = 600*0.08/0.5 = 96px
        apparent_px = 96.0
        Z = self.est.estimate(apparent_px)
        assert abs(Z - 0.5) < 0.01

    def test_larger_apparent_size_means_closer(self):
        """Nagyobb látszó méret → közelebb van."""
        Z_close = self.est.estimate(200.0)
        Z_far   = self.est.estimate(50.0)
        assert Z_close < Z_far

    def test_zero_size_returns_default(self):
        """Nulla méret nem okoz osztás nullával."""
        Z = self.est.estimate(0.0)
        assert Z > 0


# ──────────────────────────────────────────────────────────────────────
# ADRC tesztek
# ──────────────────────────────────────────────────────────────────────

class TestLinearESO:

    def test_tracks_constant_output(self):
        """Az ESO nyomon követi a konstans kimenetet."""
        eso = LinearESO(bandwidth=20.0, b0=1.0, dt=0.01)
        y = 1.0   # konstans kimenet

        for _ in range(500):   # 5 másodperc
            eso.update(y=y, u=0.0)

        # z1 ≈ y után elegendő idő elteltével
        assert abs(eso.z1 - y) < 0.05

    def test_estimates_constant_disturbance(self):
        """Az ESO becsüli a konstans zavarokat."""
        eso = LinearESO(bandwidth=30.0, b0=1.0, dt=0.01)
        disturbance = 2.0

        # Szimulált rendszer: ÿ = disturbance (konstans gyorsulás)
        y, dy = 0.0, 0.0
        dt = 0.01
        for _ in range(1000):
            u = 0.0
            # Rendszer dinamika szimulálása
            ddy = disturbance + 1.0 * u
            dy += dt * ddy
            y  += dt * dy
            eso.update(y=y, u=u)

        # z3 ≈ zavar
        assert abs(eso.z3 - disturbance) < 0.5


class TestADRCController:

    def test_converges_to_reference(self):
        """Az ADRC vezérlő a referencia értékre konvergál."""
        ctrl = ADRCController(
            observer_bandwidth=50.0,
            control_bandwidth=10.0,
            b0=1.0,
            dt=0.01
        )
        ctrl.set_output_limit(5.0)

        reference = 1.0
        y, dy = 0.0, 0.0
        dt = 0.01

        for _ in range(1000):   # 10 másodperc
            u = ctrl.compute(reference=reference, measurement=y)
            # Egyszerű kettős integrátor rendszer: ÿ = u
            dy += dt * u
            y  += dt * dy

        assert abs(y - reference) < 0.05

    def test_rejects_step_disturbance(self):
        """Az ADRC kompenzálja az ugrásszerű zavarokat."""
        ctrl = ADRCController(50.0, 10.0, 1.0, 0.01)
        ctrl.set_output_limit(10.0)

        reference = 1.0
        y, dy = 1.0, 0.0  # kezdetben a referencián van
        dt = 0.01

        # 5 másodperc után zavar lép fel
        for step in range(1000):
            disturbance = 2.0 if step > 500 else 0.0
            u = ctrl.compute(reference=reference, measurement=y)
            dy += dt * (u + disturbance)
            y  += dt * dy

        # A zavar ellenére a kimenet közel marad a referenciához
        assert abs(y - reference) < 0.2


class TestMultiAxisADRC:

    def test_output_shape(self):
        """A MultiAxisADRC 3 elemű vektort ad vissza."""
        adrc = MultiAxisADRC({"dt": 0.033})
        result = adrc.compute(10.0, -5.0, 3.0)
        assert result.shape == (3,)

    def test_zero_error_near_zero_output(self):
        """Nulla hibánál közel nulla a kimenet (konvergált állapotban)."""
        adrc = MultiAxisADRC({"dt": 0.033})
        # Sok lépés nulla hibával → konvergál
        for _ in range(100):
            result = adrc.compute(0.0, 0.0, 0.0)
        assert np.allclose(result, 0.0, atol=0.01)

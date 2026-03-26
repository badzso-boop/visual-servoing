"""
feature_detector.py
-------------------
A fali csatlakozóaljzat és a robotkar végén lévő dugasz
detektálása OpenCV segítségével.

Nincs ROS2 függőség – tisztán OpenCV + NumPy.
Így külön is tesztelhető, unit tesztelható.
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class DetectionResult:
    """Egy detektálás eredménye."""
    center: np.ndarray        # [u, v] – középpont pixelben
    corners: np.ndarray       # (4, 2) – sarokpontok pixelben
    area: float               # területe pixelben²
    success: bool             # sikerült-e detektálni


class ConnectorDetector:
    """
    Fali csatlakozóaljzat detektálása él- és kontúrdetektálással.

    A csatlakozó téglalap alakú, fehér/bézs színű, jellemzően
    kb. 80x80 mm méretű. A kamera távolságától függ a képi mérete.
    """

    def __init__(self, config: dict = None):
        """
        Paraméterek:
            config: hangolható paraméterek szótára (ld. alább)
        """
        cfg = config or {}

        # Canny éldetektálás küszöbök
        self.canny_low  = cfg.get("canny_low",  40)
        self.canny_high = cfg.get("canny_high", 120)

        # Kontúr szűrési paraméterek
        self.min_area   = cfg.get("min_area",   500)    # px²
        self.max_area   = cfg.get("max_area",   50000)  # px²
        self.min_ratio  = cfg.get("min_ratio",  0.5)    # szélesség/magasság
        self.max_ratio  = cfg.get("max_ratio",  2.0)

        # CLAHE beállítások (inhomogén fény kezelése)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Kép előfeldolgozása: szürkeárnyalat, zajszűrés, kontrasztjavítás.

        Args:
            image: BGR kép (OpenCV alapértelmezett formátum)
        Returns:
            Előfeldolgozott szürkeárnyalatos kép
        """
        # Szürkeárnyalatossá alakítás
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Gaussian zajszűrés (5x5-ös kernel)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)

        # CLAHE – adaptív hisztogram-egyenlítés
        # Segít ha a megvilágítás egyenetlen (árnyékok, ablak)
        enhanced = self.clahe.apply(blurred)

        return enhanced

    def detect(self, image: np.ndarray) -> DetectionResult:
        """
        Csatlakozóaljzat detektálása a képen.

        Args:
            image: BGR kép
        Returns:
            DetectionResult – tartalmazza a pozíciót és sarokpontokat
        """
        processed = self.preprocess(image)

        # Canny éldetektálás
        edges = cv2.Canny(processed, self.canny_low, self.canny_high)

        # Morfológiai lezárás – kis hézagok kitöltése az élekben
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Kontúrok keresése (csak külső kontúrokat keresünk)
        contours, _ = cv2.findContours(
            closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best = self._find_best_contour(contours)

        if best is None:
            return DetectionResult(
                center=np.array([0.0, 0.0]),
                corners=np.zeros((4, 2)),
                area=0.0,
                success=False
            )

        # Sarokpontok meghatározása
        corners = self._extract_corners(best)
        center = corners.mean(axis=0)
        area = cv2.contourArea(best)

        return DetectionResult(
            center=center,
            corners=corners,
            area=area,
            success=True
        )

    def _find_best_contour(self, contours) -> Optional[np.ndarray]:
        """
        A legjobb (csatlakozónak megfelelő) kontúr kiválasztása
        terület és arány alapján.
        """
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Terület szűrés
            if not (self.min_area < area < self.max_area):
                continue

            # Határoló téglalap arány szűrés
            _, (w, h), _ = cv2.minAreaRect(cnt)
            if h == 0:
                continue
            ratio = w / h
            if not (self.min_ratio < ratio < self.max_ratio):
                continue

            # Konvexitás ellenőrzés – a csatlakozó közel konvex
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = area / hull_area
            if solidity < 0.7:  # legalább 70% tele kell legyen
                continue

            candidates.append(cnt)

        if not candidates:
            return None

        # A legnagyobb területű jelölt a legjobb
        return max(candidates, key=cv2.contourArea)

    def _extract_corners(self, contour: np.ndarray) -> np.ndarray:
        """
        4 sarokpont kinyerése a kontúrból.
        Megpróbálja négyszögként közelíteni, ha nem sikerül
        a határoló téglalap sarkait adja vissza.

        Returns:
            (4, 2) alakú numpy tömb – sarokpontok [u, v] sorrendben
        """
        # Kontúr kerülete
        peri = cv2.arcLength(contour, True)

        # Poligon közelítés (2%-os tolerancia)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

        if len(approx) == 4:
            corners = approx.reshape(4, 2).astype(float)
        else:
            # Határoló téglalapból számítjuk a sarokpontokat
            rect = cv2.minAreaRect(contour)
            corners = cv2.boxPoints(rect).astype(float)

        # Rendezés: bal-felső, jobb-felső, jobb-alsó, bal-alsó sorrendbe
        corners = self._sort_corners(corners)
        return corners

    def _sort_corners(self, corners: np.ndarray) -> np.ndarray:
        """
        Sarokpontok rendezése az óramutató járásával megegyező sorrendbe:
        [bal-felső, jobb-felső, jobb-alsó, bal-alsó]
        """
        # Összeg és különbség alapján rendezés
        s = corners.sum(axis=1)
        d = np.diff(corners, axis=1).flatten()

        sorted_corners = np.zeros((4, 2), dtype=float)
        sorted_corners[0] = corners[np.argmin(s)]  # bal-felső (legkisebb összeg)
        sorted_corners[2] = corners[np.argmax(s)]  # jobb-alsó (legnagyobb összeg)
        sorted_corners[1] = corners[np.argmin(d)]  # jobb-felső (legkisebb különbség)
        sorted_corners[3] = corners[np.argmax(d)]  # bal-alsó (legnagyobb különbség)

        return sorted_corners

    def draw_debug(self, image: np.ndarray, result: DetectionResult) -> np.ndarray:
        """
        Debug vizualizáció: rajzolja rá a képre a detektált aljzatot.

        Args:
            image: eredeti BGR kép
            result: detektálás eredménye
        Returns:
            Annotált BGR kép
        """
        debug = image.copy()

        if not result.success:
            cv2.putText(debug, "NO DETECTION", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return debug

        # Sarokpontok és élvonalak rajzolása
        corners = result.corners.astype(int)
        for i in range(4):
            cv2.line(debug, tuple(corners[i]), tuple(corners[(i+1) % 4]),
                     (0, 255, 0), 2)
            cv2.circle(debug, tuple(corners[i]), 6, (255, 0, 0), -1)

        # Középpont
        cx, cy = result.center.astype(int)
        cv2.circle(debug, (cx, cy), 8, (0, 0, 255), -1)
        cv2.putText(debug, f"({cx}, {cy})", (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return debug


class PlugDetector:
    """
    A robotkar végén lévő dugasz detektálása.

    Kétféle módszer támogatott:
    - Szín alapú (ha a dugaszon egyedi színű jelölő van)
    - Kör alapú (Hough transzformáció, ha a dugasz vége kör alakú)
    """

    def __init__(self, method: str = "color", config: dict = None):
        """
        Args:
            method: "color" vagy "circle"
            config: hangolható paraméterek
        """
        self.method = method
        cfg = config or {}

        # Szín alapú detektálás (HSV tartomány – alapértelmezett: kék)
        self.color_lower = np.array(cfg.get("hsv_lower", [100, 150, 50]))
        self.color_upper = np.array(cfg.get("hsv_upper", [130, 255, 255]))

        # Kör detektálás paraméterek
        self.min_radius = cfg.get("min_radius", 10)
        self.max_radius = cfg.get("max_radius", 60)

    def detect(self, image: np.ndarray) -> DetectionResult:
        """
        Dugasz detektálása a választott módszerrel.
        """
        if self.method == "color":
            return self._detect_by_color(image)
        else:
            return self._detect_by_circle(image)

    def _detect_by_color(self, image: np.ndarray) -> DetectionResult:
        """Szín alapú detektálás HSV színtérben."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # Morfológiai tisztítás
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return DetectionResult(np.zeros(2), np.zeros((4, 2)), 0.0, False)

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)

        if area < 100:
            return DetectionResult(np.zeros(2), np.zeros((4, 2)), 0.0, False)

        M = cv2.moments(best)
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        center = np.array([cx, cy])

        # Közelítő sarkok (bounding box sarkaiból)
        x, y, w, h = cv2.boundingRect(best)
        corners = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h]], dtype=float)

        return DetectionResult(center, corners, area, True)

    def _detect_by_circle(self, image: np.ndarray) -> DetectionResult:
        """Hough kör transzformáció alapú detektálás."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=100,
            param2=30,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )

        if circles is None:
            return DetectionResult(np.zeros(2), np.zeros((4, 2)), 0.0, False)

        # A legjobb (legmagasabb szavazatszámú) kör
        circles = np.round(circles[0, :]).astype(int)
        cx, cy, r = circles[0]

        center = np.array([cx, cy], dtype=float)
        area = np.pi * r ** 2

        # Határoló négyzet sarkainak becslése
        corners = np.array([
            [cx - r, cy - r],
            [cx + r, cy - r],
            [cx + r, cy + r],
            [cx - r, cy + r]
        ], dtype=float)

        return DetectionResult(center, corners, area, True)
